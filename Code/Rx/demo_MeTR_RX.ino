/*
  ESP32 Web OTA + LoRa Receiver Dashboard + LoRa Config
  3_WebOTA_LoRa_Receiver.ino
  Companion receiver for 2_WebOTA_GPS_LoRa.ino (the GPS/sender node)
  Base: DroneBot Workshop 2026 ArduinoOTA demo (OTA path since replaced)
  https://dronebotworkshop.com

  Board: ESP32 dev board (no GPS on this node)

  Features:
    - SX1278 (RA-02) LoRa RX via VSPI (SCK18/MISO19/MOSI23/NSS5/RST26/DIO0 4)
      -- identical pinout to the sender node, so a wired-up sender's config
         (freq/SF/BW/CR) can be copied straight across for the link to work.
    - Local web server (no cloud, no Arduino IDE network step):
        /             -> live LoRa RX dashboard + LoRa config
        /rx.json      -> received message log + link stats (GET)
        /lora.json    -> current LoRa config (GET)
        /lora/set     -> apply new LoRa config (POST form)
        /update       -> ElegantOTA .bin upload page

  IMPORTANT: freq/SF/BW/CR must match the sender's configuration exactly
  for packets to be received at all -- LoRa demodulation depends on both
  ends agreeing on those four parameters. TX power only affects the
  sender's output, receiver doesn't need to match that one.

  Requires (Library Manager):
    - LoRa (Sandeep Mistry)
    - ElegantOTA (Ayush Sharma) -- v3, "Async Mode: false" (uses WebServer, not AsyncWebServer)

  NOTE: Move ssid/password into a separate secrets.h (gitignored)
  before committing this — don't ship Wi-Fi credentials in-repo.
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <SPI.h>
#include <LoRa.h>

// ---------------- Network Credentials ----------------
const char* ssid = "DIGI-34Cz";
const char* password = "HH9F23RUzA";

WebServer server(80);

// ---------------- LoRa (SX1278 / RA-02) ----------------
// Same physical pinout as the sender node -- keep these identical across
// both boards so wiring instructions stay copy-paste between them.
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_NSS   5
#define LORA_DIO0  4
#define LORA_RST   26

// Runtime-adjustable LoRa parameters (defaults on boot)
// Must match the sender's freq/SF/BW/CR/sync for packets to demodulate at all.
long loraFrequency   = 433E6;  // Hz
int  loraSF          = 7;      // spreading factor 6-12
long loraBandwidth   = 125E3;  // Hz
int  loraCodingRate  = 5;      // 4/5 .. 4/8 -> pass 5-8
int  loraTxPower     = 17;     // dBm, 2-20 (unused for RX-only, kept for parity/future TX replies)
// Sync word: must match the sender exactly or packets silently fail to
// receive. 0x12 is the SX127x LoRa library's own default.
uint8_t loraSyncWord = 0x12;

// ---------------- Received message log ----------------
// Small ring buffer of recent packets, newest first when read out.
#define RX_LOG_SIZE 20
struct RxEntry {
  String msg;
  int    rssi;
  float  snr;
  unsigned long atMillis;
};
RxEntry rxLog[RX_LOG_SIZE];
int  rxLogHead  = 0;   // next slot to write
int  rxLogCount = 0;   // how many valid entries so far (caps at RX_LOG_SIZE)
unsigned int rxTotalCount = 0;
unsigned long lastRxMillis = 0;

// ---------------- Radio health / error state ----------------
// Tracks what's actually happening at the LoRa hardware level, separate
// from the packet log, so the dashboard can tell "no packets yet" apart
// from "the radio never came up" or "packets are arriving corrupted."
bool loraOk = false;               // true only after a successful LoRa.begin()
String loraLastError = "";         // human-readable last failure, empty if none
unsigned long loraLastErrorMillis = 0;
unsigned int loraInitFailCount = 0;
unsigned int loraRxErrorCount = 0; // corrupted/empty packets, CRC issues, etc.

// Set by the LoRa receive interrupt/callback; drained in loop() so we
// never do String work or touch the web server state from inside the
// callback itself.
volatile bool packetPending = false;

// ---------------- Transceiver / ping state ----------------
// Same packet framing as the sender node:
//   'P'        ping        (this board -> sender, manual troubleshoot button)
//   'A'        ack/pong    (sender -> this board, reply to our ping)
//   'G:<data>' GPS fix     (sender -> this board)
//   'K:<seq>'  GPS-ack     (this board -> sender, auto-sent on receiving 'G')
//   'T:<msg>'  free text   (sender -> this board, antenna-test message)
bool pingWaitingForAck = false;
unsigned long pingWaitStartMillis = 0;
unsigned long lastPingRttMs = 0;
bool lastPingOk = false;
unsigned int pingSentCount = 0;
unsigned int pingAckCount = 0;
#define PING_TIMEOUT_MS 3000

// Most recent parsed GPS fix received from the sender (separate from the
// raw rxLog, which keeps every packet type for the troubleshooting table)
struct GpsRxData {
  bool valid = false;
  unsigned int seq = 0;
  float lat = 0, lon = 0, alt = 0;
  int sat = 0;
  unsigned long atMillis = 0;
};
GpsRxData lastGpsRx;
unsigned int gpsRxCount = 0;

// ---------------- LoRa init/apply helper ----------------
bool applyLoRaConfig() {
  LoRa.end();  // safe even if not started yet in most impls after begin() once
  loraOk = false;

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(loraFrequency)) {
    loraLastError = "LoRa.begin() failed -- check SPI wiring (SCK18/MISO19/MOSI23/NSS5/RST26) and module power.";
    loraLastErrorMillis = millis();
    loraInitFailCount++;
    loraOk = false;
    Serial.println(loraLastError);
    return false;
  }

  LoRa.setSpreadingFactor(loraSF);
  LoRa.setSignalBandwidth(loraBandwidth);
  LoRa.setCodingRate4(loraCodingRate);
  LoRa.setTxPower(loraTxPower);
  LoRa.setSyncWord(loraSyncWord);

  // Put the radio into continuous receive mode and re-arm the callback
  // every time config changes (LoRa.end() above drops it).
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

  loraOk = true;
  Serial.println("LoRa configured: freq=" + String(loraFrequency) +
                  " SF=" + String(loraSF) +
                  " BW=" + String(loraBandwidth) +
                  " CR=4/" + String(loraCodingRate) +
                  " Pwr=" + String(loraTxPower) +
                  " Sync=0x" + String(loraSyncWord, HEX));
  return true;
}

// Called from the LoRa library's interrupt context when a packet arrives.
// Keep this fast and free of String/heap work -- just flag it and let
// loop() do the actual read via LoRa.available()/LoRa.read(), same as
// the library's own examples do.
volatile unsigned int isrFireCount = 0;  // diagnostic: proves DIO0 interrupt is firing at all
void onLoRaReceive(int packetSize) {
  isrFireCount++;
  if (packetSize == 0) return;
  packetPending = true;
}

// Escapes characters that would otherwise break the JSON string literal
String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c >= 0x20) out += c;  // drop raw control chars rather than escape them
  }
  return out;
}

// Sends a raw LoRa payload and re-arms continuous receive afterward --
// beginPacket()/endPacket() drops out of receive mode while transmitting.
bool loraSendRaw(const String& payload) {
  LoRa.beginPacket();
  LoRa.print(payload);
  bool ok = (LoRa.endPacket() == 1);
  LoRa.receive();
  return ok;
}

// Drains one pending packet (if any) into the ring buffer, and reacts to
// its type tag: auto-acks GPS fixes, resolves a pending ping wait, logs
// free-text test messages. Called from loop(), and also pumped manually
// from handlePing()'s blocking wait so an incoming ack isn't missed while
// that request handler is paused.
void drainPendingPacket() {
  if (!packetPending) return;
  packetPending = false;

  String msg;
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }

  // An empty payload after a receive event usually means the packet
  // failed the SX1278's internal CRC check and the library dropped it --
  // real over-the-air noise/corruption, not a bug in this sketch. Log it
  // as an error instead of silently adding a blank row to the table.
  if (msg.length() == 0) {
    loraRxErrorCount++;
    loraLastError = "Received a packet that failed CRC / was empty -- likely RF noise or a config mismatch with the sender.";
    loraLastErrorMillis = millis();
    Serial.println(loraLastError);
    LoRa.receive();
    return;
  }

  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();

  // Always log the raw packet to the ring buffer regardless of type, so
  // the troubleshooting table shows everything that came in over the air.
  rxLog[rxLogHead].msg      = msg;
  rxLog[rxLogHead].rssi     = rssi;
  rxLog[rxLogHead].snr      = snr;
  rxLog[rxLogHead].atMillis = millis();
  rxLogHead = (rxLogHead + 1) % RX_LOG_SIZE;
  if (rxLogCount < RX_LOG_SIZE) rxLogCount++;
  rxTotalCount++;
  lastRxMillis = millis();

  Serial.println("LoRa RX: \"" + msg + "\" RSSI=" + String(rssi) + " SNR=" + String(snr));

  char tag = msg.charAt(0);

  if (tag == 'A') {
    // Ack reply to our own ping.
    if (pingWaitingForAck) {
      pingWaitingForAck = false;
      lastPingOk = true;
      lastPingRttMs = millis() - pingWaitStartMillis;
      pingAckCount++;
    }
  } else if (tag == 'G' && msg.length() > 2 && msg.charAt(1) == ':') {
    // GPS fix from the sender: "G:<seq>,<lat>,<lon>,<alt>,<sat>"
    String body = msg.substring(2);
    int c1 = body.indexOf(',');
    int c2 = body.indexOf(',', c1 + 1);
    int c3 = body.indexOf(',', c2 + 1);
    int c4 = body.indexOf(',', c3 + 1);

    if (c1 > 0 && c2 > c1 && c3 > c2 && c4 > c3) {
      unsigned int seq = body.substring(0, c1).toInt();
      lastGpsRx.valid    = true;
      lastGpsRx.seq      = seq;
      lastGpsRx.lat      = body.substring(c1 + 1, c2).toFloat();
      lastGpsRx.lon      = body.substring(c2 + 1, c3).toFloat();
      lastGpsRx.alt      = body.substring(c3 + 1, c4).toFloat();
      lastGpsRx.sat      = body.substring(c4 + 1).toInt();
      lastGpsRx.atMillis = millis();
      gpsRxCount++;

      // Auto-ack, echoing back the sequence number so the sender can
      // match this ack to the specific fix it just sent (rather than a
      // stray/late ack from an earlier send being mistaken for this one).
      loraSendRaw("K:" + String(seq));
      Serial.println("LoRa TX: \"K:" + String(seq) + "\" (auto-ack for GPS fix)");
    } else {
      Serial.println("Malformed GPS payload, not acking: \"" + msg + "\"");
    }
  }
  // 'P' (a ping) and 'T:' (free text) are logged above via rxLog but
  // don't need any special reaction on this board -- pings are things
  // WE send, and 'T:' test messages are just meant to be seen, not acked.

  LoRa.receive();
}

// ---------------- Dashboard HTML (served from PROGMEM) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Me-TR Receiver Dashboard</title>
<style>
  body { font-family: -apple-system, Arial, sans-serif; background:#111; color:#eee; margin:0; padding:20px; }
  h1 { font-size:1.3em; margin-bottom: 4px; }
  .sub { color:#888; margin-bottom:20px; font-size:0.85em; }
  .grid { display:grid; grid-template-columns: repeat(auto-fit, minmax(140px,1fr)); gap:10px; margin-bottom:24px; }
  .card { background:#1c1c1c; border-radius:8px; padding:12px; }
  .card .label { color:#888; font-size:0.75em; text-transform:uppercase; }
  .card .value { font-size:1.4em; margin-top:4px; }
  fieldset { border:1px solid #333; border-radius:8px; margin-bottom:20px; }
  legend { padding:0 8px; color:#ccc; }
  label { display:block; margin:10px 0 4px; color:#aaa; font-size:0.85em; }
  input, select { width:100%; box-sizing:border-box; padding:8px; background:#222; border:1px solid #333; color:#eee; border-radius:4px; }
  button { margin-top:14px; padding:10px 16px; background:#3a7; border:none; border-radius:6px; color:#fff; font-weight:bold; cursor:pointer; }
  button:hover { background:#4b8; }
  a.otalink { display:inline-block; margin-top:10px; color:#7af; }
  #status { font-size:0.8em; color:#666; margin-top:6px; }
  table { width:100%; border-collapse: collapse; font-size:0.85em; }
  th, td { text-align:left; padding:6px 8px; border-bottom:1px solid #2a2a2a; }
  th { color:#888; font-weight:normal; text-transform:uppercase; font-size:0.75em; }
  td.msg { word-break: break-word; max-width: 240px; }
  #rxEmpty { color:#666; font-size:0.85em; padding:8px 0; }
</style>
</head>
<body>
  <h1>Me-TR Receiver Dashboard</h1>
  <div class="sub">LoRa packet log &amp; radio config</div>

  <div id="errorBanner" style="display:none; background:#3a1414; border:1px solid #7a2a2a; border-radius:8px; padding:12px; margin-bottom:16px; color:#f88;"></div>

  <div class="grid">
    <div class="card"><div class="label">Radio Status</div><div class="value" id="radiostatus">--</div></div>
    <div class="card"><div class="label">Packets Received</div><div class="value" id="rxcount">--</div></div>
    <div class="card"><div class="label">Last RX</div><div class="value" id="rxage">--</div></div>
    <div class="card"><div class="label">Last RSSI</div><div class="value" id="rssi">--</div></div>
    <div class="card"><div class="label">Last SNR</div><div class="value" id="snr">--</div></div>
    <div class="card"><div class="label">RX Errors</div><div class="value" id="rxerrors">--</div></div>
  </div>

  <fieldset>
    <legend>LoRa Link Troubleshooting</legend>
    <label>Ping the sender (waits for an ack, reports round-trip time)</label>
    <button type="button" id="pingBtn">Send Ping</button>
    <div id="pingStatus" class="sub" style="margin-top:8px;"></div>
  </fieldset>

  <fieldset>
    <legend>Latest GPS Fix From Sender</legend>
    <div class="grid" style="margin-bottom:0;">
      <div class="card"><div class="label">Latitude</div><div class="value" id="gpsLat">--</div></div>
      <div class="card"><div class="label">Longitude</div><div class="value" id="gpsLon">--</div></div>
      <div class="card"><div class="label">Altitude (m)</div><div class="value" id="gpsAlt">--</div></div>
      <div class="card"><div class="label">Satellites</div><div class="value" id="gpsSat">--</div></div>
      <div class="card"><div class="label">Received</div><div class="value" id="gpsAge">--</div></div>
      <div class="card"><div class="label">Fixes Received</div><div class="value" id="gpsCount">--</div></div>
    </div>
  </fieldset>

  <fieldset>
    <legend>Received Messages</legend>
    <table id="rxTable">
      <thead><tr><th>When</th><th>Message</th><th>RSSI</th><th>SNR</th></tr></thead>
      <tbody id="rxBody"></tbody>
    </table>
    <div id="rxEmpty">No packets received yet.</div>
  </fieldset>

  <fieldset>
    <legend>LoRa Configuration</legend>
    <div class="sub">Must match the sender's settings exactly (except TX Power, which only matters on the sender) or packets won't demodulate at all.</div>
    <form id="loraForm">
      <label>Frequency (Hz)</label>
      <input type="number" id="freq" name="freq" step="1">

      <label>Spreading Factor (6-12)</label>
      <input type="number" id="sf" name="sf" min="6" max="12">

      <label>Bandwidth (Hz)</label>
      <select id="bw" name="bw">
        <option value="7800">7.8 kHz</option>
        <option value="10400">10.4 kHz</option>
        <option value="15600">15.6 kHz</option>
        <option value="20800">20.8 kHz</option>
        <option value="31250">31.25 kHz</option>
        <option value="41700">41.7 kHz</option>
        <option value="62500">62.5 kHz</option>
        <option value="125000">125 kHz</option>
        <option value="250000">250 kHz</option>
        <option value="500000">500 kHz</option>
      </select>

      <label>Coding Rate (4/x)</label>
      <input type="number" id="cr" name="cr" min="5" max="8">

      <label>TX Power (dBm, 2-20)</label>
      <input type="number" id="pwr" name="pwr" min="2" max="20">

      <label>Sync Word (hex)</label>
      <input type="text" id="sync" name="sync" placeholder="0x12" maxlength="4">
      <div class="sub" style="margin-top:4px;">Must match the sender exactly, or packets fail to demodulate -- same silent-failure mode as a freq/SF/BW/CR mismatch.</div>

      <button type="submit">Apply Configuration</button>
      <div id="status"></div>
    </form>
  </fieldset>

  <a class="otalink" href="/update">&#8593; Firmware Update (.bin upload)</a>

<script>
let consecutiveFetchFails = 0;

async function pollRx() {
  try {
    const r = await fetch('/rx.json');
    const d = await r.json();
    consecutiveFetchFails = 0;
    document.getElementById('rxcount').textContent = d.count;
    document.getElementById('rxage').textContent = d.count > 0 ? (d.lastAgeSec + 's ago') : 'never';
    document.getElementById('rssi').textContent = d.count > 0 ? (d.lastRssi + ' dBm') : '--';
    document.getElementById('snr').textContent = d.count > 0 ? (d.lastSnr + ' dB') : '--';
    document.getElementById('rxerrors').textContent = d.rxErrorCount;
    document.getElementById('radiostatus').textContent = d.loraOk ? 'OK' : 'FAILED';
    document.getElementById('radiostatus').style.color = d.loraOk ? '' : '#f88';

    const banner = document.getElementById('errorBanner');
    if (!d.loraOk || (d.lastError && d.lastErrorAgeSec < 30)) {
      banner.style.display = 'block';
      let text = '';
      if (!d.loraOk) {
        text = 'Radio is not initialized (' + d.initFailCount + ' init failure(s) since boot). ';
      }
      if (d.lastError) {
        text += d.lastError + ' (' + d.lastErrorAgeSec + 's ago)';
      }
      banner.textContent = text;
    } else {
      banner.style.display = 'none';
    }

    const body = document.getElementById('rxBody');
    const empty = document.getElementById('rxEmpty');
    if (d.entries.length === 0) {
      body.innerHTML = '';
      empty.style.display = 'block';
    } else {
      empty.style.display = 'none';
      body.innerHTML = d.entries.map(e =>
        '<tr><td>' + e.ageSec + 's ago</td><td class="msg">' + e.msg +
        '</td><td>' + e.rssi + '</td><td>' + e.snr + '</td></tr>'
      ).join('');
    }
  } catch (e) {
    consecutiveFetchFails++;
    if (consecutiveFetchFails >= 2) {
      const banner = document.getElementById('errorBanner');
      banner.style.display = 'block';
      banner.textContent = 'Cannot reach the board -- dashboard fetch failed. Check Wi-Fi connection to the device.';
    }
  }
}

async function loadLoRaConfig() {
  try {
    const r = await fetch('/lora.json');
    const d = await r.json();
    document.getElementById('freq').value = d.freq;
    document.getElementById('sf').value = d.sf;
    document.getElementById('bw').value = d.bw;
    document.getElementById('cr').value = d.cr;
    document.getElementById('pwr').value = d.pwr;
    document.getElementById('sync').value = d.sync;
  } catch (e) {}
}

document.getElementById('loraForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  const params = new URLSearchParams(new FormData(e.target));
  const statusEl = document.getElementById('status');
  statusEl.textContent = 'Applying...';
  try {
    const r = await fetch('/lora/set', { method: 'POST', body: params });
    const text = await r.text();
    statusEl.textContent = text;
  } catch (err) {
    statusEl.textContent = 'Failed to apply config.';
  }
});

document.getElementById('pingBtn').addEventListener('click', async () => {
  const btn = document.getElementById('pingBtn');
  const statusEl = document.getElementById('pingStatus');
  btn.disabled = true;
  statusEl.textContent = 'Pinging... (waiting up to 3s for ack)';
  try {
    const r = await fetch('/ping', { method: 'POST' });
    const d = await r.json();
    statusEl.textContent = d.ok
      ? ('Ack received in ' + d.rttMs + ' ms. (' + d.ackCount + '/' + d.sentCount + ' pings acked this session)')
      : ('No ack -- timed out. (' + d.ackCount + '/' + d.sentCount + ' pings acked this session)');
  } catch (err) {
    statusEl.textContent = 'Request failed.';
  }
  btn.disabled = false;
});

async function pollGpsRx() {
  try {
    const r = await fetch('/gpsrx.json');
    const d = await r.json();
    document.getElementById('gpsLat').textContent = d.valid ? d.lat.toFixed(6) : '--';
    document.getElementById('gpsLon').textContent = d.valid ? d.lon.toFixed(6) : '--';
    document.getElementById('gpsAlt').textContent = d.valid ? d.alt.toFixed(1) : '--';
    document.getElementById('gpsSat').textContent = d.valid ? d.sat : '--';
    document.getElementById('gpsAge').textContent = d.valid ? (d.ageSec + 's ago (#' + d.seq + ')') : 'never';
    document.getElementById('gpsCount').textContent = d.count;
  } catch (e) {}
}

loadLoRaConfig();
pollRx();
pollGpsRx();
setInterval(pollRx, 2000);
setInterval(pollGpsRx, 2000);
</script>
</body>
</html>
)HTML";

// ---------------- HTTP Handlers ----------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleRxJson() {
  unsigned long lastAgeSec = rxTotalCount > 0 ? (millis() - lastRxMillis) / 1000 : 0;
  unsigned long lastErrAgeSec = loraLastErrorMillis > 0 ? (millis() - loraLastErrorMillis) / 1000 : 0;

  String json = "{";
  json += "\"count\":" + String(rxTotalCount) + ",";
  json += "\"lastAgeSec\":" + String(lastAgeSec) + ",";
  json += "\"loraOk\":" + String(loraOk ? "true" : "false") + ",";
  json += "\"initFailCount\":" + String(loraInitFailCount) + ",";
  json += "\"rxErrorCount\":" + String(loraRxErrorCount) + ",";
  json += "\"lastError\":\"" + jsonEscape(loraLastError) + "\",";
  json += "\"lastErrorAgeSec\":" + String(lastErrAgeSec) + ",";

  if (rxTotalCount > 0) {
    int lastIdx = (rxLogHead - 1 + RX_LOG_SIZE) % RX_LOG_SIZE;
    json += "\"lastRssi\":" + String(rxLog[lastIdx].rssi) + ",";
    json += "\"lastSnr\":" + String(rxLog[lastIdx].snr, 1) + ",";
  } else {
    json += "\"lastRssi\":0,\"lastSnr\":0,";
  }

  json += "\"entries\":[";
  for (int i = 0; i < rxLogCount; i++) {
    int idx = (rxLogHead - 1 - i + RX_LOG_SIZE * 2) % RX_LOG_SIZE;  // walk newest-first
    unsigned long ageSec = (millis() - rxLog[idx].atMillis) / 1000;
    if (i > 0) json += ",";
    json += "{";
    json += "\"msg\":\"" + jsonEscape(rxLog[idx].msg) + "\",";
    json += "\"rssi\":" + String(rxLog[idx].rssi) + ",";
    json += "\"snr\":" + String(rxLog[idx].snr, 1) + ",";
    json += "\"ageSec\":" + String(ageSec);
    json += "}";
  }
  json += "]";
  json += "}";
  server.send(200, "application/json", json);
}

// Sends a ping to the sender and BLOCKS (with timeout) waiting for its
// ack. Bounded block on this one HTTP request only -- drainPendingPacket()
// is pumped manually inside the wait so an ack arriving here isn't missed
// just because loop() itself is paused during this call.
void handlePing() {
  pingWaitingForAck = true;
  pingWaitStartMillis = millis();
  lastPingOk = false;

  bool sent = loraSendRaw("P");
  pingSentCount++;
  if (!sent) {
    pingWaitingForAck = false;
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"LoRa send failed -- check wiring.\"}");
    return;
  }

  while (pingWaitingForAck && (millis() - pingWaitStartMillis) < PING_TIMEOUT_MS) {
    if (packetPending) drainPendingPacket();
    delay(5);
  }
  pingWaitingForAck = false;  // clears itself if it timed out

  String json = "{";
  json += "\"ok\":" + String(lastPingOk ? "true" : "false") + ",";
  json += "\"rttMs\":" + String(lastPingRttMs) + ",";
  json += "\"sentCount\":" + String(pingSentCount) + ",";
  json += "\"ackCount\":" + String(pingAckCount);
  json += "}";
  server.send(200, "application/json", json);
}

void handleGpsRxJson() {
  unsigned long ageSec = lastGpsRx.valid ? (millis() - lastGpsRx.atMillis) / 1000 : 0;
  String json = "{";
  json += "\"valid\":" + String(lastGpsRx.valid ? "true" : "false") + ",";
  json += "\"seq\":" + String(lastGpsRx.seq) + ",";
  json += "\"lat\":" + String(lastGpsRx.lat, 6) + ",";
  json += "\"lon\":" + String(lastGpsRx.lon, 6) + ",";
  json += "\"alt\":" + String(lastGpsRx.alt, 1) + ",";
  json += "\"sat\":" + String(lastGpsRx.sat) + ",";
  json += "\"ageSec\":" + String(ageSec) + ",";
  json += "\"count\":" + String(gpsRxCount);
  json += "}";
  server.send(200, "application/json", json);
}

void handleLoRaJson() {
  String json = "{";
  json += "\"freq\":" + String(loraFrequency) + ",";
  json += "\"sf\":" + String(loraSF) + ",";
  json += "\"bw\":" + String(loraBandwidth) + ",";
  json += "\"cr\":" + String(loraCodingRate) + ",";
  json += "\"pwr\":" + String(loraTxPower) + ",";
  json += "\"sync\":\"0x" + String(loraSyncWord, HEX) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleLoRaSet() {
  if (server.hasArg("freq")) loraFrequency  = server.arg("freq").toInt();
  if (server.hasArg("sf"))   loraSF         = server.arg("sf").toInt();
  if (server.hasArg("bw"))   loraBandwidth  = server.arg("bw").toInt();
  if (server.hasArg("cr"))   loraCodingRate = server.arg("cr").toInt();
  if (server.hasArg("pwr"))  loraTxPower    = server.arg("pwr").toInt();
  if (server.hasArg("sync")) {
    loraSyncWord = (uint8_t)strtoul(server.arg("sync").c_str(), nullptr, 16);
  }

  // Clamp to safe ranges
  loraSF          = constrain(loraSF, 6, 12);
  loraCodingRate  = constrain(loraCodingRate, 5, 8);
  loraTxPower     = constrain(loraTxPower, 2, 20);

  bool ok = applyLoRaConfig();
  server.send(200, "text/plain", ok ? "Applied." : "Failed — check LoRa wiring.");
}

void setup() {
  Serial.begin(115200);

  // ---------------- Wi-Fi ----------------
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // keeps radio responsive for OTA transfer + polling

  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // mDNS so you can reach it at http://me-tr-node-rx.local/ too
  if (MDNS.begin("me-tr-node-rx")) {
    Serial.println("mDNS responder started: me-tr-node-rx.local");
  }

  // ---------------- LoRa ----------------
  // Deliberately continue even if this fails -- the dashboard itself
  // still needs to come up so you can SEE the error, rather than the
  // board going dark with no way to diagnose it remotely.
  applyLoRaConfig();

  // ---------------- Web routes ----------------
  server.on("/", handleRoot);
  server.on("/rx.json", handleRxJson);
  server.on("/lora.json", handleLoRaJson);
  server.on("/lora/set", HTTP_POST, handleLoRaSet);
  server.on("/ping", HTTP_POST, handlePing);
  server.on("/gpsrx.json", handleGpsRxJson);

  // ElegantOTA mounts /update itself
  ElegantOTA.begin(&server);

  server.begin();
  Serial.println("Web server started.");
  Serial.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  Serial.println("OTA page:  http://" + WiFi.localIP().toString() + "/update");
}

void loop() {
  drainPendingPacket();

  int pktSize = LoRa.parsePacket();
  if (pktSize) {
    Serial.print("Polled packet: ");
    String msg;
    while (LoRa.available()) msg += (char)LoRa.read();
    Serial.println(msg);
    LoRa.receive(); // re-arm
  }

  // Diagnostic heartbeat: proves whether the DIO0 receive interrupt is
  // firing at all, independent of the dashboard or JSON layer. Watch in
  // Serial Monitor at 115200. If isrFireCount never climbs while the
  // sender is transmitting, point at DIO0 wiring/continuity, not
  // software. If it climbs but no "LoRa RX:" lines print, the ISR's
  // fine and the bug is in packet parsing.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    Serial.println("[heartbeat] isrFireCount=" + String(isrFireCount) +
                    " loraOk=" + String(loraOk ? "true" : "false") +
                    " freeHeap=" + String(ESP.getFreeHeap()));
  }

  server.handleClient();
  ElegantOTA.loop();
}