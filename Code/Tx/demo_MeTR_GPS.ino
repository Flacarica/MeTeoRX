/*
  ESP32 Web OTA + GPS Dashboard + LoRa Config
  1_WebOTA_GPS_LoRa.ino
  Base: DroneBot Workshop 2026 ArduinoOTA demo (OTA path since replaced)
  https://dronebotworkshop.com

  Board: ESP32 dev board

  Features:
    - NEO-7M GPS on UART1 peripheral, mapped to GPIO16 (RX) / GPIO17 (TX)
    - SX1278 (RA-02) LoRa TX via VSPI (SCK18/MISO19/MOSI23/NSS5/RST26/DIO0 4)
    - Local web server (no cloud, no Arduino IDE network step):
        /            -> live GPS dashboard + LoRa config/test + GPS config
        /gps.json    -> polled GPS telemetry + status (GET)
        /lora.json   -> current LoRa config (GET)
        /lora/set    -> apply new LoRa config (POST form)
        /lora/send   -> transmit an arbitrary test string over LoRa (POST form)
        /gpscfg.json -> current GPS config (GET)
        /gpscfg/set  -> apply GPS update rate / baud rate (POST form)
        /update      -> ElegantOTA .bin upload page

  NEO-7M notes (from u-blox NEO-7 datasheet + module manuals):
    - Default baud 9600; supports 4800/9600/19200/38400/57600/115200/230400
    - Default update rate 1 Hz; UBX-CFG-RATE can push it to 5 Hz or 10 Hz
    - Speaks NMEA (default) and/or UBX binary, interleaved
    - Outputs GGA/GSA/GSV/RMC/VTG once a second by default
    - Full sentence-mask / GNSS-constellation config needs UBX-CFG-MSG /
      UBX-CFG-GNSS with checksums, best done once via u-center — NOT exposed
      here since a bad mask can silently deafen the module until a
      power-cycle. Rate + baud are the two settings safe enough to expose
      as live web toggles.
    - Cheap breakout boards usually lack the backup EEPROM, so any UBX
      config (including what this sketch sends) is volatile and resets
      to 9600/1Hz on power loss unless you add a backup battery/cap.

  Requires (Library Manager):
    - TinyGPSPlus (Mikal Hart)
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
#include <TinyGPSPlus.h>

// ---------------- Network Credentials ----------------
const char* ssid = "DIGI-34Cz";
const char* password = "HH9F23RUzA";

WebServer server(80);

// ---------------- GPS (NEO-7M) on UART1 (peripheral), NOT GPIO1/3 ----------------
// GPIO1/GPIO3 are physically wired to the onboard USB-serial bridge chip
// (CP2102/CH340) on every standard ESP32 dev board -- that's UART0, used
// for flashing and Serial Monitor. Routing a second UART's signals onto
// those same pins puts two drivers fighting over one wire: the bridge
// chip and the GPS module both trying to drive the RX line into the ESP32.
// Symptom is exactly this: module has a fix (LED blinking normally) but
// the ESP32 never parses a valid sentence -- 0 satellites, no fix, forever.
// Moved to GPIO16/17, the standard free UART2 pins on non-WROVER boards.
// (If your board uses a WROVER module with PSRAM, GPIO16/17 are reserved
// instead -- check your module silkscreen and use two other free GPIOs.)
#define GPS_RX_PIN 16  // ESP32 RX  <- GPS TX
#define GPS_TX_PIN 17  // ESP32 TX  -> GPS RX
#define GPS_BAUD   9600
#define GPS_RX_BUFFER_SIZE 512  // default HW UART buffer (256) fills fast at 10Hz if loop() stalls

HardwareSerial GPSSerial(1);  // UART1 peripheral, mapped to GPIO16/17 above
TinyGPSPlus gps;

// ---------------- LoRa (SX1278 / RA-02) ----------------
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_NSS   5
#define LORA_DIO0  4
#define LORA_RST   26

// Runtime-adjustable LoRa parameters (defaults on boot)
long loraFrequency   = 433E6;  // Hz
int  loraSF          = 7;      // spreading factor 6-12
long loraBandwidth   = 125E3;  // Hz
int  loraCodingRate  = 5;      // 4/5 .. 4/8 -> pass 5-8
int  loraTxPower     = 17;     // dBm, 2-20
// Sync word: must match on both ends of the link or packets silently fail
// to receive (same failure mode as a freq/SF/BW/CR mismatch, just easier
// to overlook since it wasn't exposed before). 0x12 is the SX127x LoRa
// library's own default -- used here explicitly instead of relying on it
// being unset. 0x34 is the conventional "public LoRaWAN" sync word; any
// other single byte works fine for a private point-to-point link like
// this one, just keep both boards on the same value.
uint8_t loraSyncWord = 0x12;

// GPS status tracking
unsigned long lastFixMillis = 0;      // millis() at last valid location update
unsigned long lastNMEAMillis = 0;     // millis() at last byte seen from module at all
unsigned int  gpsCharsProcessed = 0;
unsigned int  gpsSentencesWithFix = 0;
unsigned int  gpsChecksumFails = 0;

// GPS config (what we last told the module — best-effort, see notes above)
long gpsBaud       = GPS_BAUD;
int  gpsUpdateRateHz = 1;

// LoRa TX test message result
String  lastLoRaTxMsg = "";
bool    lastLoRaTxOk  = false;
unsigned long lastLoRaTxMillis = 0;
unsigned int  loraTxCount = 0;

// ---------------- LoRa packet framing (transceiver mode) ----------------
// Every payload starts with a single-character type tag so both ends can
// tell messages apart on receive:
//   'P'        ping        (receiver -> sender, manual troubleshoot button)
//   'A'        ack/pong    (sender -> receiver, auto-reply to a ping)
//   'G:<data>' GPS fix     (sender -> receiver)
//   'K:<seq>'  GPS-ack     (receiver -> sender, echoes the fix's sequence number)
//   'T:<msg>'  free text   (either direction, the existing antenna-test message)
unsigned int gpsSendSeq = 0;   // increments each GPS send, embedded in the payload
                                // and echoed back in the ack so a late/stray ack
                                // from a previous send can't be mistaken for this one

// Set from the LoRa receive interrupt; drained in loop(), same pattern as
// the receiver node uses -- keep the ISR itself free of String/heap work.
volatile bool packetPending = false;

// Ping/ack (troubleshooting) state
bool pingWaitingForAck = false;
unsigned long pingWaitStartMillis = 0;
unsigned long lastPingRttMs = 0;      // round-trip time of the last successful ping
bool lastPingOk = false;
unsigned int pingSentCount = 0;
unsigned int pingAckCount = 0;
#define PING_TIMEOUT_MS 3000

// GPS-send ack state
bool gpsAckWaitingForAck = false;
unsigned long gpsAckWaitStartMillis = 0;
unsigned int  gpsAckWaitingSeq = 0;
bool lastGpsSendAckOk = false;
unsigned long lastGpsSendRttMs = 0;
unsigned int  gpsSendAckCount = 0;
unsigned int  gpsSendTimeoutCount = 0;
#define GPS_ACK_TIMEOUT_MS 4000  // GPS payload's bigger than a bare ping, give it more room

// Last received packet (any type) for Serial logging / debugging
String lastRxRaw = "";

// ---------------- UBX-CFG-RATE sender (best-effort, no ACK check) ----------------
// Builds and sends a UBX-CFG-RATE packet to set the GPS measurement rate.
// This is a minimal, unverified send (no ACK/NAK parsing) — good enough for
// a "try it and see if satellites/fix keep updating" dashboard toggle.
void sendUbxSetRate(unsigned int rateHz) {
  if (rateHz == 0) rateHz = 1;
  unsigned int measRateMs = 1000 / rateHz;

  uint8_t payload[6] = {
    (uint8_t)(measRateMs & 0xFF), (uint8_t)(measRateMs >> 8),  // measRate
    0x01, 0x00,                                                 // navRate = 1 cycle
    0x01, 0x00                                                  // timeRef = 1 (GPS time)
  };

  uint8_t packet[14];
  packet[0] = 0xB5; packet[1] = 0x62;      // UBX sync chars
  packet[2] = 0x06; packet[3] = 0x08;      // class CFG, id RATE
  packet[4] = 0x06; packet[5] = 0x00;      // payload length = 6
  memcpy(&packet[6], payload, 6);

  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 12; i++) {
    ckA += packet[i];
    ckB += ckA;
  }
  packet[12] = ckA;
  packet[13] = ckB;

  GPSSerial.write(packet, 14);
  gpsUpdateRateHz = rateHz;
}

// ---------------- LoRa init/apply helper ----------------
bool applyLoRaConfig() {
  LoRa.end();  // safe even if not started yet in most impls after begin() once

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(loraFrequency)) {
    Serial.println("LoRa init failed! Check wiring.");
    return false;
  }

  LoRa.setSpreadingFactor(loraSF);
  LoRa.setSignalBandwidth(loraBandwidth);
  LoRa.setCodingRate4(loraCodingRate);
  LoRa.setTxPower(loraTxPower);
  LoRa.setSyncWord(loraSyncWord);

  // Transceiver mode: arm the receive callback so incoming pings/acks get
  // caught between transmits, then drop into continuous receive.
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

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
// loop() do the actual read via LoRa.available()/LoRa.read().
volatile unsigned int isrFireCount = 0;  // diagnostic: proves DIO0 interrupt is firing at all
void onLoRaReceive(int packetSize) {
  isrFireCount++;
  if (packetSize == 0) return;
  packetPending = true;
}

// Sends a raw LoRa payload and re-arms continuous receive afterward.
// LoRa.beginPacket()/endPacket() drops out of receive mode while
// transmitting, so every send needs to explicitly call LoRa.receive()
// again once it's done, or the radio goes deaf until the next config
// change re-arms it.
bool loraSendRaw(const String& payload) {
  LoRa.beginPacket();
  LoRa.print(payload);
  bool ok = (LoRa.endPacket() == 1);
  LoRa.receive();  // re-arm listening immediately after sending
  return ok;
}

// Drains one pending packet (if any), parses its type tag, and reacts:
// auto-replies to pings with an ack, and to GPS-acks resolves the pending
// wait so the blocking-with-timeout send in handleGPSCfgSend can return
// early instead of always waiting the full timeout.
void drainPendingPacket() {
  if (!packetPending) return;
  packetPending = false;

  String msg;
  while (LoRa.available()) {
    msg += (char)LoRa.read();
  }
  if (msg.length() == 0) {
    LoRa.receive();
    return;  // failed CRC / empty read, nothing to parse
  }

  lastRxRaw = msg;
  Serial.println("LoRa RX: \"" + msg + "\"");

  char tag = msg.charAt(0);

  if (tag == 'P') {
    // Receiver pinged us -- reply immediately with an ack.
    loraSendRaw("A");
    Serial.println("LoRa TX: \"A\" (auto-reply to ping)");
  } else if (tag == 'K' && msg.length() > 2 && msg.charAt(1) == ':') {
    // GPS-ack from the receiver, echoing back a sequence number.
    unsigned int ackedSeq = msg.substring(2).toInt();
    if (gpsAckWaitingForAck && ackedSeq == gpsAckWaitingSeq) {
      gpsAckWaitingForAck = false;
      lastGpsSendAckOk = true;
      lastGpsSendRttMs = millis() - gpsAckWaitStartMillis;
      gpsSendAckCount++;
    }
  }
  // 'A' (a bare ack) and other tags aren't expected as unsolicited
  // incoming traffic on the sender node -- ignored rather than errored,
  // since a stray/duplicate packet isn't itself a fault condition.
}

// ---------------- Dashboard HTML (served from PROGMEM) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Me-TR Node Dashboard</title>
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
</style>
</head>
<body>
  <h1>Me-TR Node Dashboard</h1>
  <div class="sub">Live GPS telemetry &amp; LoRa radio config</div>

  <div class="grid">
    <div class="card"><div class="label">Latitude</div><div class="value" id="lat">--</div></div>
    <div class="card"><div class="label">Longitude</div><div class="value" id="lon">--</div></div>
    <div class="card"><div class="label">Altitude (m)</div><div class="value" id="alt">--</div></div>
    <div class="card"><div class="label">Satellites</div><div class="value" id="sat">--</div></div>
    <div class="card"><div class="label">Fix</div><div class="value" id="fix">--</div></div>
    <div class="card"><div class="label">HDOP</div><div class="value" id="hdop">--</div></div>
    <div class="card"><div class="label">Fix Age</div><div class="value" id="fixage">--</div></div>
    <div class="card"><div class="label">Module Link</div><div class="value" id="link">--</div></div>
    <div class="card"><div class="label">Checksum Fails</div><div class="value" id="ckfail">--</div></div>
  </div>

  <fieldset>
    <legend>LoRa Link Troubleshooting</legend>

    <label>Ping the receiver (waits for an ack, reports round-trip time)</label>
    <button type="button" id="pingBtn">Send Ping</button>
    <div id="pingStatus" class="sub" style="margin-top:8px;"></div>

    <label style="margin-top:18px;">Send current GPS fix (waits for the receiver's ack)</label>
    <button type="button" id="gpsSendBtn">Send GPS Data</button>
    <div id="gpsSendStatus" class="sub" style="margin-top:8px;"></div>
  </fieldset>

  <fieldset>
    <legend>LoRa TX Test (antenna check, no ack expected)</legend>
    <form id="loraSendForm">
      <label>Message to transmit</label>
      <input type="text" id="txmsg" name="msg" maxlength="200" placeholder="hello from the bench" value="ping">
      <button type="submit">Send over LoRa</button>
      <div id="sendStatus"></div>
      <div class="sub" id="sendHistory" style="margin-top:8px;"></div>
    </form>
  </fieldset>

  <fieldset>
    <legend>LoRa Configuration</legend>
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
      <div class="sub" style="margin-top:4px;">Must match the receiver exactly, or packets fail to demodulate -- same silent-failure mode as a freq/SF/BW/CR mismatch.</div>

      <button type="submit">Apply Configuration</button>
      <div id="status"></div>
    </form>
  </fieldset>

  <fieldset>
    <legend>GPS Configuration (NEO-7M)</legend>
    <form id="gpsForm">
      <label>Update Rate</label>
      <select id="rate" name="rate">
        <option value="1">1 Hz (default)</option>
        <option value="5">5 Hz</option>
        <option value="10">10 Hz</option>
      </select>
      <div class="sub" style="margin-top:6px;">
        Sent as a best-effort UBX-CFG-RATE command (no ACK check). Cheap
        NEO-7M breakouts usually lack a backup battery/EEPROM, so this
        resets to 1&nbsp;Hz / 9600&nbsp;baud on power loss.
      </div>
      <button type="submit" style="margin-top:14px;">Apply Update Rate</button>
      <div id="gpsStatus"></div>
    </form>
  </fieldset>

  <a class="otalink" href="/update">&#8593; Firmware Update (.bin upload)</a>

<script>
function hdopLabel(h) {
  if (h <= 0) return '--';
  if (h < 1) return h.toFixed(2) + ' (ideal)';
  if (h < 2) return h.toFixed(2) + ' (excellent)';
  if (h < 5) return h.toFixed(2) + ' (good)';
  if (h < 10) return h.toFixed(2) + ' (moderate)';
  if (h < 20) return h.toFixed(2) + ' (fair)';
  return h.toFixed(2) + ' (poor)';
}

async function pollGPS() {
  try {
    const r = await fetch('/gps.json');
    const d = await r.json();
    document.getElementById('lat').textContent = d.valid ? d.lat.toFixed(6) : '--';
    document.getElementById('lon').textContent = d.valid ? d.lon.toFixed(6) : '--';
    document.getElementById('alt').textContent = d.valid ? d.alt.toFixed(1) : '--';
    document.getElementById('sat').textContent = d.sat;
    document.getElementById('fix').textContent = d.valid ? 'Valid' : 'No fix';
    document.getElementById('hdop').textContent = hdopLabel(d.hdop);
    document.getElementById('fixage').textContent = d.valid ? (d.fixAgeSec + 's ago') : 'never';
    document.getElementById('link').textContent = d.linkOk ? 'Receiving data' : 'No data from module';
    document.getElementById('ckfail').textContent = d.checksumFails;
  } catch (e) { /* server busy handling OTA etc, just skip a beat */ }
}

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

document.getElementById('gpsSendBtn').addEventListener('click', async () => {
  const btn = document.getElementById('gpsSendBtn');
  const statusEl = document.getElementById('gpsSendStatus');
  btn.disabled = true;
  statusEl.textContent = 'Sending GPS fix... (waiting up to 4s for ack)';
  try {
    const r = await fetch('/gps/send', { method: 'POST' });
    const d = await r.json();
    if (d.error) {
      statusEl.textContent = d.error;
    } else {
      statusEl.textContent = d.ok
        ? ('Ack received in ' + d.rttMs + ' ms for fix #' + d.seq + '. (' + d.ackCount + ' acked, ' + d.timeoutCount + ' timed out this session)')
        : ('No ack for fix #' + d.seq + ' -- timed out. (' + d.ackCount + ' acked, ' + d.timeoutCount + ' timed out this session)');
    }
  } catch (err) {
    statusEl.textContent = 'Request failed.';
  }
  btn.disabled = false;
});

document.getElementById('loraSendForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  const params = new URLSearchParams(new FormData(e.target));
  const statusEl = document.getElementById('sendStatus');
  statusEl.textContent = 'Sending...';
  try {
    const r = await fetch('/lora/send', { method: 'POST', body: params });
    const d = await r.json();
    statusEl.textContent = d.ok ? ('Sent (#' + d.count + ')') : 'Send failed — check LoRa wiring.';
    document.getElementById('sendHistory').textContent =
      'Last: "' + d.msg + '" @ ' + d.count + ' total sent this session.';
  } catch (err) {
    statusEl.textContent = 'Request failed.';
  }
});

document.getElementById('gpsForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  const params = new URLSearchParams(new FormData(e.target));
  const statusEl = document.getElementById('gpsStatus');
  statusEl.textContent = 'Applying...';
  try {
    const r = await fetch('/gpscfg/set', { method: 'POST', body: params });
    const text = await r.text();
    statusEl.textContent = text;
  } catch (err) {
    statusEl.textContent = 'Failed to apply.';
  }
});

async function loadGPSConfig() {
  try {
    const r = await fetch('/gpscfg.json');
    const d = await r.json();
    document.getElementById('rate').value = d.rate;
  } catch (e) {}
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

loadLoRaConfig();
loadGPSConfig();
pollGPS();
setInterval(pollGPS, 2000);
</script>
</body>
</html>
)HTML";

// ---------------- HTTP Handlers ----------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleGPSJson() {
  bool valid = gps.location.isValid();
  unsigned long fixAgeSec = valid ? (millis() - lastFixMillis) / 1000 : 0;
  bool linkOk = (millis() - lastNMEAMillis) < 3000;  // heard any byte in last 3s

  String json = "{";
  json += "\"valid\":" + String(valid ? "true" : "false") + ",";
  json += "\"lat\":" + String(valid ? gps.location.lat() : 0, 6) + ",";
  json += "\"lon\":" + String(valid ? gps.location.lng() : 0, 6) + ",";
  json += "\"alt\":" + String(gps.altitude.isValid() ? gps.altitude.meters() : 0, 1) + ",";
  json += "\"sat\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",";
  json += "\"hdop\":" + String(gps.hdop.isValid() ? gps.hdop.hdop() : 0, 2) + ",";
  json += "\"fixAgeSec\":" + String(fixAgeSec) + ",";
  json += "\"linkOk\":" + String(linkOk ? "true" : "false") + ",";
  json += "\"checksumFails\":" + String(gpsChecksumFails);
  json += "}";
  server.send(200, "application/json", json);
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

void handleLoRaSend() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";
  bool ok = false;

  if (msg.length() > 0) {
    ok = loraSendRaw("T:" + msg);
    if (ok) {
      loraTxCount++;
      lastLoRaTxMsg = msg;
      lastLoRaTxMillis = millis();
    }
  }
  lastLoRaTxOk = ok;

  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"msg\":\"" + jsonEscape(msg) + "\",";
  json += "\"count\":" + String(loraTxCount);
  json += "}";
  server.send(200, "application/json", json);
}

// Sends the current GPS fix over LoRa and BLOCKS (with a hard timeout)
// waiting for a 'K:<seq>' ack from the receiver. This is a deliberate,
// bounded block -- server.handleClient() for THIS request is what's
// stalling, not the whole loop() forever, and GPS_ACK_TIMEOUT_MS caps it.
// A truly non-blocking version would need to return immediately and have
// the browser poll for the result separately; this is simpler and the
// multi-second wait is expected/visible to the person who just clicked
// the button, not a silent freeze.
void handleGPSSend() {
  if (!gps.location.isValid()) {
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"No valid GPS fix to send.\"}");
    return;
  }

  gpsSendSeq++;
  String payload = "G:" + String(gpsSendSeq) + "," +
                    String(gps.location.lat(), 6) + "," +
                    String(gps.location.lng(), 6) + "," +
                    String(gps.altitude.meters(), 1) + "," +
                    String(gps.satellites.value());

  gpsAckWaitingForAck = true;
  gpsAckWaitingSeq    = gpsSendSeq;
  gpsAckWaitStartMillis = millis();
  lastGpsSendAckOk = false;

  bool sent = loraSendRaw(payload);
  if (!sent) {
    gpsAckWaitingForAck = false;
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"LoRa send failed -- check wiring.\"}");
    return;
  }

  // Block THIS request handler (not all of loop()) until the ack arrives
  // or we time out. drainPendingPacket() must keep running during this
  // wait, so we pump it manually here instead of only relying on the
  // top of loop() -- otherwise an ack arriving during this wait would
  // never get processed since loop() itself is paused inside this call.
  while (gpsAckWaitingForAck && (millis() - gpsAckWaitStartMillis) < GPS_ACK_TIMEOUT_MS) {
    if (packetPending) drainPendingPacket();
    delay(5);
  }

  bool ok;
  unsigned long rtt = 0;
  if (gpsAckWaitingForAck) {
    // Timed out waiting
    gpsAckWaitingForAck = false;
    gpsSendTimeoutCount++;
    ok = false;
  } else {
    ok = lastGpsSendAckOk;
    rtt = lastGpsSendRttMs;
  }

  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"seq\":" + String(gpsSendSeq) + ",";
  json += "\"rttMs\":" + String(rtt) + ",";
  json += "\"ackCount\":" + String(gpsSendAckCount) + ",";
  json += "\"timeoutCount\":" + String(gpsSendTimeoutCount);
  json += "}";
  server.send(200, "application/json", json);
}

// Sends a ping and BLOCKS (with timeout) waiting for the receiver's ack --
// same bounded-block reasoning as handleGPSSend() above.
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
    if (packetPending) {
      // Handle inline here rather than via drainPendingPacket()'s normal
      // tag dispatch, since a ping's ack is a bare "A" with no sequence
      // number to check -- any 'A' received while waiting counts.
      packetPending = false;
      String msg;
      while (LoRa.available()) msg += (char)LoRa.read();
      lastRxRaw = msg;
      if (msg.length() > 0 && msg.charAt(0) == 'A') {
        pingWaitingForAck = false;
        lastPingOk = true;
        lastPingRttMs = millis() - pingWaitStartMillis;
        pingAckCount++;
      }
      LoRa.receive();
    }
    delay(5);
  }
  pingWaitingForAck = false;  // in case it timed out

  String json = "{";
  json += "\"ok\":" + String(lastPingOk ? "true" : "false") + ",";
  json += "\"rttMs\":" + String(lastPingRttMs) + ",";
  json += "\"sentCount\":" + String(pingSentCount) + ",";
  json += "\"ackCount\":" + String(pingAckCount);
  json += "}";
  server.send(200, "application/json", json);
}

void handleGPSCfgJson() {
  String json = "{";
  json += "\"rate\":" + String(gpsUpdateRateHz) + ",";
  json += "\"baud\":" + String(gpsBaud);
  json += "}";
  server.send(200, "application/json", json);
}

void handleGPSCfgSet() {
  if (server.hasArg("rate")) {
    int rate = server.arg("rate").toInt();
    if (rate == 1 || rate == 5 || rate == 10) {
      sendUbxSetRate(rate);
      server.send(200, "text/plain", "Update rate command sent (" + String(rate) + " Hz). No ACK check — watch the Satellites/Fix Age cards to confirm it's still talking.");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid rate — use 1, 5, or 10.");
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
    // Accepts "0x12", "12" (hex either way), strtoul with base 16 handles
    // the "0x" prefix itself if present, ignores it if not.
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

  // mDNS so you can reach it at http://me-tr-node-1.local/ too
  if (MDNS.begin("me-tr-node-1")) {
    Serial.println("mDNS responder started: me-tr-node-1.local");
  }

  // ---------------- GPS ----------------
  GPSSerial.setRxBufferSize(GPS_RX_BUFFER_SIZE);  // must be called BEFORE begin()
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART1 initialized.");

  // ---------------- LoRa ----------------
  applyLoRaConfig();

  // ---------------- Web routes ----------------
  server.on("/", handleRoot);
  server.on("/gps.json", handleGPSJson);
  server.on("/lora.json", handleLoRaJson);
  server.on("/lora/set", HTTP_POST, handleLoRaSet);
  server.on("/lora/send", HTTP_POST, handleLoRaSend);
  server.on("/gps/send", HTTP_POST, handleGPSSend);
  server.on("/ping", HTTP_POST, handlePing);
  server.on("/gpscfg.json", handleGPSCfgJson);
  server.on("/gpscfg/set", HTTP_POST, handleGPSCfgSet);

  // ElegantOTA mounts /update itself
  ElegantOTA.begin(&server);

  server.begin();
  Serial.println("Web server started.");
  Serial.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  Serial.println("OTA page:  http://" + WiFi.localIP().toString() + "/update");
}

void loop() {
  // Drain the GPS UART FIRST, every loop, before anything that might block
  // (server.handleClient() can take a while mid-OTA-upload or on a slow
  // client). If GPS parsing happens after a stall, the hardware UART's
  // RX FIFO can overflow and silently drop bytes -- that's the likely
  // cause of a "froze even though I had a fix" dashboard.
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    lastNMEAMillis = millis();
    gpsCharsProcessed++;
    gps.encode(c);
  }

  // isUpdated() is a one-shot flag -- true only once per fresh fix, then
  // false again until the next one. Checking it here, right after the
  // drain above, is the only place it should be read.
  if (gps.location.isUpdated()) {
    lastFixMillis = millis();
    gpsSentencesWithFix++;
  }
  gpsChecksumFails = gps.failedChecksum();

  // Drain any LoRa packet that arrived outside of an active ping/GPS-ack
  // wait (those two paths pump drainPendingPacket()/inline-handle it
  // themselves while blocked). This catches unsolicited incoming pings
  // from the receiver at any other time.
  if (packetPending) drainPendingPacket();

  // Diagnostic heartbeat: proves whether the DIO0 receive interrupt is
  // firing at all, independent of the web dashboard, JSON parsing, or
  // anything else that could mask the real signal. Watch this in Serial
  // Monitor at 115200 baud. If isrFireCount never climbs while the other
  // board is transmitting, the interrupt itself isn't firing -- point at
  // DIO0 wiring/continuity, not software. If it climbs steadily but no
  // "LoRa RX:" lines ever print, the ISR's firing fine and the bug is in
  // packet parsing instead.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    Serial.println("[heartbeat] isrFireCount=" + String(isrFireCount) +
                    " loraOkLikely=" + String(true) +
                    " freeHeap=" + String(ESP.getFreeHeap()));
  }

  server.handleClient();
  ElegantOTA.loop();
}