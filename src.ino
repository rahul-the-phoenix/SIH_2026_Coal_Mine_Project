#include "BluetoothSerial.h"
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <U8g2lib.h>

const char* ssid = "Me";
const char* password = "password";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;

// Pin Definitions
#define KNOCK_PIN 4
#define TILT_PIN 5
#define LED_PIN 2

// MPU6050 Object
MPU6050 mpu(Wire);
const uint8_t MPU_ADDR = 0x68;

BluetoothSerial bt;

// ===== OLED (1.3" - most common driver is SH1106, 128x64, I2C) =====
// If your OLED is actually SSD1306 (common on 0.96"), comment the SH1106
// line below and uncomment the SSD1306 line instead.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ===== WEB SERVER (WiFi Dashboard) =====
WebServer server(80);

volatile bool knock = false;
unsigned long lastKnock = 0;
unsigned long lastNotif = 0;

// ===== TILT VARIABLES (1.5 sec verification) =====
unsigned long tiltStartTime = 0;
unsigned long lastTiltNotif = 0;
bool tiltAlertSent = false;
bool isTilted = false;
const unsigned long TILT_VERIFICATION_TIME = 1500;

// ===== MPU6050 VARIABLES =====
float ax, ay, az;
float gx, gy, gz;
float prevAx = 0, prevAy = 0, prevAz = 0;
float axChange = 0, ayChange = 0, azChange = 0;
unsigned long lastMPURead = 0;
unsigned long lastMPUNotif = 0;
bool mpuAlertSent = false;
bool mpuInitialized = false;

const float MPU_THRESHOLD = 1.0;
const unsigned long MPU_CHECK_INTERVAL = 100;
const unsigned long MPU_COOLDOWN = 3000;

// ===== TIME VARIABLES =====
bool timeOK = false;

// ===== OLED STATE VARIABLES =====
unsigned long lastOledUpdate = 0;
const unsigned long OLED_NORMAL_REFRESH = 500;

bool oledShowingAlert = false;
unsigned long oledAlertStart = 0;
const unsigned long OLED_ALERT_DURATION = 6000; // how long alert stays on OLED
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 400;
bool oledBlinkState = false;

// pre-wrapped lines for the alert screen (computed once per alert, redrawn on blink)
String alertTitleLines[2];
int alertTitleLineCount = 0;
String alertActionLines[2];
int alertActionLineCount = 0;

// ===== LAST ALERT INFO (for the web dashboard banner) =====
String lastAlertType   = "No Alerts Yet";
String lastAlertSensor = "--";
String lastAlertAction = "System monitoring normally";
String lastAlertTime   = "--";
unsigned long alertActiveUntil = 0;         // dashboard shows RED/pulsing banner until this time
const unsigned long ALERT_ACTIVE_DURATION = 8000;

// ===== INTERRUPT =====
void IRAM_ATTR isrKnock() {
  if (millis() - lastKnock > 100) {
    knock = true;
    lastKnock = millis();
  }
}

// ===== TIME FUNCTIONS =====
String getTime() {
  struct tm t;
  if (!getLocalTime(&t)) return "No Time";
  char b[60];
  strftime(b, sizeof(b), "%A, %d-%B-%Y %I:%M:%S %p", &t);
  return String(b);
}

String getDateOnly() {
  struct tm t;
  if (!getLocalTime(&t)) return "No Date";
  char b[30];
  strftime(b, sizeof(b), "%d/%m/%Y", &t);
  return String(b);
}

String getDayOnly() {
  struct tm t;
  if (!getLocalTime(&t)) return "No Day";
  char b[20];
  strftime(b, sizeof(b), "%A", &t);
  return String(b);
}

// short 3-letter day (Mon, Tue...) - keeps the OLED date line from overflowing
String getDayShort() {
  struct tm t;
  if (!getLocalTime(&t)) return "---";
  char b[8];
  strftime(b, sizeof(b), "%a", &t);
  return String(b);
}

String getTimeOnly() {
  struct tm t;
  if (!getLocalTime(&t)) return "No Time";
  char b[20];
  strftime(b, sizeof(b), "%I:%M:%S %p", &t);
  return String(b);
}

// ===== MPU6050 FUNCTIONS =====
bool initMPU6050() {
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("MPU6050 not detected on I2C!");
    return false;
  }

  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("MPU6050 Initialized!");
  return true;
}

void readMPU6050() {
  mpu.update();
  ax = mpu.getAccX();
  ay = mpu.getAccY();
  az = mpu.getAccZ();
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();
}

bool detectSuddenMovement() {
  axChange = abs(ax - prevAx);
  ayChange = abs(ay - prevAy);
  azChange = abs(az - prevAz);

  if (axChange > MPU_THRESHOLD || ayChange > MPU_THRESHOLD || azChange > MPU_THRESHOLD) {
    return true;
  }
  return false;
}

String getMPUStatus() {
  String status = "X:" + String(ax, 2) + "g, Y:" + String(ay, 2) + "g, Z:" + String(az, 2) + "g";
  status += " | dX:" + String(axChange, 2) + ", dY:" + String(ayChange, 2) + ", dZ:" + String(azChange, 2);
  return status;
}

String getMPUChange() {
  String change = "dX:" + String(axChange, 2) + "g, dY:" + String(ayChange, 2) + "g, dZ:" + String(azChange, 2) + "g";
  return change;
}

// simple JSON-safety: swap double quotes so we never break the JSON string
String jsonSafe(String s) {
  s.replace("\"", "'");
  return s;
}

// =====================================================================
//                              OLED HELPERS
// =====================================================================

// shrinks a string (with current font already set) until it fits maxWidth
void truncateFit(String &s, int maxWidth) {
  while (u8g2.getStrWidth(s.c_str()) > maxWidth && s.length() > 1) {
    s.remove(s.length() - 1);
  }
}

// word-wraps text (with current font already set) into at most maxLines lines,
// each no wider than maxWidth. Extra text beyond maxLines is truncated with "..".
int wrapText(String text, int maxWidth, String outLines[], int maxLines) {
  int count = 0;
  String cur = "";
  int start = 0;
  int len = text.length();

  while (start < len && count < maxLines) {
    int spaceIdx = text.indexOf(' ', start);
    String word = (spaceIdx == -1) ? text.substring(start) : text.substring(start, spaceIdx);
    String testLine = (cur.length() == 0) ? word : (cur + " " + word);

    if (u8g2.getStrWidth(testLine.c_str()) <= maxWidth) {
      cur = testLine;
    } else if (cur.length() > 0) {
      outLines[count++] = cur;
      cur = word;
    } else {
      // single word already too long - hard cut it
      cur = word;
      truncateFit(cur, maxWidth);
    }

    start = (spaceIdx == -1) ? len : spaceIdx + 1;
  }

  if (cur.length() > 0 && count < maxLines) {
    outLines[count++] = cur;
  }

  // if there is leftover text we couldn't fit, mark the last line with ".."
  if (start < len && count > 0) {
    String &last = outLines[count - 1];
    String withDots = last + "..";
    while (u8g2.getStrWidth(withDots.c_str()) > maxWidth && withDots.length() > 2) {
      last.remove(last.length() - 1);
      withDots = last + "..";
    }
    last = withDots;
  }

  return count;
}

// =====================================================================
//                              OLED SCREENS
// =====================================================================
void initOLED() {
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB10_tr);
  u8g2.drawStr(8, 26, "ALERT SYSTEM");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(8, 44, "Starting up...");
  u8g2.sendBuffer();
  delay(1200);
}

// ---- Normal "home screen": big time, day/date, compact sensor row ----
void oledNormalDisplay() {
  u8g2.clearBuffer();

  // Top status bar
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 8);
  u8g2.print(WiFi.status() == WL_CONNECTED ? "WiFi:OK" : "WiFi:--");

  String mpuTxt = mpuInitialized ? "MPU:OK" : "MPU:--";
  int mw = u8g2.getStrWidth(mpuTxt.c_str());
  u8g2.setCursor(OLED_WIDTH - mw, 8);
  u8g2.print(mpuTxt);

  u8g2.drawHLine(0, 11, OLED_WIDTH);

  // Big time, always centered & always fits (HH:MM:SS AM/PM)
  u8g2.setFont(u8g2_font_fub17_tr);
  String t = timeOK ? getTimeOnly() : "--:--:--";
  int w = u8g2.getStrWidth(t.c_str());
  u8g2.setCursor((OLED_WIDTH - w) / 2, 33);
  u8g2.print(t);

  // Day (short) + Date - guaranteed to fit using truncateFit as a safety net
  u8g2.setFont(u8g2_font_7x13_tf);
  String dd = timeOK ? (getDayShort() + ", " + getDateOnly()) : "Time Not Synced";
  truncateFit(dd, OLED_WIDTH - 4);
  int w2 = u8g2.getStrWidth(dd.c_str());
  u8g2.setCursor((OLED_WIDTH - w2) / 2, 47);
  u8g2.print(dd);

  u8g2.drawHLine(0, 51, OLED_WIDTH);

  // Bottom row: three short fixed-width sensor chips (never overflows)
  u8g2.setFont(u8g2_font_6x10_tf);
  String kTxt = "K:" + String(digitalRead(KNOCK_PIN) ? "!" : "OK");
  String tTxt = "T:" + String(tiltAlertSent ? "!" : (isTilted ? ".." : "OK"));
  String mTxt = "M:" + String(mpuAlertSent ? "!" : "OK");

  u8g2.setCursor(4, 62);
  u8g2.print(kTxt);
  int cx = (OLED_WIDTH - u8g2.getStrWidth(tTxt.c_str())) / 2;
  u8g2.setCursor(cx, 62);
  u8g2.print(tTxt);
  int rx = OLED_WIDTH - u8g2.getStrWidth(mTxt.c_str()) - 4;
  u8g2.setCursor(rx, 62);
  u8g2.print(mTxt);

  u8g2.sendBuffer();
}

// ---- Alert screen: big bold ALERT title + ACTION text, blinks for visibility ----
void drawOLEDAlertFrame(bool inverted) {
  u8g2.clearBuffer();

  if (inverted) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, OLED_WIDTH, OLED_HEIGHT);
    u8g2.setDrawColor(0);
  } else {
    u8g2.setDrawColor(1);
  }

  u8g2.drawFrame(0, 0, OLED_WIDTH, OLED_HEIGHT);

  int yPos = 15;

  // ALERT title - bold, wrapped, always centered, never overflows
  u8g2.setFont(u8g2_font_7x13B_tr);
  for (int i = 0; i < alertTitleLineCount; i++) {
    int w = u8g2.getStrWidth(alertTitleLines[i].c_str());
    u8g2.setCursor((OLED_WIDTH - w) / 2, yPos);
    u8g2.print(alertTitleLines[i]);
    yPos += 13;
  }

  yPos += 3;
  u8g2.drawHLine(6, yPos, OLED_WIDTH - 12);
  yPos += 12;

  // ACTION text - regular font, wrapped, centered
  u8g2.setFont(u8g2_font_6x10_tf);
  for (int i = 0; i < alertActionLineCount; i++) {
    int w = u8g2.getStrWidth(alertActionLines[i].c_str());
    u8g2.setCursor((OLED_WIDTH - w) / 2, yPos);
    u8g2.print(alertActionLines[i]);
    yPos += 11;
  }

  u8g2.setDrawColor(1); // reset for next draw
  u8g2.sendBuffer();
}

// Precomputes the wrapped lines once, then starts the blink cycle
void triggerOLEDAlert(String oledTitle, String oledAction) {
  u8g2.setFont(u8g2_font_7x13B_tr);
  alertTitleLineCount = wrapText(oledTitle, OLED_WIDTH - 12, alertTitleLines, 2);

  u8g2.setFont(u8g2_font_6x10_tf);
  alertActionLineCount = wrapText(oledAction, OLED_WIDTH - 10, alertActionLines, 2);

  oledShowingAlert = true;
  oledAlertStart = millis();
  oledBlinkState = true;
  lastBlink = millis();

  drawOLEDAlertFrame(true);
}

// =====================================================================
//                    UNIFIED ALERT FUNCTION (Serial + BT + OLED + Web)
// =====================================================================
// oledTitle / oledAction should be SHORT & plain text (no emoji - fonts
// can't render them), e.g. title:"FALL DETECTED", action:"Inspect now!"
void sendAlert(String alertType, String sensorName, String actionLine, String oledTitle, String oledAction) {
  String dateLine, timeLine;
  if (timeOK) {
    dateLine = "Date: " + getDayOnly() + ", " + getDateOnly();
    timeLine = "Time: " + getTimeOnly();
  } else {
    dateLine = "Time: Not Synced";
    timeLine = "";
  }

  // ---- Serial Output ----
  Serial.println(alertType);
  Serial.println("Sensor: " + sensorName);
  Serial.println(dateLine);
  if (timeLine != "") Serial.println(timeLine);
  Serial.println(actionLine);
  Serial.println("");
  Serial.println("");

  // ---- Bluetooth Output (identical format) ----
  bt.println(alertType);
  bt.println("Sensor: " + sensorName);
  bt.println(dateLine);
  if (timeLine != "") bt.println(timeLine);
  bt.println(actionLine);
  bt.println("");
  bt.println("");

  // ---- OLED Output ----
  triggerOLEDAlert(oledTitle, oledAction);

  // ---- Web Dashboard Banner ----
  lastAlertType   = alertType;
  lastAlertSensor = sensorName;
  lastAlertAction = actionLine;
  lastAlertTime   = timeOK ? getTime() : "Time not synced";
  alertActiveUntil = millis() + ALERT_ACTIVE_DURATION;
}

// =====================================================================
//                        WIFI DASHBOARD (WEB SERVER)
// =====================================================================
const char dashboardHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Alert System Dashboard</title>
<style>
  * { box-sizing: border-box; }
  body {
    background: radial-gradient(circle at top, #182236 0%, #0a0e17 65%);
    color: #e8ecf4; font-family: 'Segoe UI', Arial, sans-serif;
    margin: 0; padding: 18px 16px 40px;
  }
  h1 { text-align: center; font-size: 22px; margin: 0 0 2px; letter-spacing: 1px;
       background: linear-gradient(90deg,#4fd1c5,#8b7bff); -webkit-background-clip: text;
       background-clip: text; color: transparent; }
  .sub { text-align: center; color: #7a86a0; margin-bottom: 18px; font-size: 12px; }

  /* ===== ALERT & ACTION BANNER ===== */
  #alertBanner {
    max-width: 900px; margin: 0 auto 18px; border-radius: 16px; padding: 20px 22px;
    display: flex; align-items: center; gap: 16px; transition: all .3s ease;
    border: 1px solid #26492f; background: linear-gradient(135deg,#122318,#0f1c14);
  }
  #alertBanner .icon { font-size: 34px; line-height: 1; flex-shrink: 0; }
  #alertBanner .text { flex: 1; min-width: 0; }
  #alertBanner .kicker { font-size: 11px; text-transform: uppercase; letter-spacing: 2px; opacity: .8; }
  #alertBanner .title { font-size: 21px; font-weight: 800; margin: 4px 0 6px; word-wrap: break-word; }
  #alertBanner .action { font-size: 15px; font-weight: 600; opacity: .95; word-wrap: break-word; }
  #alertBanner .meta { font-size: 11px; margin-top: 8px; opacity: .65; }

  #alertBanner.normal { color: #bdf5d6; }
  #alertBanner.normal .icon { color: #48d597; }

  #alertBanner.active {
    border-color: #ff5c5c; background: linear-gradient(135deg,#3a0f12,#2a0a0d);
    color: #ffe3e3; animation: pulseGlow 1s ease-in-out infinite;
  }
  #alertBanner.active .icon { color: #ff5c5c; }
  #alertBanner.active .title { color: #ff8080; }

  @keyframes pulseGlow {
    0%   { box-shadow: 0 0 0px rgba(255,92,92,0.6); }
    50%  { box-shadow: 0 0 26px rgba(255,92,92,0.85); }
    100% { box-shadow: 0 0 0px rgba(255,92,92,0.6); }
  }

  .grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    gap: 14px; max-width: 900px; margin: 0 auto;
  }
  .card {
    background: #171d2e; border-radius: 12px; padding: 16px;
    border: 1px solid #262f45; text-align: center;
  }
  .card .label { font-size: 12px; color: #8a94a8; text-transform: uppercase; letter-spacing: 1px; }
  .card .value { font-size: 20px; font-weight: 600; margin-top: 8px; }
  .ok { color: #48d597; }
  .warn { color: #ff5c5c; }
  .neutral { color: #e8ecf4; }
  .time-card { grid-column: 1 / -1; }
  .time-card .value { font-size: 34px; }
  footer { text-align: center; margin-top: 22px; color: #566; font-size: 11px; }
</style>
</head>
<body>
  <h1>Alert Detection System</h1>
  <div class="sub">Live status - auto refreshing every 2s</div>

  <div id="alertBanner" class="normal">
    <div class="icon" id="alertIcon">&#9989;</div>
    <div class="text">
      <div class="kicker" id="alertKicker">Current Status</div>
      <div class="title" id="alertTitle">All Normal</div>
      <div class="action" id="alertAction">System monitoring normally</div>
      <div class="meta" id="alertMeta">Sensor: -- &middot; Time: --</div>
    </div>
  </div>

  <div class="grid">
    <div class="card time-card">
      <div class="label">Current Time</div>
      <div class="value neutral" id="time">--:--:--</div>
      <div class="sub" id="dayDate">--</div>
    </div>
    <div class="card"><div class="label">WiFi</div><div class="value" id="wifi">--</div></div>
    <div class="card"><div class="label">IP Address</div><div class="value neutral" id="ip">--</div></div>
    <div class="card"><div class="label">Knock (D4)</div><div class="value" id="knock">--</div></div>
    <div class="card"><div class="label">Tilt (D5)</div><div class="value" id="tilt">--</div></div>
    <div class="card"><div class="label">MPU6050</div><div class="value" id="mpu">--</div></div>
    <div class="card"><div class="label">LED</div><div class="value" id="led">--</div></div>
    <div class="card"><div class="label">Accel X</div><div class="value neutral" id="ax">--</div></div>
    <div class="card"><div class="label">Accel Y</div><div class="value neutral" id="ay">--</div></div>
    <div class="card"><div class="label">Accel Z</div><div class="value neutral" id="az">--</div></div>
  </div>
  <footer>ESP32 Alert System &middot; Knock + Tilt + MPU6050</footer>

<script>
async function refresh() {
  try {
    const r = await fetch('/data');
    const d = await r.json();

    document.getElementById('time').textContent = d.time;
    document.getElementById('dayDate').textContent = d.day + ', ' + d.date;
    document.getElementById('wifi').textContent = d.wifi;
    document.getElementById('wifi').className = 'value ' + (d.wifi === 'Connected' ? 'ok' : 'warn');
    document.getElementById('ip').textContent = d.ip;
    document.getElementById('knock').textContent = d.knock;
    document.getElementById('knock').className = 'value ' + (d.knock === 'Normal' ? 'ok' : 'warn');
    document.getElementById('tilt').textContent = d.tilt;
    document.getElementById('tilt').className = 'value ' + (d.tilt === 'LEVEL' ? 'ok' : 'warn');
    document.getElementById('mpu').textContent = d.mpu;
    document.getElementById('mpu').className = 'value ' + (d.mpu === 'Normal' ? 'ok' : 'warn');
    document.getElementById('led').textContent = d.led;
    document.getElementById('led').className = 'value ' + (d.led === 'OFF' ? 'ok' : 'warn');
    document.getElementById('ax').textContent = d.ax + ' g';
    document.getElementById('ay').textContent = d.ay + ' g';
    document.getElementById('az').textContent = d.az + ' g';

    const banner = document.getElementById('alertBanner');
    document.getElementById('alertTitle').textContent = d.alertTitle;
    document.getElementById('alertAction').textContent = d.alertAction;
    document.getElementById('alertMeta').textContent = 'Sensor: ' + d.alertSensor + ' \u00b7 Time: ' + d.alertTime;

    if (d.alertActive) {
      banner.className = 'active';
      document.getElementById('alertKicker').textContent = 'ALERT ACTIVE';
      document.getElementById('alertIcon').innerHTML = '&#9888;&#65039;';
    } else {
      banner.className = 'normal';
      document.getElementById('alertKicker').textContent = 'Last Event';
      document.getElementById('alertIcon').innerHTML = '&#9989;';
    }
  } catch (e) { console.log('refresh failed', e); }
}
setInterval(refresh, 2000);
refresh();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", dashboardHTML);
}

void handleData() {
  String tiltStatus;
  if (tiltAlertSent) tiltStatus = "TILTED";
  else if (isTilted) tiltStatus = "Verifying";
  else tiltStatus = "LEVEL";

  bool alertActive = millis() < alertActiveUntil;

  String json = "{";
  json += "\"time\":\"" + (timeOK ? getTimeOnly() : String("N/A")) + "\",";
  json += "\"date\":\"" + (timeOK ? getDateOnly() : String("N/A")) + "\",";
  json += "\"day\":\"" + (timeOK ? getDayOnly() : String("N/A")) + "\",";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\",";
  json += "\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("N/A")) + "\",";
  json += "\"knock\":\"" + String(digitalRead(KNOCK_PIN) ? "FALL DETECTED" : "Normal") + "\",";
  json += "\"tilt\":\"" + tiltStatus + "\",";
  json += "\"mpu\":\"" + String(mpuAlertSent ? "ALERT" : "Normal") + "\",";
  json += "\"ax\":" + String(ax, 2) + ",";
  json += "\"ay\":" + String(ay, 2) + ",";
  json += "\"az\":" + String(az, 2) + ",";
  json += "\"led\":\"" + String(digitalRead(LED_PIN) ? "ON" : "OFF") + "\",";
  json += "\"alertActive\":" + String(alertActive ? "true" : "false") + ",";
  json += "\"alertTitle\":\"" + jsonSafe(lastAlertType) + "\",";
  json += "\"alertSensor\":\"" + jsonSafe(lastAlertSensor) + "\",";
  json += "\"alertAction\":\"" + jsonSafe(lastAlertAction) + "\",";
  json += "\"alertTime\":\"" + jsonSafe(lastAlertTime) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web dashboard started!");
}

void setup() {
  Serial.begin(115200);

  Serial.println("========================================");
  Serial.println("   COMPLETE ALERT DETECTION SYSTEM");
  Serial.println("   KNOCK + TILT + MPU6050 + OLED + WEB");
  Serial.println("========================================\n");

  // ===== OLED (init early so it can show boot progress) =====
  initOLED();

  // ===== WIFI =====
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  for (int i = 0; i < 15 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    configTime(gmtOffset_sec, 0, ntpServer);
    Serial.print("Syncing Time");
    for (int i = 0; i < 8; i++) {
      struct tm t;
      if (getLocalTime(&t)) {
        timeOK = true;
        break;
      }
      delay(1000);
      Serial.print(".");
    }
    if (timeOK) {
      Serial.println("\nTime Synced!");
      Serial.println(getTime());
    } else {
      Serial.println("\nTime Sync Failed!");
    }

    setupWebServer();
  } else {
    Serial.println("\nWiFi Failed! Time and dashboard unavailable.");
  }

  // ===== MPU6050 =====
  Serial.println("\nInitializing MPU6050...");
  mpuInitialized = initMPU6050();
  if (mpuInitialized) {
    readMPU6050();
    prevAx = ax;
    prevAy = ay;
    prevAz = az;
  }

  // ===== BLUETOOTH =====
  bt.begin("Alert_System");
  Serial.println("Bluetooth Started: Alert_System");

  bt.println("========================================");
  bt.println("   COMPLETE ALERT DETECTION SYSTEM");
  bt.println("========================================");
  bt.println("D4: Heavy Weight Fall (Knock)");
  bt.println("D5: Land Tilt (1.5 sec delay) - REVERSED LOGIC (LOW=Tilt)");
  if (mpuInitialized) {
    bt.println("MPU6050: Sudden Movement");
    bt.println("   Threshold: " + String(MPU_THRESHOLD) + "g");
  } else {
    bt.println("MPU6050: NOT DETECTED!");
  }
  if (WiFi.status() == WL_CONNECTED) {
    bt.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  }
  bt.println("========================================");
  bt.println("Type 'help' for commands");
  bt.println("========================================");

  // ===== PINS & INTERRUPTS =====
  pinMode(KNOCK_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(KNOCK_PIN, isrKnock, RISING);

  Serial.println("\nSystem Ready!");
  Serial.println("D4: Heavy Weight Fall Detection");
  Serial.println("D5: Land Tilt Detection (1.5 sec delay) - REVERSED (LOW=Tilt)");
  if (mpuInitialized) {
    Serial.println("MPU6050: Sudden Movement Detection");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  }
  Serial.println("========================================\n");
  Serial.println("Waiting for alerts...\n");

  // draw first normal OLED screen
  oledNormalDisplay();
  lastOledUpdate = millis();
}

void loop() {
  // ===== 1. CHECK KNOCK SENSOR (D4) =====
  if (knock) {
    digitalWrite(LED_PIN, HIGH);
    knock = false;

    if (millis() - lastNotif > 3000) {
      sendAlert(
        "!!! HEAVY WEIGHT FALL DETECTED !!!",
        "KY-031 Knock Module (D4)",
        "Action: Immediate inspection required",
        "FALL DETECTED",
        "Inspect area now!"
      );
      lastNotif = millis();
    }
  }

  // ===== 2. CHECK TILT SENSOR (D5) - REVERSED LOGIC =====
  int currentTiltState = digitalRead(TILT_PIN);

  // REVERSED: LOW means tilted (originally HIGH meant tilted)
  if (currentTiltState == LOW) {
    if (tiltStartTime == 0) {
      tiltStartTime = millis();
      isTilted = true;
      tiltAlertSent = false;
    }

    if (!tiltAlertSent && (millis() - tiltStartTime >= TILT_VERIFICATION_TIME)) {
      tiltAlertSent = true;
      digitalWrite(LED_PIN, HIGH);

      if (millis() - lastTiltNotif > 3000) {
        sendAlert(
          "!!! THE LAND IS TILTED !!!",
          "Mercury Tilt Sensor (D5)",
          "Action: Check land stability immediately",
          "LAND TILTED",
          "Check stability now!"
        );
        lastTiltNotif = millis();
      }
    }
  } else {
    if (isTilted && tiltAlertSent) {
      if (millis() - lastTiltNotif > 3000) {
        sendAlert(
          "LAND IS LEVEL AGAIN",
          "Mercury Tilt Sensor (D5)",
          "Action: No action needed",
          "LEVEL AGAIN",
          "No action needed"
        );
        lastTiltNotif = millis();
      }
    }
    isTilted = false;
    tiltAlertSent = false;
    tiltStartTime = 0;
  }

  // ===== 3. CHECK MPU6050 =====
  if (mpuInitialized && (millis() - lastMPURead >= MPU_CHECK_INTERVAL)) {
    readMPU6050();

    if (detectSuddenMovement()) {
      mpuAlertSent = true;
      digitalWrite(LED_PIN, HIGH);

      if (millis() - lastMPUNotif > MPU_COOLDOWN) {
        sendAlert(
          "!!! SUDDEN MOVEMENT DETECTED !!!",
          "MPU6050 Accelerometer",
          "Action: Check for impact or fall",
          "SUDDEN MOVEMENT",
          "Check for impact!"
        );
        lastMPUNotif = millis();
      }
    } else {
      mpuAlertSent = false;
    }

    prevAx = ax;
    prevAy = ay;
    prevAz = az;

    lastMPURead = millis();
  }

  // ===== LED OFF AFTER TIMEOUT =====
  if (millis() - lastKnock > 300 && !tiltAlertSent && !mpuAlertSent) {
    digitalWrite(LED_PIN, LOW);
  }

  // ===== OLED UPDATE =====
  if (oledShowingAlert) {
    if (millis() - oledAlertStart > OLED_ALERT_DURATION) {
      oledShowingAlert = false; // go back to normal screen next cycle
    } else if (millis() - lastBlink > BLINK_INTERVAL) {
      oledBlinkState = !oledBlinkState;
      lastBlink = millis();
      drawOLEDAlertFrame(oledBlinkState);
    }
  } else if (millis() - lastOledUpdate > OLED_NORMAL_REFRESH) {
    oledNormalDisplay();
    lastOledUpdate = millis();
  }

  // ===== WEB SERVER =====
  server.handleClient();

  // ===== BLUETOOTH COMMANDS =====
  if (bt.available()) {
    String c = bt.readString();
    c.trim();

    if (c == "status" || c == "STATUS") {
      bt.println("========================================");
      bt.println("SYSTEM STATUS");
      bt.println("========================================");
      bt.println("Status: RUNNING");
      bt.println("Device: ESP32");
      bt.println("========================================");
      bt.println("D4 (Knock): " + String(digitalRead(KNOCK_PIN) ? "FALL DETECTED" : "Normal"));

      if (tiltAlertSent) {
        bt.println("D5 (Tilt): TILTED (Verified) - REVERSED LOGIC");
      } else if (isTilted) {
        unsigned long elapsed = millis() - tiltStartTime;
        bt.println("D5 (Tilt): Verifying... " + String(elapsed / 1000) + "s / 1.5s");
      } else {
        bt.println("D5 (Tilt): LEVEL");
      }

      if (mpuInitialized) {
        bt.println("========================================");
        bt.println("MPU6050 DATA:");
        bt.println("   " + getMPUStatus());
        bt.println("   Threshold: " + String(MPU_THRESHOLD) + "g");
        bt.println(mpuAlertSent ? "   ALERT: Sudden Movement Detected!" : "   Status: Normal");
      }

      bt.println("========================================");
      bt.println("LED: " + String(digitalRead(LED_PIN) ? "ON" : "OFF"));
      bt.println("========================================");
      if (timeOK) {
        bt.println("Day: " + getDayOnly());
        bt.println("Date: " + getDateOnly());
        bt.println("Time: " + getTimeOnly());
      } else {
        bt.println("Time: NOT SYNCED");
      }
      bt.println("========================================");
      bt.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "No"));
      if (WiFi.status() == WL_CONNECTED) {
        bt.println("IP: " + WiFi.localIP().toString());
        bt.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
      }
      bt.println("========================================");
      bt.println("Tilt Logic: REVERSED (LOW = Tilted)");
      bt.println("========================================");
    }
    else if (c == "mpu" || c == "MPU") {
      if (mpuInitialized) {
        readMPU6050();
        bt.println("========================================");
        bt.println("MPU6050 REAL-TIME DATA");
        bt.println("========================================");
        bt.println("Accelerometer:");
        bt.println("   X: " + String(ax, 2) + "g");
        bt.println("   Y: " + String(ay, 2) + "g");
        bt.println("   Z: " + String(az, 2) + "g");
        bt.println("========================================");
        bt.println("Gyroscope:");
        bt.println("   X: " + String(gx, 2) + " deg/s");
        bt.println("   Y: " + String(gy, 2) + " deg/s");
        bt.println("   Z: " + String(gz, 2) + " deg/s");
        bt.println("========================================");
        bt.println("Change: " + getMPUChange());
        bt.println("Threshold: " + String(MPU_THRESHOLD) + "g");
        bt.println(detectSuddenMovement() ? "SUDDEN MOVEMENT DETECTED!" : "Normal");
        bt.println("========================================");
      } else {
        bt.println("MPU6050 Not Detected!");
      }
    }
    else if (c == "time" || c == "TIME") {
      if (timeOK) {
        bt.println("========================================");
        bt.println("CURRENT INDIAN TIME (IST)");
        bt.println("========================================");
        bt.println("Day: " + getDayOnly());
        bt.println("Date: " + getDateOnly());
        bt.println("Time: " + getTimeOnly());
        bt.println("Full: " + getTime());
        bt.println("Time Zone: GMT+5:30 (IST)");
        bt.println("========================================");
      } else {
        bt.println("Time Not Synced!");
      }
    }
    else if (c == "d4" || c == "D4") {
      bt.println("D4 (Knock): " + String(digitalRead(KNOCK_PIN) ? "FALL DETECTED" : "Normal"));
    }
    else if (c == "d5" || c == "D5") {
      if (tiltAlertSent) {
        bt.println("D5 (Tilt): TILTED (Verified for 1.5s) - REVERSED LOGIC");
      } else if (isTilted) {
        unsigned long elapsed = millis() - tiltStartTime;
        bt.println("D5 (Tilt): Verifying... " + String(elapsed / 1000) + "s / 1.5s");
      } else {
        bt.println("D5 (Tilt): LEVEL");
      }
    }
    else if (c == "threshold" || c == "THRESHOLD") {
      bt.println("MPU6050 Threshold: " + String(MPU_THRESHOLD) + "g");
      bt.println("Lower = More Sensitive");
      bt.println("Higher = Less Sensitive");
    }
    else if (c == "wifi" || c == "WIFI") {
      if (WiFi.status() == WL_CONNECTED) {
        bt.println("WiFi: Connected");
        bt.println("IP: " + WiFi.localIP().toString());
        bt.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
      } else {
        bt.println("WiFi: Not Connected");
      }
    }
    else if (c == "led on") {
      digitalWrite(LED_PIN, HIGH);
      bt.println("LED Turned ON");
    }
    else if (c == "led off") {
      digitalWrite(LED_PIN, LOW);
      bt.println("LED Turned OFF");
    }
    else if (c == "reset" || c == "RESET") {
      lastKnock = 0;
      knock = false;
      tiltStartTime = 0;
      isTilted = false;
      tiltAlertSent = false;
      mpuAlertSent = false;
      oledShowingAlert = false;
      digitalWrite(LED_PIN, LOW);
      bt.println("System Reset Done!");
    }
    else if (c == "help" || c == "HELP" || c == "?") {
      bt.println("========================================");
      bt.println("AVAILABLE COMMANDS");
      bt.println("========================================");
      bt.println("  status     - System status");
      bt.println("  mpu        - MPU6050 data");
      bt.println("  time       - Show Indian time (IST)");
      bt.println("  d4         - Check Knock sensor (D4)");
      bt.println("  d5         - Check Tilt sensor (D5)");
      bt.println("  threshold  - Show MPU threshold");
      bt.println("  wifi       - Show WiFi + dashboard link");
      bt.println("  led on     - Turn LED ON");
      bt.println("  led off    - Turn LED OFF");
      bt.println("  reset      - Reset system");
      bt.println("  help       - This menu");
      bt.println("========================================");
      bt.println("TILT DELAY: 1.5 seconds");
      bt.println("Tilt Logic: REVERSED (LOW = Tilted)");
      bt.println("MPU THRESHOLD: " + String(MPU_THRESHOLD) + "g");
      bt.println("========================================");
    }
    else if (c != "") {
      bt.println("Unknown command! Type 'help'");
    }
  }

  delay(5);
}
