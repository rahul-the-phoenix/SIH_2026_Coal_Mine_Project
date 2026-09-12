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
#define KNOCK_PIN 32
#define TILT_PIN 5
#define LED_PIN 2
#define BUZZER_PIN 13

// MPU6050 Object
MPU6050 mpu(Wire);
const uint8_t MPU_ADDR = 0x68;

BluetoothSerial bt;

// ===== OLED (1.3" SH1106, 128x64, I2C) =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ===== WEB SERVER =====
WebServer server(80);

volatile bool knock = false;
unsigned long lastKnock = 0;
unsigned long lastNotif = 0;

// ===== SW-180P VIBRATION SENSOR VARIABLES =====
// SW-180P is a spring-based switch: stable = HIGH, vibration = LOW pulse
// Mechanical bounce is high, so we use multi-sample verification.
volatile unsigned int knockPulseCount = 0;
volatile unsigned long lastKnockPulseTime = 0;

const unsigned long KNOCK_PULSE_WINDOW = 50;    // 50ms between valid pulses
const unsigned int  KNOCK_MIN_PULSES  = 3;       // need 4+ pulses to confirm
const unsigned long KNOCK_CHECK_INTERVAL = 500;  // check every 500ms
const unsigned long KNOCK_ALERT_COOLDOWN = 3000; // 3s between alerts

unsigned long lastKnockCheck = 0;

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
unsigned long mpuAlertUntil = 0;
bool mpuInitialized = false;

const float MPU_THRESHOLD = 0.5;    // increased from 1.0 to reduce false alerts
const unsigned long MPU_CHECK_INTERVAL = 100;
const unsigned long MPU_COOLDOWN = 3000;

// ===== MPU LOW-PASS FILTER =====
float axFiltered = 0, ayFiltered = 0, azFiltered = 0;
const float MPU_ALPHA = 0.4;

// ===== TIME VARIABLES =====
bool timeOK = false;

// ===== OLED STATE VARIABLES =====
unsigned long lastOledUpdate = 0;
const unsigned long OLED_NORMAL_REFRESH = 500;

bool oledShowingAlert = false;
unsigned long oledAlertStart = 0;
const unsigned long OLED_ALERT_DURATION = 6000;
unsigned long lastBlink = 0;
const unsigned long BLINK_INTERVAL = 400;
bool oledBlinkState = false;

bool oledShowingStatus = false;
unsigned long oledStatusStart = 0;
const unsigned long OLED_STATUS_DURATION = 5000;

String alertTitleLines[2];
int alertTitleLineCount = 0;
String alertActionLines[2];
int alertActionLineCount = 0;

// ===== LAST ALERT INFO =====
String lastAlertType   = "No Alerts Yet";
String lastAlertSensor = "--";
String lastAlertAction = "System monitoring normally";
String lastAlertTime   = "--";
unsigned long alertActiveUntil = 0;
const unsigned long ALERT_ACTIVE_DURATION = 8000;

// ===== BUZZER VARIABLES =====
bool buzzerBeeping = false;
bool buzzerPinState = false;
unsigned long buzzerStopAt = 0;
unsigned long lastBuzzerToggle = 0;
const unsigned long BUZZER_BEEP_INTERVAL = 80;
const unsigned long BUZZER_ALERT_DURATION = 4000;

// ===== INTERRUPT (SW-180P triggers on FALLING edge) =====
void IRAM_ATTR isrKnock() {
  unsigned long now = millis();
  if (now - lastKnockPulseTime > KNOCK_PULSE_WINDOW) {
    knockPulseCount++;
    lastKnockPulseTime = now;
  }
}

// ===== BUZZER FUNCTIONS =====
void startBuzzer() {
  buzzerBeeping = true;
  buzzerStopAt = millis() + BUZZER_ALERT_DURATION;
  lastBuzzerToggle = millis();
  buzzerPinState = true;
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer() {
  buzzerBeeping = false;
  buzzerPinState = false;
  digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer() {
  if (!buzzerBeeping) return;
  if (millis() > buzzerStopAt) {
    stopBuzzer();
    return;
  }
  if (millis() - lastBuzzerToggle >= BUZZER_BEEP_INTERVAL) {
    buzzerPinState = !buzzerPinState;
    digitalWrite(BUZZER_PIN, buzzerPinState ? HIGH : LOW);
    lastBuzzerToggle = millis();
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
  float rawAx = mpu.getAccX();
  float rawAy = mpu.getAccY();
  float rawAz = mpu.getAccZ();

  // Low-pass filter to reduce noise
  axFiltered = MPU_ALPHA * axFiltered + (1.0 - MPU_ALPHA) * rawAx;
  ayFiltered = MPU_ALPHA * ayFiltered + (1.0 - MPU_ALPHA) * rawAy;
  azFiltered = MPU_ALPHA * azFiltered + (1.0 - MPU_ALPHA) * rawAz;

  ax = axFiltered;
  ay = ayFiltered;
  az = azFiltered;

  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();
}

bool detectSuddenMovement() {
  axChange = abs(ax - prevAx);
  ayChange = abs(ay - prevAy);
  azChange = abs(az - prevAz);
  float magnitude = sqrt(axChange*axChange + ayChange*ayChange + azChange*azChange);
  return magnitude > MPU_THRESHOLD;
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

String jsonSafe(String s) {
  s.replace("\\", "/");
  s.replace("\"", "'");
  s.replace("\n", " ");
  s.replace("\r", " ");
  return s;
}

bool anySensorIssue() {
  // SW-180P: stable = HIGH, triggered = LOW
  return (digitalRead(KNOCK_PIN) == LOW) || isTilted || mpuAlertSent;
}

// =====================================================================
//                              OLED HELPERS
// =====================================================================

void truncateFit(String &s, int maxWidth) {
  while (u8g2.getStrWidth(s.c_str()) > maxWidth && s.length() > 1) {
    s.remove(s.length() - 1);
  }
}

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
      cur = word;
      truncateFit(cur, maxWidth);
    }
    start = (spaceIdx == -1) ? len : spaceIdx + 1;
  }
  if (cur.length() > 0 && count < maxLines) {
    outLines[count++] = cur;
  }
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

  u8g2.drawFrame(0, 0, OLED_WIDTH, OLED_HEIGHT);

  u8g2.setFont(u8g2_font_helvB14_tr);
  String title = "SIH 2026";
  int tw = u8g2.getStrWidth(title.c_str());
  u8g2.setCursor((OLED_WIDTH - tw) / 2, 20);
  u8g2.print(title);

  u8g2.setFont(u8g2_font_helvB10_tr);
  String subtitle = "Bhumi Rakshha";
  int sw = u8g2.getStrWidth(subtitle.c_str());
  u8g2.setCursor((OLED_WIDTH - sw) / 2, 36);
  u8g2.print(subtitle);

  u8g2.drawHLine(8, 42, OLED_WIDTH - 16);

  u8g2.setFont(u8g2_font_6x10_tf);
  String startTxt = "Starting up...";
  int stw = u8g2.getStrWidth(startTxt.c_str());
  u8g2.setCursor((OLED_WIDTH - stw) / 2, 56);
  u8g2.print(startTxt);

  u8g2.sendBuffer();
  delay(1200);
}

void oledNormalDisplay() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_9x15B_tr);
  String t = timeOK ? getTimeOnly() : "--:--:-- --";
  int w = u8g2.getStrWidth(t.c_str());
  u8g2.setCursor((OLED_WIDTH - w) / 2, 20);
  u8g2.print(t);

  u8g2.setFont(u8g2_font_7x13_tf);
  String dd = timeOK ? (getDayShort() + ", " + getDateOnly()) : "Time Not Synced";
  truncateFit(dd, OLED_WIDTH - 4);
  int w2 = u8g2.getStrWidth(dd.c_str());
  u8g2.setCursor((OLED_WIDTH - w2) / 2, 34);
  u8g2.print(dd);

  u8g2.drawHLine(0, 40, OLED_WIDTH);

  u8g2.setFont(u8g2_font_7x13B_tr);
  String topStatus = anySensorIssue() ? "Checking..." : "No Problem Found";
  truncateFit(topStatus, OLED_WIDTH - 4);
  int tw = u8g2.getStrWidth(topStatus.c_str());
  u8g2.setCursor((OLED_WIDTH - tw) / 2, 56);
  u8g2.print(topStatus);

  u8g2.sendBuffer();
}

void drawOLEDStatusScreen() {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, OLED_WIDTH, OLED_HEIGHT);
  u8g2.setFont(u8g2_font_7x13B_tr);
  String header = "SYSTEM STATUS";
  int hw = u8g2.getStrWidth(header.c_str());
  u8g2.setCursor((OLED_WIDTH - hw) / 2, 11);
  u8g2.print(header);
  u8g2.drawHLine(4, 14, OLED_WIDTH - 8);
  u8g2.setFont(u8g2_font_5x8_tf);
  int y = 23;
  String wifiLine = "WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  u8g2.setCursor(4, y); u8g2.print(wifiLine); y += 8;
  String btLine = "Bluetooth: " + String(bt.hasClient() ? "Connected" : "Not Connected");
  u8g2.setCursor(4, y); u8g2.print(btLine); y += 8;
  // SW-180P: LOW = vibration detected
  String knockLine = "D12 SW-180P: " + String(digitalRead(KNOCK_PIN) == LOW ? "VIB!" : "Normal");
  u8g2.setCursor(4, y); u8g2.print(knockLine); y += 8;
  String tiltLine;
  if (tiltAlertSent) tiltLine = "D5 Tilt: TILTED!";
  else if (isTilted) tiltLine = "D5 Tilt: Checking...";
  else tiltLine = "D5 Tilt: LEVEL";
  u8g2.setCursor(4, y); u8g2.print(tiltLine); y += 8;
  String mpuLine = mpuInitialized ? ("MPU6050: " + String(mpuAlertSent ? "ALERT!" : "Normal")) : "MPU6050: Not Found";
  u8g2.setCursor(4, y); u8g2.print(mpuLine); y += 8;
  String ledLine = "LED: " + String(digitalRead(LED_PIN) ? "ON" : "OFF");
  u8g2.setCursor(4, y); u8g2.print(ledLine); y += 8;
  u8g2.sendBuffer();
}

void triggerOLEDStatus() {
  oledShowingAlert = false;
  oledShowingStatus = true;
  oledStatusStart = millis();
  drawOLEDStatusScreen();
  lastOledUpdate = millis();
}

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
  u8g2.drawFrame(2, 2, OLED_WIDTH - 4, OLED_HEIGHT - 4);

  int yPos = 16;
  u8g2.setFont(u8g2_font_8x13B_tr);
  for (int i = 0; i < alertTitleLineCount; i++) {
    int w = u8g2.getStrWidth(alertTitleLines[i].c_str());
    u8g2.setCursor((OLED_WIDTH - w) / 2, yPos);
    u8g2.print(alertTitleLines[i]);
    yPos += 14;
  }
  yPos += 1;
  u8g2.drawHLine(8, yPos, OLED_WIDTH - 16);
  yPos += 9;
  u8g2.setFont(u8g2_font_6x12_tf);
  for (int i = 0; i < alertActionLineCount; i++) {
    int w = u8g2.getStrWidth(alertActionLines[i].c_str());
    u8g2.setCursor((OLED_WIDTH - w) / 2, yPos);
    u8g2.print(alertActionLines[i]);
    yPos += 12;
  }
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

void triggerOLEDAlert(String oledTitle, String oledAction) {
  oledShowingStatus = false;
  u8g2.setFont(u8g2_font_8x13B_tr);
  alertTitleLineCount = wrapText(oledTitle, OLED_WIDTH - 16, alertTitleLines, 2);
  u8g2.setFont(u8g2_font_6x12_tf);
  alertActionLineCount = wrapText(oledAction, OLED_WIDTH - 14, alertActionLines, 2);
  oledShowingAlert = true;
  oledAlertStart = millis();
  oledBlinkState = true;
  lastBlink = millis();
  drawOLEDAlertFrame(true);
}

// =====================================================================
//                    UNIFIED ALERT FUNCTION
// =====================================================================

void sendAlert(String alertType, String sensorName, String actionLine,
               String oledTitle, String oledAction,
               String titleEmoji, String actionEmoji,
               bool buzz = true) {
  String titleLine = titleEmoji + titleEmoji + titleEmoji + " " + alertType + " " + titleEmoji + titleEmoji + titleEmoji;
  String sensorLine = "\xF0\x9F\x93\x8D " + sensorName;
  String dateLine, timeLine;
  if (timeOK) {
    dateLine = "\xF0\x9F\x93\x85 " + getDayOnly() + ", " + getDateOnly();
    timeLine = "\xF0\x9F\x95\x90 " + getTimeOnly();
  } else {
    dateLine = "\xF0\x9F\x93\x85 Time Not Synced";
    timeLine = "";
  }
  String actionOut = actionEmoji + " Action: " + actionLine;

  Serial.println(titleLine);
  Serial.println(sensorLine);
  Serial.println(dateLine);
  if (timeLine != "") Serial.println(timeLine);
  Serial.println(actionOut);
  Serial.println("");
  Serial.println("");

  bt.println(titleLine);
  bt.println(sensorLine);
  bt.println(dateLine);
  if (timeLine != "") bt.println(timeLine);
  bt.println(actionOut);
  bt.println("");
  bt.println("");

  triggerOLEDAlert(oledTitle, oledAction);
  if (buzz) startBuzzer();

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
  * { box-sizing: border-box; margin: 0; padding: 0; }
  :root{
    --bg:#070a12; --card:#0f1524; --card2:#0c111d; --line:#1c2740;
    --line-soft:#161f34; --text:#eaf0fb; --muted:#7c8aa8;
    --accent:#4fd1c5; --accent2:#8b7bff; --good:#3fe0a5;
    --bad:#ff5470; --warn:#ffb84f;
  }
  html,body{ height:100%; }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Arial, sans-serif;
    min-height: 100vh; padding: 22px 16px 50px;
    position: relative; overflow-x: hidden;
  }
  .bg-glow{ position: fixed; inset: 0; z-index: -1; overflow: hidden; pointer-events: none; }
  .bg-glow span{ position: absolute; border-radius: 50%; filter: blur(90px); opacity: .35; }
  .bg-glow span:nth-child(1){ width: 380px; height: 380px; background: var(--accent); top: -120px; left: -100px; animation: floatBlob 12s ease-in-out infinite; }
  .bg-glow span:nth-child(2){ width: 320px; height: 320px; background: var(--accent2); bottom: -100px; right: -80px; animation: floatBlob 14s ease-in-out infinite reverse; }
  .bg-glow span:nth-child(3){ width: 260px; height: 260px; background: #2a5bff; top: 40%; left: 60%; opacity: .15; animation: floatBlob 18s ease-in-out infinite; }
  @keyframes floatBlob{ 0%,100% { transform: translate(0,0) scale(1); } 50% { transform: translate(30px,-25px) scale(1.08); } }
  .container { max-width: 1180px; margin: 0 auto; position: relative; }
  .header { text-align: center; padding: 18px 0 6px; }
  .header .badge{ display:inline-flex; align-items:center; gap:6px; font-size:11px; letter-spacing:2px; text-transform:uppercase; color: var(--accent); background: rgba(79,209,197,.08); border:1px solid rgba(79,209,197,.25); padding:4px 12px; border-radius: 999px; margin-bottom:10px; }
  .header .badge .dot{ width:6px; height:6px; border-radius:50%; background: var(--good); box-shadow: 0 0 8px var(--good); animation: blinkDot 1.6s ease-in-out infinite; }
  @keyframes blinkDot{ 0%,100%{opacity:1;} 50%{opacity:.35;} }
  .header h1 { font-size: 34px; font-weight: 800; background: linear-gradient(135deg, #4fd1c5 0%, #8b7bff 50%, #4fd1c5 100%); background-size: 200% 200%; -webkit-background-clip: text; background-clip: text; color: transparent; animation: gradientShift 4s ease-in-out infinite; letter-spacing: 1px; }
  @keyframes gradientShift { 0%, 100% { background-position: 0% 50%; } 50% { background-position: 100% 50%; } }
  .header .sub { color: var(--muted); font-size: 13px; margin-top: 6px; letter-spacing: 3px; text-transform: uppercase; }
  .btn-group { display: flex; justify-content: center; gap: 12px; flex-wrap: wrap; margin: 20px 0 24px; }
  .btn { background: linear-gradient(135deg, #141d34, #0d1424); border: 1px solid var(--line); color: #c2cde3; padding: 11px 24px; border-radius: 14px; font-size: 14px; font-weight: 600; cursor: pointer; transition: all .25s ease; display: flex; align-items: center; gap: 8px; }
  .btn:hover { transform: translateY(-2px); box-shadow: 0 10px 28px rgba(79, 209, 197, 0.18); border-color: var(--accent); color: #fff; }
  .btn:active { transform: scale(0.96); }
  .btn:disabled{ opacity:.7; cursor:default; transform:none; }
  .btn-primary { background: linear-gradient(135deg, var(--accent), var(--accent2)); border-color: transparent; color: #06111a; }
  .btn-primary:hover { box-shadow: 0 10px 34px rgba(139, 123, 255, 0.35); color:#06111a; }
  .btn .icon { font-size: 18px; }
  .status-banner { background: linear-gradient(160deg, var(--card), var(--card2)); border-radius: 20px; padding: 22px 26px; margin-bottom: 22px; border: 1px solid var(--line); display: flex; align-items: center; gap: 20px; flex-wrap: wrap; transition: all 0.5s ease; position: relative; overflow: hidden; }
  .status-banner::before{ content:""; position:absolute; inset:0; background: radial-gradient(600px circle at 0% 0%, rgba(79,209,197,.08), transparent 60%); pointer-events:none; }
  .icon-ring{ width: 62px; height:62px; border-radius:50%; display:flex; align-items:center; justify-content:center; font-size: 28px; background: rgba(79,209,197,.1); border: 2px solid rgba(79,209,197,.35); flex-shrink:0; }
  .status-banner .content { flex: 1; min-width: 220px; z-index:1; }
  .status-banner .label { font-size: 11px; text-transform: uppercase; letter-spacing: 2px; opacity: 0.65; font-weight:700; }
  .status-banner .title { font-size: 23px; font-weight: 800; margin: 5px 0 6px; }
  .status-banner .action { font-size: 15px; opacity: 0.9; }
  .status-banner .meta { font-size: 12px; opacity: 0.5; margin-top: 8px; }
  .status-banner.normal { border-color: rgba(63,224,165,.28); }
  .status-banner.normal .icon-ring{ background: rgba(63,224,165,.1); border-color: rgba(63,224,165,.4); }
  .status-banner.normal .title { color: var(--good); }
  .status-banner.active { border-color: rgba(255,84,112,.6); background: linear-gradient(160deg, #1c0f18, #150a10); animation: pulseGlow 1.2s ease-in-out infinite; }
  .status-banner.active .icon-ring{ background: rgba(255,84,112,.12); border-color: rgba(255,84,112,.55); }
  .status-banner.active .title { color: var(--bad); }
  @keyframes pulseGlow { 0%, 100% { box-shadow: 0 0 20px rgba(255, 84, 112, 0.35), 0 0 55px rgba(255, 84, 112, 0.18); } 50% { box-shadow: 0 0 40px rgba(255, 84, 112, 0.65), 0 0 100px rgba(255, 84, 112, 0.35); } }
  .stats-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(165px, 1fr)); gap: 14px; margin-bottom: 20px; }
  .stat-card { background: var(--card); border-radius: 16px; padding: 18px 16px; text-align: center; border: 1px solid var(--line-soft); transition: all 0.25s ease; position: relative; }
  .stat-card:hover { border-color: rgba(79,209,197,.4); transform: translateY(-3px); box-shadow: 0 10px 28px rgba(0,0,0,0.35); }
  .stat-card .stat-icon{ font-size:20px; margin-bottom:6px; }
  .stat-card .stat-label { font-size: 11px; text-transform: uppercase; letter-spacing: 1.5px; color: var(--muted); font-weight:600; }
  .stat-card .stat-value { font-size: 21px; font-weight: 800; margin-top: 8px; transition: color 0.3s; }
  .stat-card .stat-value.ok { color: var(--good); }
  .stat-card .stat-value.warn { color: var(--bad); }
  .stat-card .stat-value.neutral { color: var(--text); }
  .stat-card .stat-sub { font-size: 12px; color: #56628060; margin-top: 4px; color:#5b6a89; }
  .pulse-badge{ position:absolute; top:12px; right:12px; width:8px; height:8px; border-radius:50%; background: var(--good); }
  .pulse-badge.warn{ background: var(--bad); box-shadow:0 0 10px var(--bad); animation: pulseGlow 1s ease-in-out infinite; }
  .time-card { grid-column: 1 / -1; background: linear-gradient(135deg, var(--card), var(--card2)); border: 1px solid var(--line-soft); display:flex; align-items:center; justify-content:space-between; flex-wrap:wrap; gap:14px; text-align:left; padding: 20px 26px; }
  .time-card .stat-value { font-size: clamp(24px, 6vw, 40px); font-weight: 200; letter-spacing: clamp(1px, 0.5vw, 3px); color: var(--accent); font-variant-numeric: tabular-nums; white-space: nowrap; }
  .time-card .stat-sub { font-size: 14px; color: var(--muted); margin-top:4px; }
  .time-card .conn-pills{ display:flex; gap:8px; flex-wrap:wrap; }
  .pill{ font-size:12px; font-weight:700; padding:6px 12px; border-radius:999px; border:1px solid var(--line); display:flex; align-items:center; gap:6px; }
  .pill .d{ width:7px; height:7px; border-radius:50%; }
  .pill.ok .d{ background:var(--good); box-shadow:0 0 6px var(--good); }
  .pill.bad .d{ background:var(--bad); box-shadow:0 0 6px var(--bad); }
  .mpu-panel{ background: var(--card); border-radius: 18px; padding: 20px 22px; border: 1px solid var(--line-soft); }
  .mpu-panel .panel-title{ text-align:center; font-size: 12px; color: var(--muted); text-transform: uppercase; letter-spacing: 2px; margin-bottom: 14px; font-weight:700; }
  .mpu-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
  .mpu-item { background: var(--card2); border-radius: 12px; padding: 12px 8px; border: 1px solid var(--line-soft); text-align:center; }
  .mpu-item .axis { font-size: 11px; color: var(--muted); font-weight:700; letter-spacing:1px; }
  .mpu-item .val { font-size: 18px; font-weight: 700; margin: 6px 0 8px; }
  .bar-track{ height:6px; background:#0a0f1a; border-radius:4px; overflow:hidden; }
  .bar-fill{ height:100%; border-radius:4px; transition: width .3s ease, background .3s ease; }
  .bar-fill.x{ background: linear-gradient(90deg,var(--accent),var(--accent2)); }
  .bar-fill.y{ background: linear-gradient(90deg,var(--accent2),#ff7bd0); }
  .bar-fill.z{ background: linear-gradient(90deg,#ffb84f,var(--bad)); }
  .footer { text-align: center; color: #3a4a62; font-size: 12px; margin-top: 26px; padding-top: 18px; border-top: 1px solid var(--line-soft); letter-spacing: 1px; }
  @media (max-width: 600px) {
    .header h1 { font-size: 24px; }
    .status-banner { padding: 18px; }
    .status-banner .title { font-size: 18px; }
    .stat-card .stat-value { font-size: 18px; }
    .time-card { flex-direction:column; align-items:flex-start; }
    .time-card .stat-value { font-size: 30px; }
    .btn { padding: 9px 18px; font-size: 12px; }
    .mpu-grid { grid-template-columns: 1fr; }
  }
  ::-webkit-scrollbar { width: 6px; }
  ::-webkit-scrollbar-track { background: var(--bg); }
  ::-webkit-scrollbar-thumb { background: #24304a; border-radius: 10px; }
</style>
</head>
<body>
<div class="bg-glow"><span></span><span></span><span></span></div>

<div class="container">
  <div class="header">
    <div class="badge"><span class="dot"></span> LIVE MONITORING</div>
    <h1>⚠️ Alert Detection System</h1>
    <div class="sub">Vibration &middot; Tilt &middot; MPU6050</div>
  </div>

  <div class="btn-group">
    <button class="btn btn-primary" onclick="showOledStatus()">
      <span class="icon">📟</span> Show Status on OLED
    </button>
    <button class="btn" onclick="refreshNow()">
      <span class="icon">🔄</span> Refresh
    </button>
  </div>

  <div id="statusBanner" class="status-banner normal">
    <div class="icon-ring" id="bannerIcon">✅</div>
    <div class="content">
      <div class="label" id="bannerKicker">Current Status</div>
      <div class="title" id="bannerTitle">All Normal</div>
      <div class="action" id="bannerAction">System monitoring normally</div>
      <div class="meta" id="bannerMeta">Sensor: -- · Time: --</div>
    </div>
  </div>

  <div class="stats-grid" id="statsGrid">
    <div class="stat-card time-card">
      <div>
        <div class="stat-label">Current Time</div>
        <div class="stat-value neutral" id="timeDisplay">--:--:--</div>
        <div class="stat-sub" id="dateDisplay">--</div>
      </div>
      <div class="conn-pills">
        <div class="pill" id="wifiPill"><span class="d"></span><span id="wifiPillText">WiFi --</span></div>
        <div class="pill" id="ipPill"><span class="d ok"></span><span id="ipDisplay">--</span></div>
      </div>
    </div>

    <div class="stat-card">
      <div class="pulse-badge" id="knockDot"></div>
      <div class="stat-icon">📳</div>
      <div class="stat-label">Vibration (D12)</div>
      <div class="stat-value" id="knockStatus">--</div>
    </div>
    <div class="stat-card">
      <div class="pulse-badge" id="tiltDot"></div>
      <div class="stat-icon">📐</div>
      <div class="stat-label">Tilt (D5)</div>
      <div class="stat-value" id="tiltStatus">--</div>
    </div>
    <div class="stat-card">
      <div class="pulse-badge" id="mpuDot"></div>
      <div class="stat-icon">🎯</div>
      <div class="stat-label">MPU6050</div>
      <div class="stat-value" id="mpuStatus">--</div>
    </div>
    <div class="stat-card">
      <div class="stat-icon">💡</div>
      <div class="stat-label">LED</div>
      <div class="stat-value" id="ledStatus">--</div>
    </div>
  </div>

  <div class="mpu-panel">
    <div class="panel-title">MPU6050 Accelerometer</div>
    <div class="mpu-grid">
      <div class="mpu-item">
        <div class="axis">X-AXIS</div>
        <div class="val neutral" id="axVal">--</div>
        <div class="bar-track"><div class="bar-fill x" id="axBar" style="width:0%"></div></div>
      </div>
      <div class="mpu-item">
        <div class="axis">Y-AXIS</div>
        <div class="val neutral" id="ayVal">--</div>
        <div class="bar-track"><div class="bar-fill y" id="ayBar" style="width:0%"></div></div>
      </div>
      <div class="mpu-item">
        <div class="axis">Z-AXIS</div>
        <div class="val neutral" id="azVal">--</div>
        <div class="bar-track"><div class="bar-fill z" id="azBar" style="width:0%"></div></div>
      </div>
    </div>
  </div>

  <div class="footer">
    ESP32 Alert System &middot; Vibration + Tilt + MPU6050 &middot; v2.2
  </div>
</div>

<script>
function showOledStatus() {
  const btn = document.querySelector('.btn-primary');
  const old = btn.innerHTML;
  btn.innerHTML = '<span class="icon">⏳</span> Sending...';
  btn.disabled = true;
  fetch('/oledstatus')
    .then(() => {
      btn.innerHTML = '<span class="icon">✅</span> Sent to OLED';
      setTimeout(() => { btn.innerHTML = old; btn.disabled = false; }, 1500);
    })
    .catch(() => {
      btn.innerHTML = '<span class="icon">❌</span> Failed';
      setTimeout(() => { btn.innerHTML = old; btn.disabled = false; }, 1500);
    });
}

function refreshNow() {
  const btn = document.querySelectorAll('.btn')[1];
  const old = btn.innerHTML;
  btn.innerHTML = '<span class="icon">⏳</span> Refreshing...';
  refresh();
  setTimeout(() => { btn.innerHTML = old; }, 800);
}

function accelBarPct(g) {
  const clamped = Math.max(-2, Math.min(2, g));
  return Math.round(((clamped + 2) / 4) * 100);
}

let clockDate = null;

function parseServerDateTime(timeStr, dateStr) {
  if (!timeStr || timeStr === 'N/A' || !dateStr || dateStr === 'N/A') return null;
  const m = timeStr.match(/(\d{1,2}):(\d{2}):(\d{2})\s*([AP]M)/i);
  const dp = dateStr.split('/');
  if (!m || dp.length !== 3) return null;
  let hh = parseInt(m[1], 10);
  const mm = parseInt(m[2], 10);
  const ss = parseInt(m[3], 10);
  const ap = m[4].toUpperCase();
  if (ap === 'PM' && hh !== 12) hh += 12;
  if (ap === 'AM' && hh === 12) hh = 0;
  const day = parseInt(dp[0], 10);
  const month = parseInt(dp[1], 10) - 1;
  const year = parseInt(dp[2], 10);
  const d = new Date(year, month, day, hh, mm, ss);
  return isNaN(d.getTime()) ? null : d;
}

function formatClock(d) {
  let hh = d.getHours();
  const ap = hh >= 12 ? 'PM' : 'AM';
  hh = hh % 12; if (hh === 0) hh = 12;
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  return String(hh).padStart(2, '0') + ':' + mm + ':' + ss + ' ' + ap;
}

setInterval(() => {
  if (!clockDate) return;
  clockDate = new Date(clockDate.getTime() + 1000);
  document.getElementById('timeDisplay').textContent = formatClock(clockDate);
}, 1000);

async function refresh() {
  try {
    const r = await fetch('/data');
    const d = await r.json();

    const parsed = parseServerDateTime(d.time, d.date);
    if (parsed) {
      clockDate = parsed;
      document.getElementById('timeDisplay').textContent = formatClock(clockDate);
    } else {
      clockDate = null;
      document.getElementById('timeDisplay').textContent = d.time;
    }
    document.getElementById('dateDisplay').textContent = d.day + ', ' + d.date;

    const wifiOK = d.wifi === 'Connected';
    const wifiPill = document.getElementById('wifiPill');
    wifiPill.className = 'pill ' + (wifiOK ? 'ok' : 'bad');
    wifiPill.querySelector('.d').className = 'd ' + (wifiOK ? 'ok' : 'bad');
    document.getElementById('wifiPillText').textContent = 'WiFi ' + (wifiOK ? 'Connected' : 'Down');
    document.getElementById('ipDisplay').textContent = d.ip;

    const knockOK = d.knock === 'Normal';
    const knockEl = document.getElementById('knockStatus');
    knockEl.textContent = d.knock;
    knockEl.className = 'stat-value ' + (knockOK ? 'ok' : 'warn');
    document.getElementById('knockDot').className = 'pulse-badge' + (knockOK ? '' : ' warn');

    const tiltOK = d.tilt === 'LEVEL';
    const tiltEl = document.getElementById('tiltStatus');
    tiltEl.textContent = d.tilt;
    tiltEl.className = 'stat-value ' + (tiltOK ? 'ok' : 'warn');
    document.getElementById('tiltDot').className = 'pulse-badge' + (tiltOK ? '' : ' warn');

    const mpuOK = d.mpu === 'Normal';
    const mpuEl = document.getElementById('mpuStatus');
    mpuEl.textContent = d.mpu;
    mpuEl.className = 'stat-value ' + (mpuOK ? 'ok' : 'warn');
    document.getElementById('mpuDot').className = 'pulse-badge' + (mpuOK ? '' : ' warn');

    const ledOn = d.led === 'ON';
    const ledEl = document.getElementById('ledStatus');
    ledEl.textContent = d.led;
    ledEl.className = 'stat-value ' + (ledOn ? 'warn' : 'ok');

    document.getElementById('axVal').textContent = d.ax + ' g';
    document.getElementById('ayVal').textContent = d.ay + ' g';
    document.getElementById('azVal').textContent = d.az + ' g';
    document.getElementById('axBar').style.width = accelBarPct(d.ax) + '%';
    document.getElementById('ayBar').style.width = accelBarPct(d.ay) + '%';
    document.getElementById('azBar').style.width = accelBarPct(d.az) + '%';

    const banner = document.getElementById('statusBanner');

    if (d.alertActive) {
      document.getElementById('bannerTitle').textContent = d.alertTitle;
      document.getElementById('bannerAction').textContent = d.alertAction;
      document.getElementById('bannerMeta').textContent = 'Sensor: ' + d.alertSensor + ' · Time: ' + d.alertTime;
      banner.className = 'status-banner active';
      document.getElementById('bannerKicker').textContent = '🚨 ALERT ACTIVE';
      document.getElementById('bannerIcon').textContent = '⚠️';
    } else {
      document.getElementById('bannerTitle').textContent = 'No Alert Detected';
      document.getElementById('bannerAction').textContent = 'All sensors normal — system monitoring';
      document.getElementById('bannerMeta').textContent =
        (d.alertSensor && d.alertSensor !== '--')
          ? 'Last alert: ' + d.alertTitle + ' · ' + d.alertTime
          : 'No alerts recorded yet';
      banner.className = 'status-banner normal';
      document.getElementById('bannerKicker').textContent = '✅ Current Status';
      document.getElementById('bannerIcon').textContent = '✅';
    }
  } catch (e) {
    console.log('refresh failed', e);
  }
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

  // SW-180P: LOW = vibration detected
  bool knockNow = (digitalRead(KNOCK_PIN) == LOW);

  String json = "{";
  json += "\"time\":\"" + (timeOK ? getTimeOnly() : String("N/A")) + "\",";
  json += "\"date\":\"" + (timeOK ? getDateOnly() : String("N/A")) + "\",";
  json += "\"day\":\"" + (timeOK ? getDayOnly() : String("N/A")) + "\",";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\",";
  json += "\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("N/A")) + "\",";
  json += "\"knock\":\"" + String(knockNow ? "VIBRATION" : "Normal") + "\",";
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

void handleOledStatus() {
  triggerOLEDStatus();
  server.send(200, "text/plain", "OK");
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/oledstatus", handleOledStatus);
  server.begin();
  Serial.println("Web dashboard started!");
}

void setup() {
  Serial.begin(115200);
  Serial.println("========================================");
  Serial.println("   COMPLETE ALERT DETECTION SYSTEM");
  Serial.println("   VIBRATION + TILT + MPU6050 + OLED + WEB");
  Serial.println("========================================\n");

  initOLED();

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

  // ===== SW-180P Pin Setup (BEFORE MPU init so pullup is stable) =====
  pinMode(KNOCK_PIN, INPUT_PULLUP);   // SW-180P: stable = HIGH
  pinMode(TILT_PIN, INPUT_PULLUP);    // Tilt reversed logic (LOW = tilted)
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // SW-180P triggers on FALLING edge
  attachInterrupt(digitalPinToInterrupt(KNOCK_PIN), isrKnock, FALLING);

  Serial.println("\nInitializing MPU6050...");
  mpuInitialized = initMPU6050();
  if (mpuInitialized) {
    delay(2000);   // let module settle after calibration
    for (int i = 0; i < 10; i++) {
      readMPU6050();
      delay(50);
    }
    prevAx = ax;
    prevAy = ay;
    prevAz = az;
  }

  bt.begin("Alert_System");
  bt.setTimeout(50);
  Serial.println("Bluetooth Started: Alert_System");

  bt.println("========================================");
  bt.println("   COMPLETE ALERT DETECTION SYSTEM");
  bt.println("========================================");
  bt.println("D12: SW-180P Vibration Sensor");
  bt.println("D5:  Land Tilt (1.5 sec delay) - REVERSED (LOW=Tilt)");
  if (mpuInitialized) {
    bt.println("MPU6050: Sudden Movement");
    bt.println("   Threshold: " + String(MPU_THRESHOLD) + "g");
  } else {
    bt.println("MPU6050: NOT DETECTED!");
  }
  bt.println("Buzzer: Active on alerts (pin " + String(BUZZER_PIN) + ")");
  if (WiFi.status() == WL_CONNECTED) {
    bt.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  }
  bt.println("========================================");
  bt.println("Type 'help' for commands");
  bt.println("========================================");

  Serial.println("\nSystem Ready!");
  Serial.println("D12: SW-180P Vibration Detection");
  Serial.println("D5:  Land Tilt Detection (1.5 sec delay) - REVERSED (LOW=Tilt)");
  if (mpuInitialized) {
    Serial.println("MPU6050: Sudden Movement Detection");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Dashboard: http://" + WiFi.localIP().toString() + "/");
  }
  Serial.println("========================================\n");
  Serial.println("Waiting for alerts...\n");

  oledNormalDisplay();
  lastOledUpdate = millis();
}

void loop() {
  // ===== 1. SW-180P VIBRATION SENSOR (D12) =====
  // Multi-sample verification: real vibration produces multiple pulses
  // within a short window; light taps produce only 1-2 pulses.
  if (millis() - lastKnockCheck >= KNOCK_CHECK_INTERVAL) {
    noInterrupts();
    unsigned int pulses = knockPulseCount;
    knockPulseCount = 0;
    interrupts();

    if (pulses >= KNOCK_MIN_PULSES) {
      digitalWrite(LED_PIN, HIGH);
      if (millis() - lastNotif > KNOCK_ALERT_COOLDOWN) {
        sendAlert(
          "HEAVY VIBRATION DETECTED!",
          "SW-180P Vibration Sensor (D12)",
          "Immediate inspection required",
          "VIBRATION!",
          "Inspect area now!",
          "⚠️",
          "⚡"
        );
        lastNotif = millis();
        lastKnock = millis();
      }
    }
    lastKnockCheck = millis();
  }

  // ===== 2. TILT SENSOR (D5) - REVERSED LOGIC =====
  int currentTiltState = digitalRead(TILT_PIN);
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
          "THE LAND IS TILTED!",
          "Mercury Tilt Sensor (D5)",
          "Check land stability immediately",
          "LAND TILTED",
          "Check stability now!",
          "🌀",
          "🎯"
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
          "No action needed",
          "LEVEL AGAIN",
          "No action needed",
          "✅",
          "✅",
          false
        );
        lastTiltNotif = millis();
      }
    }
    isTilted = false;
    tiltAlertSent = false;
    tiltStartTime = 0;
  }

  // ===== 3. MPU6050 =====
  if (mpuInitialized && (millis() - lastMPURead >= MPU_CHECK_INTERVAL)) {
    readMPU6050();
    if (detectSuddenMovement()) {
      mpuAlertUntil = millis() + MPU_COOLDOWN;
      digitalWrite(LED_PIN, HIGH);
      if (millis() - lastMPUNotif > MPU_COOLDOWN) {
        sendAlert(
          "SUDDEN MOVEMENT DETECTED!",
          "MPU6050 Accelerometer",
          "Check for impact or fall",
          "SUDDEN MOVEMENT",
          "Check for impact!",
          "💥",
          "💥"
        );
        lastMPUNotif = millis();
      }
    }
    mpuAlertSent = (millis() < mpuAlertUntil);
    prevAx = ax;
    prevAy = ay;
    prevAz = az;
    lastMPURead = millis();
  }

  // ===== BUZZER UPDATE =====
  updateBuzzer();

  // ===== LED OFF AFTER TIMEOUT =====
  if (millis() - lastKnock > 300 && !tiltAlertSent && !mpuAlertSent) {
    digitalWrite(LED_PIN, LOW);
  }

  // ===== OLED UPDATE =====
  if (oledShowingAlert) {
    if (millis() - oledAlertStart > OLED_ALERT_DURATION) {
      oledShowingAlert = false;
    } else if (millis() - lastBlink > BLINK_INTERVAL) {
      oledBlinkState = !oledBlinkState;
      lastBlink = millis();
      drawOLEDAlertFrame(oledBlinkState);
    }
  } else if (oledShowingStatus) {
    if (millis() - oledStatusStart > OLED_STATUS_DURATION) {
      oledShowingStatus = false;
    } else if (millis() - lastOledUpdate > OLED_NORMAL_REFRESH) {
      drawOLEDStatusScreen();
      lastOledUpdate = millis();
    }
  } else if (millis() - lastOledUpdate > OLED_NORMAL_REFRESH) {
    oledNormalDisplay();
    lastOledUpdate = millis();
  }

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
      bt.println("D12 (Vibration): " + String(digitalRead(KNOCK_PIN) == LOW ? "VIBRATION" : "Normal"));
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
      bt.println("Bluetooth: Connected");
      bt.println("========================================");
      bt.println("Tilt Logic: REVERSED (LOW = Tilted)");
      bt.println("========================================");
      triggerOLEDStatus();
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
    else if (c == "d4" || c == "D4" || c == "d12" || c == "D12") {
      bt.println("D12 (SW-180P): " + String(digitalRead(KNOCK_PIN) == LOW ? "VIBRATION" : "Normal"));
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
      triggerOLEDStatus();
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
      knockPulseCount = 0;
      lastKnockPulseTime = 0;
      tiltStartTime = 0;
      isTilted = false;
      tiltAlertSent = false;
      mpuAlertSent = false;
      mpuAlertUntil = 0;
      oledShowingAlert = false;
      oledShowingStatus = false;
      digitalWrite(LED_PIN, LOW);
      stopBuzzer();
      bt.println("System Reset Done!");
    }
    else if (c == "help" || c == "HELP" || c == "?") {
      bt.println("========================================");
      bt.println("AVAILABLE COMMANDS");
      bt.println("========================================");
      bt.println("  status     - System status (+ shows on OLED)");
      bt.println("  mpu        - MPU6050 data");
      bt.println("  time       - Show Indian time (IST)");
      bt.println("  d4 / d12   - Check SW-180P vibration sensor");
      bt.println("  d5         - Check Tilt sensor (D5)");
      bt.println("  threshold  - Show MPU threshold");
      bt.println("  wifi       - Show WiFi + dashboard link (+ OLED)");
      bt.println("  led on     - Turn LED ON");
      bt.println("  led off    - Turn LED OFF");
      bt.println("  reset      - Reset system");
      bt.println("  help       - This menu");
      bt.println("========================================");
      bt.println("TILT DELAY: 1.5 seconds");
      bt.println("Tilt Logic: REVERSED (LOW = Tilted)");
      bt.println("Vibration: SW-180P on D12");
      bt.println("MPU THRESHOLD: " + String(MPU_THRESHOLD) + "g");
      bt.println("========================================");
    }
    else if (c != "") {
      bt.println("Unknown command! Type 'help'");
    }
  }

  delay(5);
}
