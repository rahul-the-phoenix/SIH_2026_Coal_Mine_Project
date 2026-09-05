// KY-031 Knock + Mercury Tilt + MPU6050 - Complete Alert System
// With Indian Time (IST) - Clean Serial + Bluetooth Output (Same Format)

#include "BluetoothSerial.h"
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// WiFi Settings
const char* ssid = "Me";
const char* password = "password";

// Time Settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // IST GMT+5:30

// Pin Definitions
#define KNOCK_PIN 4
#define TILT_PIN 5
#define LED_PIN 2

// MPU6050 Object
MPU6050 mpu(Wire);
const uint8_t MPU_ADDR = 0x68;

// Bluetooth
BluetoothSerial bt;

// ===== KNOCK VARIABLES =====
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
    Serial.println("❌ MPU6050 not detected on I2C!");
    return false;
  }

  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("✅ MPU6050 Initialized!");
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
  status += " | ΔX:" + String(axChange, 2) + ", ΔY:" + String(ayChange, 2) + ", ΔZ:" + String(azChange, 2);
  return status;
}

String getMPUChange() {
  String change = "ΔX:" + String(axChange, 2) + "g, ΔY:" + String(ayChange, 2) + "g, ΔZ:" + String(azChange, 2) + "g";
  return change;
}

// ===== UNIFIED ALERT FUNCTION (Serial + Bluetooth, SAME clean format) =====
// Format:
// <alertType>
// 📍 <sensorName>
// 📅 <Day>, <Date>
// 🕐 <Time>
// <actionLine>
// (blank)
// (blank)
void sendAlert(String alertType, String sensorName, String actionLine) {
  String dateLine, timeLine;
  if (timeOK) {
    dateLine = "📅 " + getDayOnly() + ", " + getDateOnly();
    timeLine = "🕐 " + getTimeOnly();
  } else {
    dateLine = "⏰ Time: Not Synced";
    timeLine = "";
  }

  // ---- Serial Output ----
  Serial.println(alertType);
  Serial.println("📍 " + sensorName);
  Serial.println(dateLine);
  if (timeLine != "") Serial.println(timeLine);
  Serial.println(actionLine);
  Serial.println("");
  Serial.println("");

  // ---- Bluetooth Output (identical format) ----
  bt.println(alertType);
  bt.println("📍 " + sensorName);
  bt.println(dateLine);
  if (timeLine != "") bt.println(timeLine);
  bt.println(actionLine);
  bt.println("");
  bt.println("");
}

void setup() {
  Serial.begin(115200);

  Serial.println("========================================");
  Serial.println("   COMPLETE ALERT DETECTION SYSTEM");
  Serial.println("   KNOCK + TILT + MPU6050");
  Serial.println("========================================\n");

  // ===== WIFI =====
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  for (int i=0; i<15 && WiFi.status()!=WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("📶 IP: ");
    Serial.println(WiFi.localIP());

    configTime(gmtOffset_sec, 0, ntpServer);
    Serial.print("⏰ Syncing Time");
    for (int i=0; i<8; i++) {
      struct tm t;
      if (getLocalTime(&t)) {
        timeOK = true;
        break;
      }
      delay(1000);
      Serial.print(".");
    }
    if (timeOK) {
      Serial.println("\n✅ Time Synced!");
      Serial.println("📅 " + getTime());
    } else {
      Serial.println("\n❌ Time Sync Failed!");
    }
  } else {
    Serial.println("\n❌ WiFi Failed! Time unavailable.");
  }

  // ===== MPU6050 =====
  Serial.println("\n🔧 Initializing MPU6050...");
  mpuInitialized = initMPU6050();
  if (mpuInitialized) {
    readMPU6050();
    prevAx = ax;
    prevAy = ay;
    prevAz = az;
  }

  // ===== BLUETOOTH =====
  bt.begin("Alert_System");
  Serial.println("📱 Bluetooth Started: Alert_System");

  bt.println("========================================");
  bt.println("   COMPLETE ALERT DETECTION SYSTEM");
  bt.println("========================================");
  bt.println("⚠️ D4: Heavy Weight Fall (Knock)");
  bt.println("⚠️ D5: Land Tilt (1.5 sec delay)");
  if (mpuInitialized) {
    bt.println("⚠️ MPU6050: Sudden Movement");
    bt.println("   Threshold: " + String(MPU_THRESHOLD) + "g");
  } else {
    bt.println("❌ MPU6050: NOT DETECTED!");
  }
  bt.println("========================================");
  bt.println("📋 Type 'help' for commands");
  bt.println("========================================");

  // ===== PINS & INTERRUPTS =====
  pinMode(KNOCK_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(KNOCK_PIN, isrKnock, RISING);

  Serial.println("\n✅ System Ready!");
  Serial.println("🔴 D4: Heavy Weight Fall Detection");
  Serial.println("🔴 D5: Land Tilt Detection (1.5 sec delay)");
  if (mpuInitialized) {
    Serial.println("🔴 MPU6050: Sudden Movement Detection");
  }
  Serial.println("========================================\n");
  Serial.println("⏳ Waiting for alerts...\n");
}

void loop() {
  // ===== 1. CHECK KNOCK SENSOR (D4) =====
  if (knock) {
    digitalWrite(LED_PIN, HIGH);
    knock = false;

    if (millis() - lastNotif > 3000) {
      sendAlert(
        "⚠️⚠️⚠️ HEAVY WEIGHT FALL DETECTED! ⚠️⚠️⚠️",
        "KY-031 Knock Module (D4)",
        "⚡ Action: Immediate inspection required"
      );
      lastNotif = millis();
    }
  }

  // ===== 2. CHECK TILT SENSOR (D5) =====
  int currentTiltState = digitalRead(TILT_PIN);

  if (currentTiltState == HIGH) {
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
          "🌋🌋🌋 THE LAND IS TILTED! 🌋🌋🌋",
          "Mercury Tilt Sensor (D5)",
          "🌋 Action: Check land stability immediately"
        );
        lastTiltNotif = millis();
      }
    }
  } else {
    if (isTilted && tiltAlertSent) {
      if (millis() - lastTiltNotif > 3000) {
        sendAlert(
          "✅✅✅ LAND IS LEVEL AGAIN ✅✅✅",
          "Mercury Tilt Sensor (D5)",
          "🟢 Action: No action needed"
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
          "💥💥💥 SUDDEN MOVEMENT DETECTED! 💥💥💥",
          "MPU6050 Accelerometer",
          "💥 Action: Check for impact or fall"
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

  // ===== BLUETOOTH COMMANDS =====
  if (bt.available()) {
    String c = bt.readString();
    c.trim();

    if (c == "status" || c == "STATUS") {
      bt.println("========================================");
      bt.println("📊 SYSTEM STATUS");
      bt.println("========================================");
      bt.println("✅ Status: RUNNING");
      bt.println("📱 Device: ESP32");
      bt.println("========================================");
      bt.println("🔴 D4 (Knock): " + String(digitalRead(KNOCK_PIN) ? "FALL DETECTED" : "Normal"));

      if (tiltAlertSent) {
        bt.println("🔴 D5 (Tilt): 🌋 TILTED (Verified)");
      } else if (isTilted) {
        unsigned long elapsed = millis() - tiltStartTime;
        bt.println("🔴 D5 (Tilt): ⏳ Verifying... " + String(elapsed/1000) + "s / 1.5s");
      } else {
        bt.println("🔴 D5 (Tilt): ✅ LEVEL");
      }

      if (mpuInitialized) {
        bt.println("========================================");
        bt.println("📊 MPU6050 DATA:");
        bt.println("   " + getMPUStatus());
        bt.println("   Threshold: " + String(MPU_THRESHOLD) + "g");
        if (mpuAlertSent) {
          bt.println("   ⚠️ ALERT: Sudden Movement Detected!");
        } else {
          bt.println("   ✅ Status: Normal");
        }
      }

      bt.println("========================================");
      bt.println("💡 LED: " + String(digitalRead(LED_PIN) ? "ON" : "OFF"));
      bt.println("========================================");
      if (timeOK) {
        bt.println("📅 Day: " + getDayOnly());
        bt.println("📆 Date: " + getDateOnly());
        bt.println("🕐 Time: " + getTimeOnly());
      } else {
        bt.println("⏰ Time: NOT SYNCED");
      }
      bt.println("========================================");
      bt.println("📶 WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "No"));
      if (WiFi.status() == WL_CONNECTED) {
        bt.println("📶 IP: " + WiFi.localIP().toString());
      }
      bt.println("========================================");
    }
    else if (c == "mpu" || c == "MPU") {
      if (mpuInitialized) {
        readMPU6050();
        bt.println("========================================");
        bt.println("📊 MPU6050 REAL-TIME DATA");
        bt.println("========================================");
        bt.println("📈 Accelerometer:");
        bt.println("   X: " + String(ax, 2) + "g");
        bt.println("   Y: " + String(ay, 2) + "g");
        bt.println("   Z: " + String(az, 2) + "g");
        bt.println("========================================");
        bt.println("🔄 Gyroscope:");
        bt.println("   X: " + String(gx, 2) + "°/s");
        bt.println("   Y: " + String(gy, 2) + "°/s");
        bt.println("   Z: " + String(gz, 2) + "°/s");
        bt.println("========================================");
        bt.println("📊 Change: " + getMPUChange());
        bt.println("🎯 Threshold: " + String(MPU_THRESHOLD) + "g");
        if (detectSuddenMovement()) {
          bt.println("⚠️ SUDDEN MOVEMENT DETECTED!");
        } else {
          bt.println("✅ Normal");
        }
        bt.println("========================================");
      } else {
        bt.println("❌ MPU6050 Not Detected!");
      }
    }
    else if (c == "time" || c == "TIME") {
      if (timeOK) {
        bt.println("========================================");
        bt.println("⏰ CURRENT INDIAN TIME (IST)");
        bt.println("========================================");
        bt.println("📅 Day: " + getDayOnly());
        bt.println("📆 Date: " + getDateOnly());
        bt.println("🕐 Time: " + getTimeOnly());
        bt.println("📅 Full: " + getTime());
        bt.println("🌐 Time Zone: GMT+5:30 (IST)");
        bt.println("========================================");
      } else {
        bt.println("❌ Time Not Synced!");
      }
    }
    else if (c == "d4" || c == "D4") {
      bt.println("D4 (Knock): " + String(digitalRead(KNOCK_PIN) ? "⚠️ FALL DETECTED" : "Normal"));
    }
    else if (c == "d5" || c == "D5") {
      if (tiltAlertSent) {
        bt.println("D5 (Tilt): 🌋 TILTED (Verified for 1.5s)");
      } else if (isTilted) {
        unsigned long elapsed = millis() - tiltStartTime;
        bt.println("D5 (Tilt): ⏳ Verifying... " + String(elapsed/1000) + "s / 1.5s");
      } else {
        bt.println("D5 (Tilt): ✅ LEVEL");
      }
    }
    else if (c == "threshold" || c == "THRESHOLD") {
      bt.println("📊 MPU6050 Threshold: " + String(MPU_THRESHOLD) + "g");
      bt.println("💡 Lower = More Sensitive");
      bt.println("💡 Higher = Less Sensitive");
    }
    else if (c == "led on") {
      digitalWrite(LED_PIN, HIGH);
      bt.println("💡 LED Turned ON");
    }
    else if (c == "led off") {
      digitalWrite(LED_PIN, LOW);
      bt.println("💡 LED Turned OFF");
    }
    else if (c == "reset" || c == "RESET") {
      lastKnock=0;
      knock=false;
      tiltStartTime=0;
      isTilted=false;
      tiltAlertSent=false;
      mpuAlertSent=false;
      digitalWrite(LED_PIN, LOW);
      bt.println("🔄 System Reset Done!");
    }
    else if (c == "help" || c == "HELP" || c == "?") {
      bt.println("========================================");
      bt.println("📋 AVAILABLE COMMANDS");
      bt.println("========================================");
      bt.println("  status     - System status");
      bt.println("  mpu        - MPU6050 data");
      bt.println("  time       - Show Indian time (IST)");
      bt.println("  d4         - Check Knock sensor (D4)");
      bt.println("  d5         - Check Tilt sensor (D5)");
      bt.println("  threshold  - Show MPU threshold");
      bt.println("  led on     - Turn LED ON");
      bt.println("  led off    - Turn LED OFF");
      bt.println("  reset      - Reset system");
      bt.println("  help       - This menu");
      bt.println("========================================");
      bt.println("⏱️ TILT DELAY: 1.5 seconds");
      bt.println("🎯 MPU THRESHOLD: " + String(MPU_THRESHOLD) + "g");
      bt.println("========================================");
    }
    else if (c != "") {
      bt.println("❌ Unknown command! Type 'help'");
    }
  }

  delay(5);
}
