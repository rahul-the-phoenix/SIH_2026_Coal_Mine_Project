// KY-031 Knock/Vibration Sensor with ESP32 built-in LED
// KY-031 Signal (S) pin -> D4 (GPIO4)
// Built-in LED -> GPIO2 (most ESP32 dev boards)

#define KNOCK_PIN 4     // D4
#define LED_PIN   2     // Built-in LED

volatile bool knockDetected = false;
unsigned long lastKnockTime = 0;
const unsigned long ledHoldTime = 300;   // keep LED on for 300ms after knock
const unsigned long debounceTime = 100;  // ignore repeat triggers within 100ms

void IRAM_ATTR knockISR() {
  unsigned long now = millis();
  if (now - lastKnockTime > debounceTime) {
    knockDetected = true;
    lastKnockTime = now;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(KNOCK_PIN, INPUT);   // KY-031 has its own pull-down/output driver
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(KNOCK_PIN), knockISR, RISING);
}

void loop() {
  if (knockDetected) {
    Serial.println("Knock/Vibration Detected!");
    digitalWrite(LED_PIN, HIGH);
    knockDetected = false;
    
  }

  // turn LED off after hold time if no new knock
  if (millis() - lastKnockTime > ledHoldTime) {
    digitalWrite(LED_PIN, LOW);
  }
}
