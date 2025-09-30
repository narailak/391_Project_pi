#include <Arduino.h>

// ====== Pin Mapping (ESP32) ======
const int PUL = 25; // Pulse
const int DIR = 26; // Direction
const int ENA = 27; // Enable (TB6600 ส่วนใหญ่ Active-Low)

// ปรับเวลาหน่วงตามความเร็วที่ต้องการ (หน่วย: ไมโครวินาที)
const int PULSE_HIGH_US = 50;   // เวลากด HIGH
const int PULSE_LOW_US  = 50;   // เวลากด LOW

void step(bool forward, long steps, int us_high, int us_low) {
  digitalWrite(DIR, forward ? LOW : HIGH);  // LOW=เดินหน้า, HIGH=ถอยหลัง (แล้วแต่การต่อ)
  digitalWrite(ENA, LOW);                   // Active-Low: LOW=Enable, HIGH=Disable

  for (long i = 0; i < steps; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(us_high);
    digitalWrite(PUL, LOW);
    delayMicroseconds(us_low);
  }
}

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  // ตั้งค่าเริ่มต้น
  digitalWrite(PUL, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(ENA, HIGH); // ปิดไว้ก่อน (Active-Low)

  Serial.begin(115200);
  delay(500);
  Serial.println("[INFO] Stepper ready.");
}

void loop() {
  // เดินหน้า 6400 สเต็ป
  Serial.println("[INFO] Forward 6400 steps");
  step(true, 3200, PULSE_HIGH_US, PULSE_LOW_US);
  delay(3000);

  // ถอยหลัง 6400 สเต็ป
  Serial.println("[INFO] Backward 6400 steps");
  step(false, 3200, PULSE_HIGH_US, PULSE_LOW_US);
  delay(500);
}
