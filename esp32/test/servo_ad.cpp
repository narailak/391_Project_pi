#include <ESP32Servo.h>   // ถ้า Arduino UNO ใช้ <Servo.h>
#include <Arduino.h>

Servo myservo;
int servoPin = 18;

void setup() {
  // myservo.attach(servoPin, 500, 2430);
  myservo.attach(servoPin, 400, 2400);
  // myservo.attach(servoPin, 527, 2430);
}

void loop() {
  myservo.write(0);     // ไปที่ 0°
  delay(5000);          // รอ 5 วินาที
  myservo.write(180);   // ไปที่ 180°
  delay(5000);          // รอ 5 วินาที
}