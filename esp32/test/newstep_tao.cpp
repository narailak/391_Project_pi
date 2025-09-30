// ========= ESP32 + TB6600 + AccelStepper (waspinator ZIP) =========
// วนลูป: +180° -> พัก 5 วินาที -> -180° -> พัก 5 วินาที -> ซ้ำ
// --------------------------------------------------

#include <Arduino.h>
#include <AccelStepper.h>

// ---------- Pin mapping (ปรับตามการต่อจริง) ----------
static const int PIN_PUL = 25;   // TB6600: PUL+ -> GPIO25
static const int PIN_DIR = 26;   // TB6600: DIR+ -> GPIO26
static const int PIN_ENA = 27;   // TB6600: ENA+ -> GPIO27 (ส่วนใหญ่ Active-LOW)
// ขั้วลบ (PUL-, DIR-, ENA-) ต่อ GND

// ---------- Motor / Driver config ----------
static const int  BASE_STEPS_PER_REV = 200;   // 1.8° => 200 steps/rev
static const int  MICROSTEP          = 16;    // ให้ตรงกับ DIP TB6600
static const long STEPS_PER_REV      = (long)BASE_STEPS_PER_REV * MICROSTEP;
static const long HALF_REV_STEPS     = STEPS_PER_REV / 2;   // 180°

static const float MAX_RPS    = 1.5f;   // รอบ/วินาทีสูงสุด (~90 rpm)
static const float ACCEL_RPS2 = 3.0f;   // ความเร่ง (รอบ/วินาที^2)

static const uint32_t PAUSE_MS = 5000;  // หน่วง 5 วินาที

// ---------- AccelStepper (โหมด DRIVER = STEP/DIR) ----------
AccelStepper stepper(AccelStepper::DRIVER, PIN_PUL, PIN_DIR);

// ---------- Helper: Enable/Disable (TB6600 ส่วนใหญ่ Active-LOW) ----------
static inline void driverEnable(bool en) {
  if (PIN_ENA >= 0) {
    pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_ENA, en ? LOW : HIGH);  // LOW = Enable, HIGH = Disable
  }
}

// ---------- State machine ----------
enum Phase { GO_POS_180, PAUSE_AT_POS, GO_NEG_180, PAUSE_AT_NEG };
static Phase phase = GO_POS_180;
static uint32_t t_pause_start = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  driverEnable(true);

  // ตั้งความเร็ว/เร่ง (หน่วย: steps/s และ steps/s^2)
  stepper.setMaxSpeed(MAX_RPS * STEPS_PER_REV);
  stepper.setAcceleration(ACCEL_RPS2 * STEPS_PER_REV);

  // ถ้าทิศทางกลับด้าน ให้ใช้บรรทัดนี้แทนการสลับสาย:
  // stepper.setPinsInverted(false, true, false); // (dirInvert=true)

  stepper.setCurrentPosition(0);            // เริ่มที่ 0°
  stepper.moveTo(+HALF_REV_STEPS);          // ตั้งเป้าหมายแรก +180°
  phase = GO_POS_180;
  Serial.println("[STATE] Go to +180°");
}

void loop() {
  stepper.run(); // ขยับมอเตอร์อย่าง non-blocking ทุกลูป

  switch (phase) {
    case GO_POS_180: {
      if (stepper.distanceToGo() == 0) {
        t_pause_start = millis();
        phase = PAUSE_AT_POS;
        Serial.println("[STATE] Reached +180°, pause...");
      }
    } break;

    case PAUSE_AT_POS: {
      if (millis() - t_pause_start >= PAUSE_MS) {
        stepper.moveTo(-HALF_REV_STEPS); // ไป -180°
        phase = GO_NEG_180;
        Serial.println("[STATE] Go to -180°");
      }
    } break;

    case GO_NEG_180: {
      if (stepper.distanceToGo() == 0) {
        t_pause_start = millis();
        phase = PAUSE_AT_NEG;
        Serial.println("[STATE] Reached -180°, pause...");
      }
    } break;

    case PAUSE_AT_NEG: {
      if (millis() - t_pause_start >= PAUSE_MS) {
        stepper.moveTo(+HALF_REV_STEPS); // วนกลับ +180°
        phase = GO_POS_180;
        Serial.println("[STATE] Go to +180°");
      }
    } break;
  }
}
