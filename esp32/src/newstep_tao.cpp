// ================= ESP32 + micro-ROS Stepper (TB6600)
// Absolute Angle via /man/cmd_tao (std_msgs/Int16, deg 0..359)
// - ไปถึงเป้าหมายแล้ว "หยุดนิ่ง" ไม่เดินต่อเอง
// - Debounce คำสั่ง, Deadband กัน jitter, กันคำสั่งซ้ำ
// - ส่งฟีดแบ็กมุมปัจจุบัน Int16 ที่ /man/cmd_tao/fb ทุก ~100 ms
// ----------------------------------------------------------------

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include "soc/gpio_reg.h"

// ---------------- Pins (ปรับตามการต่อจริง) ----------------
static const int  PIN_PUL = 25;   // TB6600: PUL+ -> GPIO25,  PUL- -> GND
static const int  PIN_DIR = 26;   // TB6600: DIR+ -> GPIO26,  DIR- -> GND
static const int  PIN_ENA = 27;   // TB6600: ENA+ -> GPIO27, ENA- -> GND (ส่วนใหญ่ Active-LOW)
static const bool ENA_ACTIVE_LOW = true; // TB6600 ทั่วไป LOW=Enable, HIGH=Disable

// ---------------- Stepper configuration ----------------
static const int   BASE_STEPS_PER_REV = 200;  // 1.8° => 200 steps/rev
static const int   MICROSTEP          = 16;   // ให้ตรงกับ DIP TB6600
static const float GEAR_RATIO         = 1.0f;

static const long  STEPS_PER_REV = (long)(BASE_STEPS_PER_REV * MICROSTEP * GEAR_RATIO);

// ---------------- Speed (เริ่มแบบปลอดภัย) ----------------
// ความถี่พัลส์ ~ 1/(2*HALF_PERIOD_US)
// 100us ~ 5 kHz (แนะนำเริ่มช้า ๆ เพื่อตัดปัญหาหลุดสเต็ปจากเร่งทันที)
static volatile uint32_t HALF_PERIOD_US = 100;

// ---------------- Behavior switches ----------------
static const bool HOLD_TORQUE = true;  // true = ค้างแรงบิดหลังถึงเป้า, false = ปล่อย ENA

// ---------------- Input filtering ----------------
static const uint32_t CMD_DEBOUNCE_MS           = 120;  // ไม่รับคำสั่งถี่กว่า X ms
static const int      CMD_MIN_DELTA_DEG         = 2;    // ค่าที่ต่างจากอันก่อนหน้า < นี้ = เมิน (กัน jitter)
static const int      CMD_DEADBAND_TO_TARGET_DEG= 1;    // ใกล้เป้าหมายเดิม <= นี้ = ถือว่าเท่าเดิม

// ---------------- micro-ROS boilerplate ----------------
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

rcl_subscription_t   sub_cmd;       // /man/cmd_tao
std_msgs__msg__Int16 cmd_msg;

rcl_publisher_t      pub_fb;        // /man/cmd_tao/fb
std_msgs__msg__Int16 fb_msg;

rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state = WAITING_AGENT;

// ---------------- Run/Position state ----------------
static volatile int8_t  RUN_DIR = 0;                 // +1=CW, -1=CCW, 0=STOP
static volatile bool    pul_high = false;
static volatile long    steps_remaining = 0;         // เท่ากับ 0 = หยุด
static volatile long    current_steps   = 0;         // ตำแหน่งปัจจุบัน (step, อ้าง 0=0°)
static volatile long    target_steps_abs = LONG_MIN; // เป้าหมายล่าสุด (abs step)

// สำหรับ debounce
static uint32_t         last_cmd_ms = 0;
static int              last_cmd_deg_processed = -999; // เก็บมุมที่ "รับไปประมวลผลแล้ว"

hw_timer_t* tmr = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Helpers ----------------
static inline void enable_driver(bool enable)
{
  if (ENA_ACTIVE_LOW) digitalWrite(PIN_ENA, enable ? LOW : HIGH);
  else                digitalWrite(PIN_ENA, enable ? HIGH : LOW);
}

static inline void set_direction(int8_t dir)
{
  // ถ้าทิศกลับด้าน ให้สลับ HIGH/LOW ตรงนี้ หรือสลับสายคอยล์ชุดใดชุดหนึ่ง
  digitalWrite(PIN_DIR, (dir == 1) ? LOW : HIGH);
}

static inline int normalize_deg(int d)
{
  int x = d % 360; if (x < 0) x += 360;
  return x;
}

static inline int16_t angle_int_from_steps(long s)
{
  long steps = s % STEPS_PER_REV;
  if (steps < 0) steps += STEPS_PER_REV;
  // ใช้ integer math: floor((deg*steps)/360) แบบกลับกัน
  // deg = round(steps*360/SPR) ก็ได้ แต่เพื่อความนิ่งใช้ปัดใกล้เคียง
  float degf = ( (float)steps * 360.0f ) / (float)STEPS_PER_REV;
  int16_t a = (int16_t)lroundf(degf);
  if (a >= 360) a -= 360;
  if (a < 0)    a += 360;
  return a;
}

static inline void publish_angle_now()
{
  fb_msg.data = angle_int_from_steps(current_steps);
  RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
}

// แปลงองศา -> สเต็ป (ใช้ integer math เพื่อตัด round แล้วเด้งรอบ)
static inline long steps_from_degrees_int(int deg)
{
  // floor((deg * STEPS_PER_REV) / 360.0) โดยใช้เลขเต็ม:
  // เพิ่ม 0.5 เพื่อเอียงไปทางใกล้สุดก็ได้ แต่เลือก floor เพื่อตัดปัญหา wrap ใกล้ 360
  deg = normalize_deg(deg);
  long num = (long)deg * (long)STEPS_PER_REV;
  long s   = num / 360;        // floor
  if (s >= STEPS_PER_REV) s -= STEPS_PER_REV;
  if (s < 0)              s += STEPS_PER_REV;
  return s;
}

static inline long shortest_delta(long current, long target_mod)
{
  long cur_mod = current % STEPS_PER_REV;
  if (cur_mod < 0) cur_mod += STEPS_PER_REV;

  long diff = target_mod - cur_mod;  // (-SPR, +SPR)
  if (diff >  (STEPS_PER_REV/2)) diff -= STEPS_PER_REV;
  if (diff < -(STEPS_PER_REV/2)) diff += STEPS_PER_REV;
  return diff; // [-half, +half]
}

// อ่าน current_steps แบบป้องกัน race
static inline long get_current_steps_atomic()
{
  portENTER_CRITICAL_ISR(&spinlock);
  long s = current_steps;
  portEXIT_CRITICAL_ISR(&spinlock);
  return s;
}

// สั่งให้ขยับ delta_steps (relative) ด้วย ISR
static inline void start_move_steps(long delta_steps)
{
  if (delta_steps == 0) {
    portENTER_CRITICAL_ISR(&spinlock);
    RUN_DIR = 0;
    steps_remaining = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
    if (!HOLD_TORQUE) enable_driver(false);
    return;
  }

  int8_t dir = (delta_steps > 0) ? 1 : -1; // delta > 0 => CW
  long   todo = labs(delta_steps);

  set_direction(dir);
  enable_driver(true);

  portENTER_CRITICAL_ISR(&spinlock);
  RUN_DIR = dir;
  steps_remaining = todo;
  portEXIT_CRITICAL_ISR(&spinlock);
}

// ระยะเชิงมุมแบบวงกลม (0..180)
static inline int circular_abs_deg_diff(int a, int b)
{
  int da = abs(normalize_deg(a) - normalize_deg(b));
  return (da > 180) ? (360 - da) : da;
}

// ไปที่องศา absolute (0..359) แบบเลือกทางสั้นสุด + Debounce + Deadband + กันสั่งซ้ำ
static void go_to_degree_filtered(int deg_in)
{
  const uint32_t now = millis();
  const int      d   = normalize_deg(deg_in);

  // 1) Debounce เวลา
  if (now - last_cmd_ms < CMD_DEBOUNCE_MS) {
    return;
  }

  // 2) เมินค่าเป้าหมายที่ใกล้กับอันก่อนหน้ามาก ๆ (กัน jitter input)
  if (last_cmd_deg_processed != -999) {
    if (circular_abs_deg_diff(d, last_cmd_deg_processed) < CMD_MIN_DELTA_DEG) {
      // ยังนับว่าเป็นค่าเดิม
      return;
    }
  }

  // 3) ถ้าใกล้ "เป้าหมายที่ตั้งไว้แล้ว" มาก ๆ ให้ snap เป็นค่าเดิม (กันเด้งไปมารอบ ๆ เป้า)
  if (target_steps_abs != LONG_MIN) {
    int target_deg_now = angle_int_from_steps(target_steps_abs);
    if (circular_abs_deg_diff(d, target_deg_now) <= CMD_DEADBAND_TO_TARGET_DEG) {
      // ถือว่ายังสั่งมุมเดิม
      last_cmd_deg_processed = target_deg_now;
      last_cmd_ms = now;
      return;
    }
  }

  // 4) คำนวณ delta และสั่งเดิน
  long target_abs = steps_from_degrees_int(d);
  long cur        = get_current_steps_atomic();

  // ถ้ากำลังไปเป้านี้อยู่และยังไม่ถึง -> เมินคำสั่งซ้ำ
  if (target_steps_abs == target_abs && steps_remaining > 0) {
    last_cmd_deg_processed = d;
    last_cmd_ms = now;
    return;
  }

  long delta_now = shortest_delta(cur, target_abs);
  if (delta_now == 0) {
    target_steps_abs = target_abs;
    last_cmd_deg_processed = d;
    last_cmd_ms = now;
    return;
  }

  target_steps_abs = target_abs;
  start_move_steps(delta_now);

  last_cmd_deg_processed = d;
  last_cmd_ms = now;
}

// ---------------- ISR (ฮาร์ดแวร์ไทเมอร์) ----------------
void IRAM_ATTR onTimer()
{
  // ไม่มีงาน = พัลส์ LOW
  if (RUN_DIR == 0 || steps_remaining <= 0) {
    if (pul_high) {
      REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL));
      pul_high = false;
    }
    return;
  }

  // Toggle พิน PUL ด้วยเรจิสเตอร์ (ไวกว่า digitalWrite)
  uint32_t mask = (1U << PIN_PUL);
  if (!pul_high) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask); // HIGH
    pul_high = true;
  } else {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask); // LOW (นับสเต็ปที่ขอบนี้)
    pul_high = false;

    // อัปเดตตัวนับ
    steps_remaining--;
    if (RUN_DIR == 1)      current_steps++;
    else if (RUN_DIR == -1) current_steps--;

    // ถึงเป้าหมายแล้ว: หยุดนิ่ง
    if (steps_remaining <= 0) {
      RUN_DIR = 0;
      // snap ให้ตรงเป้าหมาย (ลด error จากการปัดเศษ)
      if (target_steps_abs != LONG_MIN) current_steps = target_steps_abs;
      if (!HOLD_TORQUE) enable_driver(false);
    }
  }

  // เผื่อมีการปรับ HALF_PERIOD_US runtime
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
}

// ---------------- Subscriber callback ----------------
static void cmd_cb(const void* msgin)
{
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  const int target_deg = m->data;  // คาด 0..359

  go_to_degree_filtered(target_deg);

  // ส่ง fb หนึ่งครั้งเมื่อรับคำสั่ง (หลังฟิลเตอร์แล้ว)
  publish_angle_now();

  Serial.print("[CMD] in=");
  Serial.print(target_deg);
  Serial.print(" -> proc=");
  Serial.print(last_cmd_deg_processed);
  Serial.print(" | cur=");
  Serial.print((int)angle_int_from_steps(current_steps));
  Serial.print(" deg, remain=");
  Serial.println((int)steps_remaining);
}

// ---------------- micro-ROS entities ----------------
static bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  // ROS_DOMAIN_ID = 96

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_stepper_abs_node", "", &support));

  // Subscriber: /man/cmd_tao (std_msgs/Int16)
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_tao"));

  // Publisher (best-effort): /man/cmd_tao/fb (std_msgs/Int16)
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_tao/fb"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_cb, ON_NEW_DATA));

  return true;
}

static bool destroyEntities()
{
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd, &node);
  rcl_publisher_fini(&pub_fb, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

// ---------------- Arduino setup/loop ----------------
void setup()
{
  // GPIO
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_PUL, LOW);
  digitalWrite(PIN_DIR, LOW);
  enable_driver(false); // ปิดไว้ก่อน

  // micro-ROS transport via Serial
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);
  Serial.println("[INFO] ESP32 Stepper (Absolute Angle) Ready");

  // Timer: 80 MHz / 80 = 1 MHz => 1 tick = 1 µs
  tmr = timerBegin(0, 80, true);
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, HALF_PERIOD_US, true); // ครึ่งคาบเริ่มต้น
  timerAlarmEnable(tmr);                       // ยิง ISR ตลอด

  // เริ่มประกาศมุม 0°
  publish_angle_now();
}

void loop()
{
  static uint32_t t_led = 0;
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if(state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1500,
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state == AGENT_CONNECTED){
        // non-blocking เพื่อให้ ISR ทำงานลื่น
        rclc_executor_spin_some(&executor, 0);

        // ส่งเฟีดแบ็กมุมเป็นระยะ (100 ms)
        EXECUTE_EVERY_N_MS(100, publish_angle_now(););
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      // เพื่อความปลอดภัย: หยุดมอเตอร์เมื่อหลุด agent
      portENTER_CRITICAL_ISR(&spinlock);
      RUN_DIR = 0;
      steps_remaining = 0;
      portEXIT_CRITICAL_ISR(&spinlock);
      enable_driver(false);
      Serial.println("[WARN] Agent disconnected -> STOP");
      state = WAITING_AGENT;
      break;
  }
}
