// ================= ESP32 + micro-ROS Stepper (TB6600) — Timer ISR + Dual Limit Switch =================
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// ---------------- Pins ----------------
static const int PIN_PUL = 25;   // Pulse
static const int PIN_DIR = 26;   // Direction
static const int PIN_ENA = 27;   // Enable (TB6600: Active-Low)

// ---- Limit switches (internal pull-up, active LOW when pressed) ----
static const int LIMIT1_PIN = 18; // ด้านซ้าย / ทิศ -1 (CCW)
static const int LIMIT2_PIN = 13; // ด้านขวา / ทิศ +1 (CW)

// ---------------- Speed (default) ----------------
// ความถี่พัลส์ (ทั้งคาบ) = 1 / (2 * HALF_PERIOD_US)
// HALF_PERIOD_US = 100 -> ~5 kHz
static volatile uint32_t HALF_PERIOD_US = 20;

// ---------------- micro-ROS boilerplate ----------------
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

rcl_subscription_t sub_cmd;
std_msgs__msg__Int16   cmd_msg;

rcl_publisher_t  pub_fb;
std_msgs__msg__Int16   fb_msg;

rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state = WAITING_AGENT;

// ---------------- Run state ----------------
static volatile int8_t RUN_DIR = 0;     // +1=CW, -1=CCW, 0=STOP
static volatile bool   pul_high = false;

hw_timer_t* tmr = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---- Limit flags (ใช้เคลียร์/แจ้งหลัง ISR) ----
static volatile bool   limit_hit_flag = false; // มีชนลิมิต
static volatile int8_t limit_side     = 0;     // -1 ชน LIMIT1, +1 ชน LIMIT2

// ---------------- Utils ----------------
static inline bool limit1_pressed() { return digitalRead(LIMIT1_PIN) == LOW; } // ซ้าย/ทิศ -1
static inline bool limit2_pressed() { return digitalRead(LIMIT2_PIN) == LOW; } // ขวา/ทิศ +1

// ---------------- ISR (ฮาร์ดแวร์ไทเมอร์) ----------------
void IRAM_ATTR onTimer()
{
  // ถ้าสั่งหยุด: ดึงพัลส์ลง LOW แล้วไม่ทำอะไรต่อ
  if (RUN_DIR == 0) {
    if (pul_high) { REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); pul_high = false; }
    return;
  }

  // ป้องกันวิ่งต่อเมื่อชนลิมิต (เช็คเร็วใน ISR)
  if ((RUN_DIR == +1 && gpio_get_level((gpio_num_t)LIMIT2_PIN) == 0) ||
      (RUN_DIR == -1 && gpio_get_level((gpio_num_t)LIMIT1_PIN) == 0)) {
    // หยุดเฉพาะในส่วนที่ปลอดภัยใน ISR: ปล่อยให้ loop ไปปิด ENA
    portENTER_CRITICAL_ISR(&spinlock);
      RUN_DIR = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL));
    pul_high = false;
    limit_hit_flag = true;
    limit_side = (gpio_get_level((gpio_num_t)LIMIT2_PIN) == 0) ? +1 : -1;
    return;
  }

  // Toggle พิน PUL ด้วย register (เร็วกว่า digitalWrite)
  uint32_t mask = (1U << PIN_PUL);
  if (!pul_high) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask); // HIGH
    pul_high = true;
  } else {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask); // LOW
    pul_high = false;
  }

  // ปรับคาบ (ถ้าคุณจะเปลี่ยน HALF_PERIOD_US ขณะวิ่ง)
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
}

// ---------------- Helpers ----------------
static inline void enable_driver(bool en_low_active = true)
{
  // Active-Low: LOW=Enable, HIGH=Disable
  digitalWrite(PIN_ENA, en_low_active ? LOW : HIGH);
}

static inline void set_direction(int8_t dir)
{
  // ปรับ LOW/HIGH ให้ตรงกับการต่อจริง
  digitalWrite(PIN_DIR, (dir == 1) ? LOW : HIGH);
}

static inline bool can_run_dir(int8_t dir)
{
  // ไม่อนุญาตให้ "ดันทิศเข้าลิมิต" หากสวิตช์ยังถูกกดอยู่
  if (dir == +1 && limit2_pressed()) return false; // จะไปทาง + แต่ชนขวาอยู่
  if (dir == -1 && limit1_pressed()) return false; // จะไปทาง - แต่ชนซ้ายอยู่
  return true;
}

static inline void start_run(int8_t dir)
{
  if (dir != 1 && dir != -1) return;

  // ถ้ากำลังทับลิมิตฝั่งนั้นอยู่ ไม่ให้วิ่งต่อทางเดิม
  if (!can_run_dir(dir)) {
    // แจ้งกลับ (เช่น ส่ง 2 หรือ -2 เป็นรหัสชนลิมิต)
    fb_msg.data = (dir == +1) ? +2 : -2;
    RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
    return;
  }

  // ตั้งทิศ & เปิดไดร์เวอร์
  set_direction(dir);
  enable_driver(true);

  // ให้ ISR รู้ว่าเริ่มหมุน
  portENTER_CRITICAL_ISR(&spinlock);
    RUN_DIR = dir;
  portEXIT_CRITICAL_ISR(&spinlock);
}

static inline void stop_now_core() // เรียกจาก loop เท่านั้น (ไม่ใช่ ISR)
{
  // แจ้ง ISR ให้หยุด toggle
  portENTER_CRITICAL_ISR(&spinlock);
    RUN_DIR = 0;
  portEXIT_CRITICAL_ISR(&spinlock);

  // ดึงพัลส์ลง LOW ทันที
  REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL));
  pul_high = false;

  // ปลด ENA (ลดความร้อน/กินไฟ) — ถ้าอยาก "ค้างแรงบิด" ให้คอมเมนต์บรรทัดนี้
  enable_driver(false);
}

static inline void stop_now_from_cmd() // ใช้ตอนรับคำสั่ง 0
{
  stop_now_core();
}

// ---------------- Feedback ----------------
static inline void publish_echo(int16_t v)
{
  fb_msg.data = v;
  RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
}

// ---- Limit ISRs: ทำแค่ set flag/หยุดปลอดภัยใน ISR ----
void IRAM_ATTR onLimit1()
{
  if (gpio_get_level((gpio_num_t)LIMIT1_PIN) == 0) {
    portENTER_CRITICAL_ISR(&spinlock);
      RUN_DIR = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL));
    pul_high = false;
    limit_hit_flag = true;
    limit_side = -1;
  }
}

void IRAM_ATTR onLimit2()
{
  if (gpio_get_level((gpio_num_t)LIMIT2_PIN) == 0) {
    portENTER_CRITICAL_ISR(&spinlock);
      RUN_DIR = 0;
    portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL));
    pul_high = false;
    limit_hit_flag = true;
    limit_side = +1;
  }
}

// ---------------- Subscriber callback ----------------
static void cmd_cb(const void* msgin)
{
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  const int16_t cmd = m->data;    // -1, 0, 1 จากจอย

  publish_echo(cmd);

  if (cmd == 1) {
    // *** เปลี่ยนแปลง: ไม่หยุดเมื่อได้คำสั่ง 1 หรือ -1 ***
    // เพียงแค่เริ่มหมุน และจะหมุนต่อไปจนกว่าจะได้คำสั่ง 0 หรือชน limit switch
    start_run(-1);  // โค้ดเดิม: cmd=1 -> หมุน CCW
    Serial.println("[CMD] CW (continuous run until STOP or LIMIT)");
  } else if (cmd == -1) {
    start_run(+1);  // โค้ดเดิม: cmd=-1 -> หมุน CW  
    Serial.println("[CMD] CCW (continuous run until STOP or LIMIT)");
  } else if (cmd == 0) {
    // *** เฉพาะคำสั่ง 0 เท่านั้นที่จะหยุดมอเตอร์ ***
    stop_now_from_cmd();
    Serial.println("[CMD] STOP");
  } else {
    // ค่าอื่น: เมิน
  }
}

// ---------------- micro-ROS entities ----------------
static bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  // ROS_DOMAIN_ID = 96

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_stepper_node", "", &support));

  // Subscriber: /man/cmd_linear (std_msgs/Int16)
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_linear"));

  // Publisher (best-effort): /man/cmd_linear/fb (std_msgs/Int16)
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_linear/fb"));

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

  // Limit switches (internal pull-up)
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT1_PIN), onLimit1, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT2_PIN), onLimit2, FALLING);

  // micro-ROS transport via Serial
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);
  Serial.println("[INFO] ESP32 Stepper (Continuous Run Mode) Ready");

  // Timer: 80 MHz / 80 = 1 MHz => 1 tick = 1 µs
  tmr = timerBegin(0, 80, true);
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, HALF_PERIOD_US, true); // ครึ่งคาบเริ่มต้น
  timerAlarmEnable(tmr);                       // ยิง ISR ตลอด (ISR จะดู RUN_DIR ว่าจะ toggle ไหม)
}

void loop()
{
  // จัดการสถานะ micro-ROS/agent
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
        // ไม่บล็อก เพื่อความไวของ ISR
        rclc_executor_spin_some(&executor, 0);
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      stop_now_core();   // ปลอดภัย: หยุดมอเตอร์เมื่อหลุด agent
      state = WAITING_AGENT;
      break;
  }

  // ---- Post-ISR handling: เคลียร์หลังชนลิมิต ----
  if (limit_hit_flag) {
    limit_hit_flag = false;

    // ปิดไดรเวอร์/ปล่อยแรงบิด และแจ้งฝั่ง ROS ว่าชนลิมิต (รหัส +2 / -2)
    stop_now_core();
    fb_msg.data = (limit_side >= 0) ? +2 : -2; // +2 = ชน LIMIT2 (ขวา), -2 = ชน LIMIT1 (ซ้าย)
    RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));

    Serial.printf("[LIMIT] Hit %s -> STOP\n", (limit_side >= 0) ? "RIGHT(+1)" : "LEFT(-1)");
  }
}