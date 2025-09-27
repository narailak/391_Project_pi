// ================= ESP32 + micro-ROS Stepper (TB6600) — Absolute Angle via /man/cmd_tao =================
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// ---------------- Pins (ปรับตามการต่อจริง) ----------------
static const int PIN_PUL = 33;   // Pulse
static const int PIN_DIR = 32;   // Direction
static const int PIN_ENA = 14;   // Enable (TB6600 ส่วนใหญ่ Active-Low)

// ---------------- Stepper configuration ----------------
// มอเตอร์ 1.8° => 200 steps/rev (เต็มสเต็ป)
static const int BASE_STEPS_PER_REV = 400;   // ปกติ 200
static const int MICROSTEP          = 16;    // ให้ตรงกับ DIP TB6600: 1,2,4,8,16,32
static const float GEAR_RATIO       = 1.0f;  // ถ้ามีทดเกียร์ ตั้งค่าที่นี่ (ไม่มีให้ 1.0)

static const long STEPS_PER_REV = (long)(BASE_STEPS_PER_REV * MICROSTEP * GEAR_RATIO);

// ---------------- Speed (default) ----------------
// ความถี่พัลส์ ~ 1/(2*HALF_PERIOD_US) [MHz→us], HALF_PERIOD_US ยิ่งน้อยยิ่งเร็ว
static volatile uint32_t HALF_PERIOD_US = 40;  // ครึ่งคาบเริ่มต้น (20us ≈ 25kHz, 40us ≈ 12.5kHz)

// ---------------- micro-ROS boilerplate ----------------
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

rcl_subscription_t sub_cmd;
std_msgs__msg__Int16   cmd_msg;

rcl_publisher_t  pub_fb;            // /man/cmd_tao/fb = มุมปัจจุบัน (Int16)
std_msgs__msg__Int16   fb_msg;

rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state = WAITING_AGENT;

// ---------------- Run/Position state ----------------
static volatile int8_t  RUN_DIR = 0;                // +1=CW, -1=CCW, 0=STOP
static volatile bool    pul_high = false;
static volatile long    steps_remaining = 0;        // นับถอยหลังจนเป็น 0 จึงหยุด
static volatile long    current_steps   = 0;        // ตำแหน่งปัจจุบัน (หน่วย: step)
                                                    // อ้าง 0 step = 0°

hw_timer_t* tmr = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Helpers ----------------
static inline void enable_driver(bool en_low_active = true)
{
  // Active-Low: LOW=Enable, HIGH=Disable
  digitalWrite(PIN_ENA, en_low_active ? LOW : HIGH);
}

static inline void set_direction(int8_t dir)
{
  // NOTE: LOW/HIGH ขึ้นกับการต่อจริง ถ้าหมุนกลับด้านให้สลับที่นี่
  digitalWrite(PIN_DIR, (dir == 1) ? LOW : HIGH);
}

static inline int16_t angle_int_from_steps(long s)
{
  // ปัดมุมเป็นจำนวนเต็ม 0..359
  // fmod อาจแพง/ไม่มีในบางสภาพแวดล้อม จัดการเองแบบ integer
  long steps = s % STEPS_PER_REV;
  if (steps < 0) steps += STEPS_PER_REV;
  float deg = ( (float)steps * 360.0f ) / (float)STEPS_PER_REV;
  // ปัดและห่อ 0..359
  int16_t a = (int16_t)lroundf(deg);
  a %= 360;
  if (a < 0) a += 360;
  return a;
}

static inline void publish_angle_now()
{
  fb_msg.data = angle_int_from_steps(current_steps);
  RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
}

// ---------------- ISR (ฮาร์ดแวร์ไทเมอร์) ----------------
void IRAM_ATTR onTimer()
{
  // ไม่มีงาน = ไม่ toggle
  if (RUN_DIR == 0 || steps_remaining <= 0) {
    // ดึงพัลส์ลง LOW
    if (pul_high) { REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); pul_high = false; }
    return;
  }

  // Toggle พิน PUL แบบ register (เร็วกว่า digitalWrite)
  uint32_t mask = (1U << PIN_PUL);
  if (!pul_high) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask); // HIGH
    pul_high = true;
  } else {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask); // LOW
    pul_high = false;

    // นับ step เมื่อครบ 1 คาบ (ที่ขอบใดขอบหนึ่ง)
    // ลดจำนวนที่เหลือ และอัปเดตตำแหน่งจริง
    steps_remaining--;
    if (RUN_DIR == 1) {          // CW
      current_steps++;
    } else if (RUN_DIR == -1) {  // CCW
      current_steps--;
    }

    if (steps_remaining <= 0) {
      // ถึงเป้าหมาย: หยุด
      RUN_DIR = 0;
      // ปล่อย ENA หากต้องการประหยัดความร้อน (คอมเมนต์บรรทัดถัดไปเพื่อ "ค้างแรงบิด")
      enable_driver(false);
    }
  }

  // อนุญาตให้ปรับคาบระหว่างวิ่ง (ถ้าคุณไปแก้ HALF_PERIOD_US ใน runtime)
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
}

// ---------------- Core motion ----------------
static inline void start_move_steps(long delta_steps)
{
  if (delta_steps == 0) {
    // ไม่ต้องทำอะไร
    RUN_DIR = 0;
    enable_driver(false);
    return;
  }

  int8_t dir = (delta_steps > 0) ? 1 : -1;           // delta > 0 => CW
  long   todo = labs(delta_steps);

  set_direction(dir);
  enable_driver(true);

  portENTER_CRITICAL_ISR(&spinlock);
  RUN_DIR = dir;
  steps_remaining = todo;
  portEXIT_CRITICAL_ISR(&spinlock);
}

static inline long steps_from_degrees_int(int16_t deg)
{
  // deg 0..359 -> step เป้าหมาย (absolute)
  float stepf = ((float)deg / 360.0f) * (float)STEPS_PER_REV;
  long s = lroundf(stepf);
  // ห่อให้อยู่ในรอบเดียว 0..STEPS_PER_REV-1 เพื่อความเสถียร
  s %= STEPS_PER_REV;
  if (s < 0) s += STEPS_PER_REV;
  return s;
}

static inline long shortest_delta(long current, long target)
{
  // คืนค่า delta ในช่วง [-STEPS_PER_REV/2, +STEPS_PER_REV/2]
  long diff = target - (current % STEPS_PER_REV);
  if (diff < 0) diff += STEPS_PER_REV;
  // ตอนนี้ diff อยู่ช่วง [0, STEPS_PER_REV)
  if (diff > (STEPS_PER_REV/2)) diff -= STEPS_PER_REV; // เลือกทางสั้นกว่า
  return diff;  // บวก=หมุน CW, ลบ=หมุน CCW
}

static void go_to_degree(int16_t deg)
{
  // Normalize
  int16_t d = deg % 360;
  if (d < 0) d += 360;

  long target_steps_abs = steps_from_degrees_int(d);
  long cur = current_steps;
  long delta = shortest_delta(cur, target_steps_abs);

  start_move_steps(delta);
}

// ---------------- Subscriber callback ----------------
static void cmd_cb(const void* msgin)
{
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  const int16_t target_deg = m->data;   // คาดว่า 0..359 จาก /man/cmd_tao

  // ตั้งเป้า
  go_to_degree(target_deg);

  // ส่งเฟีดแบ็กมุม (int) ตอนรับคำสั่งทันทีหนึ่งครั้ง
  publish_angle_now();

  Serial.print("[CMD] target_deg=");
  Serial.print(target_deg);
  Serial.print(" | cur_deg=");
  Serial.println((int)angle_int_from_steps(current_steps));
}

// ---------------- micro-ROS entities ----------------
static bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  // ROS_DOMAIN_ID = 96 (ปรับได้)

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
  timerAlarmEnable(tmr);                       // ยิง ISR ตลอด (ISR จะดู RUN_DIR/steps_remaining)

  // เริ่มต้นประกาศมุม 0° (assume โฮมที่ 0 ตอนบูต)
  publish_angle_now();
}

void loop()
{
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

        // ส่งเฟีดแบ็กมุมเป็นระยะ (optional)
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
