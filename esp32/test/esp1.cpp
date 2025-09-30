// ========================== esp1_combined_main.cpp (FULL, COMPILE-READY) ==========================

#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

#define DRIVER_CYTRON
#define DRIVE_DIFF

#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { (void)(fn); }
#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t=-1; if(t==-1)t=uxr_millis(); if(uxr_millis()-t>(MS)){ X; t=uxr_millis(); } }while(0)

// ---------------- DC Motor Pins & PWM (updated) ----------------
#define L_DIR1  32
#define L_PWM1  33
#define L_DIR2  18    // moved from 26
#define L_PWM2  13    // moved from 25
#define R_DIR1  22
#define R_PWM1  23
#define R_DIR2  19
#define R_PWM2  21

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3

// ---------------- Encoders ----------------
#define ENC1_A  34
#define ENC1_B  35
#define ENC3_A  36
#define ENC3_B  39
#define ENC2_A   4    // moved from 27
#define ENC2_B  14
#define ENC4_A  16
#define ENC4_B  17

volatile long encoder1_count=0, encoder2_count=0, encoder3_count=0, encoder4_count=0;

// ---------------- Robot params (DC) ----------------
const float WHEEL_RADIUS = 0.060f;
const float TRACK_WIDTH  = 0.200f;
const float MAX_RPM      = 60.0f;

const float DEADBAND           = 0.02f;
const uint32_t CMD_TIMEOUT_MS  = 300;
const float SLEW_RPM_PER_SEC   = 400.0f;

// ---------------- Stepper pins (25/26/27) ----------------
static const int PIN_PUL = 25;
static const int PIN_DIR = 26;
static const int PIN_ENA = 27;

// ---------------- Stepper configuration ----------------
static const int BASE_STEPS_PER_REV = 400;   // keep as in your previous code
static const int MICROSTEP          = 16;    // DIP: 1,2,4,8,16,32
static const float GEAR_RATIO       = 1.0f;
static const long STEPS_PER_REV     = (long)(BASE_STEPS_PER_REV * MICROSTEP * GEAR_RATIO);

// pulse freq ~= 1/(2*HALF_PERIOD_US)
static volatile uint32_t HALF_PERIOD_US = 40;  // 40us ≈ 12.5kHz

// ---------------- ROS entities ----------------
rcl_publisher_t debug_motor_pub;     // Float32MultiArray
std_msgs__msg__Float32MultiArray debug_motor_msg;

rcl_publisher_t encoder_pub;         // Int16MultiArray
std_msgs__msg__Int16MultiArray encoder_msg;

rcl_publisher_t counter_pub;         // Int32
std_msgs__msg__Int32 counter_msg;

rcl_publisher_t stepper_fb_pub;      // Int16 (0..359)
std_msgs__msg__Int16 stepper_fb_msg;

rcl_subscription_t cmd_sub;          // Twist /man/cmd_move
geometry_msgs__msg__Twist cmd_msg;

rcl_subscription_t tao_sub;          // Int16 /man/cmd_tao
std_msgs__msg__Int16 tao_msg;

rcl_timer_t control_timer;           // 50 Hz
rcl_timer_t fb_timer;                // 10 Hz

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_init_options_t init_options;

enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;

// ---------------- Command state (DC) ----------------
volatile float cmd_vx = 0.0f;
volatile float cmd_wz = 0.0f;
volatile uint32_t last_cmd_ms = 0;

// ---------------- Motor/RPM state (DC) --------------
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// ---------------- Stepper runtime state -------------
static volatile int8_t  RUN_DIR = 0;         // +1=CW, -1=CCW, 0=STOP
static volatile bool    pul_high = false;
static volatile long    steps_remaining = 0;
static volatile long    current_steps   = 0;

hw_timer_t* tmr = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ---------------- Helpers ----------------
uint8_t rpm_to_duty(float rpm){
  float a = fabsf(rpm)/MAX_RPM; if(a>1)a=1;
  return (uint8_t)(a*255.0f);
}
float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
float v_to_rpm(float v_mps){ return (v_mps / WHEEL_RADIUS) * 60.0f / (2.0f*(float)M_PI); }

void IRAM_ATTR encoder1_ISR(){ (digitalRead(ENC1_A)==digitalRead(ENC1_B))?encoder1_count++:encoder1_count--; }
void IRAM_ATTR encoder2_ISR(){ (digitalRead(ENC2_A)==digitalRead(ENC2_B))?encoder2_count++:encoder2_count--; }
void IRAM_ATTR encoder3_ISR(){ (digitalRead(ENC3_A)==digitalRead(ENC3_B))?encoder3_count++:encoder3_count--; }
void IRAM_ATTR encoder4_ISR(){ (digitalRead(ENC4_A)==digitalRead(ENC4_B))?encoder4_count++:encoder4_count--; }

void setMotor1(float rpm){ digitalWrite(L_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M1, rpm_to_duty(rpm)); }
void setMotor2(float rpm){ digitalWrite(R_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M2, rpm_to_duty(rpm)); }
void setMotor3(float rpm){ digitalWrite(L_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M3, rpm_to_duty(rpm)); }
void setMotor4(float rpm){ digitalWrite(R_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M4, rpm_to_duty(rpm)); }

// ---------- Stepper helpers ----------
static inline void enable_driver(bool en_low_active = true){
  // Active-Low: LOW=Enable, HIGH=Disable
  digitalWrite(PIN_ENA, en_low_active ? LOW : HIGH);
}
static inline void set_direction(int8_t dir){
  // NOTE: LOW/HIGH ขึ้นกับการต่อจริง — ถ้าทิศกลับ สลับที่นี่
  digitalWrite(PIN_DIR, (dir == 1) ? LOW : HIGH);
}
static inline long steps_from_degrees_int(int16_t deg){
  float stepf = ((float)deg / 360.0f) * (float)STEPS_PER_REV;
  long s = lroundf(stepf);
  s %= STEPS_PER_REV;
  if (s < 0) s += STEPS_PER_REV;
  return s;
}
static inline int16_t angle_int_from_steps(long s){
  long steps = s % STEPS_PER_REV;
  if (steps < 0) steps += STEPS_PER_REV;
  float deg = ((float)steps * 360.0f) / (float)STEPS_PER_REV;
  int16_t a = (int16_t)lroundf(deg);
  a %= 360;
  if (a < 0) a += 360;
  return a;
}
static inline long shortest_delta(long current, long target){
  long cur = current % STEPS_PER_REV;
  if (cur < 0) cur += STEPS_PER_REV;
  long diff = target - cur;        // [-inf, inf]
  diff %= STEPS_PER_REV;
  if (diff < 0) diff += STEPS_PER_REV;          // [0, STEPS_PER_REV)
  if (diff > (STEPS_PER_REV/2)) diff -= STEPS_PER_REV; // [-half, +half]
  return diff;
}
static inline void publish_angle_now(){
  stepper_fb_msg.data = angle_int_from_steps(current_steps);
  RCSOFTCHECK(rcl_publish(&stepper_fb_pub, &stepper_fb_msg, NULL));
}

// ---------- Stepper Timer ISR ----------
void IRAM_ATTR onTimer(){
  if (RUN_DIR == 0 || steps_remaining <= 0) {
    if (pul_high) { REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); pul_high = false; }
    return;
  }
  uint32_t mask = (1U << PIN_PUL);
  if (!pul_high) {
    REG_WRITE(GPIO_OUT_W1TS_REG, mask); // HIGH
    pul_high = true;
  } else {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask); // LOW
    pul_high = false;

    steps_remaining--;
    if (RUN_DIR == 1)       current_steps++;
    else if (RUN_DIR == -1) current_steps--;

    if (steps_remaining <= 0) {
      RUN_DIR = 0;
      enable_driver(false); // ปลด ENA (ค้างแรงบิดให้คอมเมนต์บรรทัดนี้)
    }
  }
  timerAlarmWrite(tmr, HALF_PERIOD_US, true); // allow runtime speed change
}

// ---------- Stepper motion core ----------
static inline void start_move_steps(long delta_steps){
  if (delta_steps == 0){
    RUN_DIR = 0;
    enable_driver(false);
    return;
  }
  int8_t dir = (delta_steps > 0) ? 1 : -1;
  long todo = labs(delta_steps);

  set_direction(dir);
  enable_driver(true);

  portENTER_CRITICAL_ISR(&spinlock);
  RUN_DIR = dir;
  steps_remaining = todo;
  portEXIT_CRITICAL_ISR(&spinlock);
}
static void go_to_degree(int16_t deg){
  int16_t d = deg % 360;
  if (d < 0) d += 360;
  long target_steps_abs = steps_from_degrees_int(d);
  long delta = shortest_delta(current_steps, target_steps_abs);
  start_move_steps(delta);
}

// ---------- Protos ----------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void controlCb(rcl_timer_t*, int64_t);
void fbCb(rcl_timer_t*, int64_t);
void twistCb(const void *msgin);
void taoCb(const void *msgin);

// ======================= setup/loop =======================
void setup(){
  // DC GPIO
  pinMode(L_DIR1,OUTPUT); pinMode(L_DIR2,OUTPUT);
  pinMode(R_DIR1,OUTPUT); pinMode(R_DIR2,OUTPUT);
  digitalWrite(L_DIR1,LOW); digitalWrite(L_DIR2,LOW);
  digitalWrite(R_DIR1,LOW); digitalWrite(R_DIR2,LOW);

  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_PWM1, PWM_CH_M1);
  ledcAttachPin(R_PWM1, PWM_CH_M2);
  ledcAttachPin(L_PWM2, PWM_CH_M3);
  ledcAttachPin(R_PWM2, PWM_CH_M4);

  pinMode(ENC1_A,INPUT_PULLUP); pinMode(ENC1_B,INPUT_PULLUP);
  pinMode(ENC2_A,INPUT_PULLUP); pinMode(ENC2_B,INPUT_PULLUP);
  pinMode(ENC3_A,INPUT_PULLUP); pinMode(ENC3_B,INPUT_PULLUP);
  pinMode(ENC4_A,INPUT_PULLUP); pinMode(ENC4_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4_ISR, CHANGE);

  // Stepper GPIO
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_PUL, LOW);
  digitalWrite(PIN_DIR, LOW);
  enable_driver(false); // ปิดไว้ก่อน

  // micro-ROS transport
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // Stepper timer: 80 MHz / 80 = 1 MHz => 1 tick = 1 µs
  tmr = timerBegin(0, 80, true);
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
  timerAlarmEnable(tmr); // ยิง ISR ตลอด

  // first FB
  stepper_fb_msg.data = 0;
  RCSOFTCHECK(rcl_publish(&stepper_fb_pub, &stepper_fb_msg, NULL));
}

void loop(){
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1))?AGENT_AVAILABLE:WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = createEntities()?AGENT_CONNECTED:WAITING_AGENT;
      if(state==WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1))?AGENT_CONNECTED:AGENT_DISCONNECTED;);
      if(state==AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // ไม่บล็อก ISR
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      // หยุด stepper อย่างปลอดภัย
      portENTER_CRITICAL_ISR(&spinlock);
      RUN_DIR = 0;
      steps_remaining = 0;
      portEXIT_CRITICAL_ISR(&spinlock);
      enable_driver(false);
      state = WAITING_AGENT;
      break;
  }
}

// ======================= Callbacks =======================
void controlCb(rcl_timer_t*, int64_t){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  float dt = (now - last_ms) / 1000.0f; if(dt<=0) dt = 0.02f;
  last_ms = now;

  // timeout => stop
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
    tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
  } else {
    float vx = cmd_vx;   // m/s
    float wz = cmd_wz;   // rad/s

    // เพิ่ม gain การหมุน (คุณตั้งไว้ *8)
    wz *= 8.0f;

    if (fabsf(vx) < DEADBAND) vx = 0.0f;
    if (fabsf(wz) < DEADBAND) wz = 0.0f;

    float v_left  = vx - wz * (TRACK_WIDTH * 0.5f);
    float v_right = vx + wz * (TRACK_WIDTH * 0.5f);
    float rpm_left  = clampf(v_to_rpm(v_left),  -MAX_RPM, MAX_RPM);
    float rpm_right = clampf(v_to_rpm(v_right), -MAX_RPM, MAX_RPM);

    tgt_m1_rpm = rpm_left;
    tgt_m2_rpm = rpm_right;
    tgt_m3_rpm = rpm_left;
    tgt_m4_rpm = rpm_right;
  }

  auto slew = [&](float cur, float tgt)->float{
    float max_step = SLEW_RPM_PER_SEC * dt;
    float diff = tgt - cur;
    if (diff >  max_step) diff =  max_step;
    if (diff < -max_step) diff = -max_step;
    return cur + diff;
  };

  m1_rpm = slew(m1_rpm, clampf(tgt_m1_rpm, -MAX_RPM, MAX_RPM));
  m2_rpm = slew(m2_rpm, clampf(tgt_m2_rpm, -MAX_RPM, MAX_RPM));
  m3_rpm = slew(m3_rpm, clampf(tgt_m3_rpm, -MAX_RPM, MAX_RPM));
  m4_rpm = slew(m4_rpm, clampf(tgt_m4_rpm, -MAX_RPM, MAX_RPM));

  setMotor1(m1_rpm);
  setMotor2(m2_rpm);
  setMotor3(m3_rpm);
  setMotor4(m4_rpm);

  // publish DC debug + enc
  debug_motor_msg.data.data[0] = rpm_to_duty(m1_rpm);
  debug_motor_msg.data.data[1] = rpm_to_duty(m2_rpm);
  debug_motor_msg.data.data[2] = rpm_to_duty(m3_rpm);
  debug_motor_msg.data.data[3] = rpm_to_duty(m4_rpm);
  RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

  encoder_msg.data.data[0] = (int16_t)(encoder1_count & 0xFFFF);
  encoder_msg.data.data[1] = (int16_t)(encoder2_count & 0xFFFF);
  encoder_msg.data.data[2] = (int16_t)(encoder3_count & 0xFFFF);
  encoder_msg.data.data[3] = (int16_t)(encoder4_count & 0xFFFF);
  RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
}

void fbCb(rcl_timer_t*, int64_t){
  // ส่งมุมสเต็ปเปอร์ + counter ทุก 100 ms
  publish_angle_now();
  static int32_t c=0; counter_msg.data = ++c;
  RCSOFTCHECK(rcl_publish(&counter_pub, &counter_msg, NULL));
}

void twistCb(const void *msgin){
  const auto *m = (const geometry_msgs__msg__Twist*)msgin;
  float vx = (float)m->linear.x;
  float wz = (float)m->angular.z;

  // หมุนไวขึ้นตามที่ตั้ง
  wz *= 8.0f;

  if (fabsf(vx) < DEADBAND) vx = 0.0f;
  if (fabsf(wz) < DEADBAND) wz = 0.0f;

  cmd_vx = vx;
  cmd_wz = wz;
  last_cmd_ms = millis();
}

void taoCb(const void *msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  const int16_t target_deg = m->data;   // 0..359
  go_to_degree(target_deg);
  publish_angle_now(); // แจ้งค่าทันทีครั้งหนึ่ง
}

// ======================= Entities =======================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  debug_motor_msg.data.capacity = 4; debug_motor_msg.data.size = 4;
  debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));
  encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
  encoder_msg.data.data = (int16_t*)malloc(4*sizeof(int16_t));
  std_msgs__msg__Int32__init(&counter_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);
  std_msgs__msg__Int16__init(&tao_msg);
  std_msgs__msg__Int16__init(&stepper_fb_msg);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));   // ให้ตรงกับฝั่ง Pi

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_combo_node", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));
  RCCHECK(rclc_publisher_init_best_effort(&encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/motor_feedback/encoders"));
  RCCHECK(rclc_publisher_init_best_effort(&counter_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/debug/counter"));
  RCCHECK(rclc_publisher_init_best_effort(&stepper_fb_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/man/cmd_tao/fb"));

  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/man/cmd_move"));
  RCCHECK(rclc_subscription_init_default(&tao_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/man/cmd_tao"));

  // Timers
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCb)); // 50Hz
  RCCHECK(rclc_timer_init_default(&fb_timer, &support, RCL_MS_TO_NS(100), fbCb));          // 10Hz

  // Executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &twistCb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &tao_sub, &tao_msg, &taoCb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &fb_timer));

  syncTime();
  last_cmd_ms = millis();
  return true;
}

bool destroyEntities(){
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&debug_motor_pub, &node);
  rcl_publisher_fini(&encoder_pub, &node);
  rcl_publisher_fini(&counter_pub, &node);
  rcl_publisher_fini(&stepper_fb_pub, &node);

  rcl_subscription_fini(&cmd_sub, &node);
  rcl_subscription_fini(&tao_sub, &node);

  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&fb_timer);

  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if (debug_motor_msg.data.data) free(debug_motor_msg.data.data);
  if (encoder_msg.data.data)     free(encoder_msg.data.data);
  return true;
}

// ======================= Time & Error =======================
void syncTime(){ RCCHECK(rmw_uros_sync_session(10)); }
void rclErrorLoop(){
  const int LED_PIN=13; pinMode(LED_PIN,OUTPUT);
  while(true){ digitalWrite(LED_PIN,HIGH); delay(100); digitalWrite(LED_PIN,LOW); delay(100); }
}
