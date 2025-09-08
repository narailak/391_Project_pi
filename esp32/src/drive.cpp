// =========================== main.cpp (ESP32 + micro-ROS, Differential) ===========================
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
#include <geometry_msgs/msg/twist.h>

// ---------------- Drive model -----------
#define DRIVE_DIFF

#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { (void)(fn); }

#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t=-1; if(t==-1)t=uxr_millis(); if(uxr_millis()-t>(MS)){ X; t=uxr_millis(); } }while(0)

// ---------------- Pins & PWM ------------
#define L_DIR1  32
#define L_PWM1  33
#define L_DIR2  26
#define L_PWM2  25

#define R_DIR1  22
#define R_PWM1  23
#define R_DIR2  19
#define R_PWM2  21

#define ENC1_A  34
#define ENC1_B  35
#define ENC3_A  36
#define ENC3_B  39
#define ENC2_A  27
#define ENC2_B  14
#define ENC4_A  16
#define ENC4_B  17

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3

// ---------------- Robot params ----------
// มอเตอร์สเปค: 60 rpm no-load, 46 rpm load
const float WHEEL_RADIUS = 0.060f;     // m
const float TRACK_WIDTH  = 0.200f;     // m
const float MAX_RPM      = 60.0f;      // ตั้งเพดานตามสเปคมอเตอร์ (รอบที่ล้อ)

// ความปลอดภัย/ความนิ่มนวล
const float DEADBAND           = 0.005f;    // ใกล้ศูนย์ตัดทิ้ง
const uint32_t CMD_TIMEOUT_MS  = 300;
const float SLEW_RPM_PER_SEC   = 1000.0f;   // อัตราเร่ง (rpm/s)

// ---------------- Gain ------------------
const float LIN_GAIN  = 1.0f;    // คันเร่งรวม ไม่ควรเกิน 3
const float TURN_GAIN = 1.0f;    // ความไวเลี้ยว

// ---------------- Encoders --------------
volatile long encoder1_count=0, encoder2_count=0, encoder3_count=0, encoder4_count=0;

// ---------------- ROS entities ----------
rcl_publisher_t debug_motor_pub;
std_msgs__msg__Float32MultiArray debug_motor_msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Int16MultiArray encoder_msg;

rcl_publisher_t counter_pub;
std_msgs__msg__Int32 counter_msg;

rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;

rcl_timer_t control_timer, counter_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;
unsigned long long time_offset = 0;

// ---------------- Command state ----------
volatile float cmd_vx = 0.0f;  // -1..1
volatile float cmd_wz = 0.0f;  // -1..1
volatile uint32_t last_cmd_ms = 0;

// ---------------- Motor/RPM state -------
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// ---------- Helpers ----------
uint8_t rpm_to_duty(float rpm){
  float a = fabsf(rpm)/MAX_RPM; if(a>1)a=1;
  const float MIN_DUTY = 0.18f;   // ค่าขั้นต่ำเอาชนะแรงเสียดทาน
  const float GAMMA    = 0.6f;    // <1 = ดันแรงช่วงต้น
  if (a < 1e-3f) return 0;
  float shaped = powf(a, GAMMA);
  float out = MIN_DUTY + (1.0f - MIN_DUTY) * shaped;
  return (uint8_t)(out * 255.0f);
}
float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

// ---------- Enc ISR ----------
void IRAM_ATTR encoder1_ISR(){ (digitalRead(ENC1_A)==digitalRead(ENC1_B))?encoder1_count++:encoder1_count--; }
void IRAM_ATTR encoder2_ISR(){ (digitalRead(ENC2_A)==digitalRead(ENC2_B))?encoder2_count++:encoder2_count--; }
void IRAM_ATTR encoder3_ISR(){ (digitalRead(ENC3_A)==digitalRead(ENC3_B))?encoder3_count++:encoder3_count--; }
void IRAM_ATTR encoder4_ISR(){ (digitalRead(ENC4_A)==digitalRead(ENC4_B))?encoder4_count++:encoder4_count--; }

// ---------- Motor out ----------
void setMotor1(float rpm){ digitalWrite(L_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M1, rpm_to_duty(rpm)); }
void setMotor2(float rpm){ digitalWrite(R_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M2, rpm_to_duty(rpm)); }
void setMotor3(float rpm){ digitalWrite(L_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M3, rpm_to_duty(rpm)); }
void setMotor4(float rpm){ digitalWrite(R_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M4, rpm_to_duty(rpm)); }

// ---------- Protos ----------
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void controlStep(float dt);
void publishData();
void controlCb(rcl_timer_t*, int64_t);
void counterCb(rcl_timer_t*, int64_t);
void twistCb(const void *msgin);

// ======================= setup/loop ===============================================
void setup(){
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

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
      if(state==AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      break;
    case AGENT_DISCONNECTED:
      destroyEntities(); state = WAITING_AGENT; break;
  }
}

// ======================= Callbacks ================================================
void controlCb(rcl_timer_t*, int64_t){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  float dt = (now - last_ms) / 1000.0f; if(dt<=0) dt = 0.02f;
  last_ms = now;
  controlStep(dt);
  publishData();
}

void counterCb(rcl_timer_t*, int64_t){
  static int32_t c=0; counter_msg.data = ++c;
  RCSOFTCHECK(rcl_publish(&counter_pub, &counter_msg, NULL));
}

void twistCb(const void *msgin){
  const auto *m = (const geometry_msgs__msg__Twist*)msgin;
  float vx = (float)m->linear.x;   // -1..1
  float wz = (float)m->angular.z;  // -1..1

  // clamp + gain
  vx = clampf(vx, -1.0f, 1.0f) * LIN_GAIN;
  wz = clampf(wz, -1.0f, 1.0f) * TURN_GAIN;

  if (fabsf(vx) < DEADBAND) vx = 0.0f;
  if (fabsf(wz) < DEADBAND) wz = 0.0f;

  cmd_vx = vx;
  cmd_wz = wz;
  last_cmd_ms = millis();
}

// ======================= Entities ================================================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  debug_motor_msg.data.capacity = 4; debug_motor_msg.data.size = 4;
  debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));
  encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
  encoder_msg.data.data = (int16_t*)malloc(4*sizeof(int16_t));
  std_msgs__msg__Int32__init(&counter_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diff_controller", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));
  RCCHECK(rclc_publisher_init_best_effort(&encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/motor_feedback/encoders"));
  RCCHECK(rclc_publisher_init_best_effort(&counter_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/debug/counter"));

  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/man/cmd_move"));

  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCb));
  RCCHECK(rclc_timer_init_default(&counter_timer, &support, RCL_MS_TO_NS(1000), counterCb));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &twistCb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));

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
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&counter_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if (debug_motor_msg.data.data) free(debug_motor_msg.data.data);
  if (encoder_msg.data.data) free(encoder_msg.data.data);
  return true;
}

// ======================= Control ================================================
void controlStep(float dt){
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
    tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
  } else {
    // สัดส่วนซ้าย/ขวา (−1..1)
    float l = cmd_vx - cmd_wz;
    float r = cmd_vx + cmd_wz;
    float mm = fmaxf(fabsf(l), fabsf(r));
    if (mm > 1.0f) { l /= mm; r /= mm; }

    tgt_m1_rpm = l * MAX_RPM;
    tgt_m2_rpm = r * MAX_RPM;
    tgt_m3_rpm = l * MAX_RPM;
    tgt_m4_rpm = r * MAX_RPM;
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
}

void publishData(){
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

// ======================= Time & Error ===========================================
void syncTime(){ RCCHECK(rmw_uros_sync_session(10)); }
void rclErrorLoop(){
  const int LED_PIN=13; pinMode(LED_PIN,OUTPUT);
  while(true){ digitalWrite(LED_PIN,HIGH); delay(100); digitalWrite(LED_PIN,LOW); delay(100); }
}
