// =========================== main.cpp (ESP32 + micro-ROS, Skid-Steer + Odometry, VX_MAX/WZ_MAX + per-wheel topics) ===========================
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
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ---------------- Driver ----------------
// #define DRIVER_L298N
#define DRIVER_CYTRON

// ---------------- Drive model -----------
#define DRIVE_SKID_STEER

#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ rclErrorLoop(); } }
#define RCSOFTCHECK(fn) { (void)(fn); }

#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t=-1; if(t==-1)t=uxr_millis(); if(uxr_millis()-t>(MS)){ X; t=uxr_millis(); } }while(0)

// ---------------- Pins & PWM ------------
#define L_DIR1  32
#define L_PWM1  33
#define L_DIR2  25
#define L_PWM2  26

#define R_DIR1  22
#define R_PWM1  23
#define R_DIR2  19
#define R_PWM2  21

// Quadrature encoders (A/B)
#define ENC1_A  35
#define ENC1_B  34
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
const float WHEEL_RADIUS = 0.060f;   // m
const float TRACK_WIDTH  = 0.200f;   // m
const float MAX_RPM      = 60.0f;    // motor limit

// Encoder model
const int   TICKS_PER_REV = 600;     // ticks per wheel revolution (แก้ให้ตรงจริง)
const float GEAR_RATIO    = 1.0f;    // wheel_rev = motor_rev / GEAR_RATIO
const bool  ENC_USE_X4    = true;

// -------- Command limits (software) -----
const float VX_MAX = 0.80f;          // m/s
const float WZ_MAX = 3.80f;          // rad/s

// Smooth/safety
const float DEADBAND           = 0.02f;
const uint32_t CMD_TIMEOUT_MS  = 300;
const float SLEW_RPM_PER_SEC   = 800.0f;

// ---------------- Encoders --------------
volatile long encoder1_count=0, encoder2_count=0, encoder3_count=0, encoder4_count=0;

// --------- Odom / per-wheel states ------
float odom_x=0.0f, odom_y=0.0f, odom_yaw=0.0f;
long  last_e1=0, last_e2=0, last_e3=0, last_e4=0;
float distance_accum=0.0f; // total path length (m)

// per-wheel accumulative distances (m)
float s1_acc=0.0f, s2_acc=0.0f, s3_acc=0.0f, s4_acc=0.0f;

// ---------------- ROS entities ----------
rcl_publisher_t debug_motor_pub;     // Float32MultiArray (duty)
std_msgs__msg__Float32MultiArray debug_motor_msg;

rcl_publisher_t encoder_pub16;       // Int16MultiArray (legacy quick debug)
std_msgs__msg__Int16MultiArray encoder_msg16;

rcl_publisher_t raw_counts_pub;      // Int32MultiArray (e1..e4)
std_msgs__msg__Int32MultiArray raw_counts_msg;

rcl_publisher_t odom_pub;            // nav_msgs/Odometry
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t distance_pub;        // std_msgs/Float32 (total)
std_msgs__msg__Float32 distance_msg;

rcl_publisher_t wheel_dist4_pub;     // Float32MultiArray (s1..s4 accum)
std_msgs__msg__Float32MultiArray wheel_dist4_msg;

rcl_publisher_t wheel_vel4_pub;      // Float32MultiArray (v1..v4 instant m/s)
std_msgs__msg__Float32MultiArray wheel_vel4_msg;

rcl_publisher_t counter_pub;         // Int32
std_msgs__msg__Int32 counter_msg;

rcl_subscription_t cmd_sub;          // Twist /man/cmd_move
geometry_msgs__msg__Twist cmd_msg;

rcl_timer_t control_timer, counter_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state = WAITING_AGENT;

// ---------------- Command state ----------
volatile float cmd_vx = 0.0f;  // m/s
volatile float cmd_wz = 0.0f;  // rad/s
volatile uint32_t last_cmd_ms = 0;

// ---------------- Motor/RPM state -------
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// ---------- Helpers ----------
static inline uint8_t rpm_to_duty(float rpm){
  float a = fabsf(rpm)/MAX_RPM; if(a>1)a=1;
  return (uint8_t)(a*255.0f);
}
static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float v_to_rpm(float v_mps){ return (v_mps / WHEEL_RADIUS) * 60.0f / (2.0f*(float)M_PI); }
static inline float norm_angle(float a){ while(a >  M_PI) a-=2*M_PI; while(a < -M_PI) a+=2*M_PI; return a; }
static inline float expo_curve(float u, float e){ float a = clampf(e, 0.0f, 1.0f); return (1.0f - a)*u + a*(u*u*u); }

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
void computeAndPublishOdom(float dt);
void controlCb(rcl_timer_t*, int64_t);
void counterCb(rcl_timer_t*, int64_t);
void twistCb(const void *msgin);

// ======================= setup/loop ===============================================
static void smartPinMode(int pin){
  // GPIO 34..39 ไม่มี PULLUP ภายใน → ใช้ INPUT
  if(pin>=34 && pin<=39) pinMode(pin, INPUT);
  else                   pinMode(pin, INPUT_PULLUP);
}

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

  smartPinMode(ENC1_A); smartPinMode(ENC1_B);
  smartPinMode(ENC2_A); smartPinMode(ENC2_B);
  smartPinMode(ENC3_A); smartPinMode(ENC3_B);
  smartPinMode(ENC4_A); smartPinMode(ENC4_B);

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
  computeAndPublishOdom(dt);
  publishData();
}

void counterCb(rcl_timer_t*, int64_t){
  static int32_t c=0; counter_msg.data = ++c;
  RCSOFTCHECK(rcl_publish(&counter_pub, &counter_msg, NULL));
}

void twistCb(const void *msgin){
  const auto *m = (const geometry_msgs__msg__Twist*)msgin;

  float vx = (float)m->linear.x;   // m/s
  float wz = (float)m->angular.z;  // rad/s

  // gain for steering input
  wz *= 8.0f;

  // clamp & expo
  vx = clampf(vx, -VX_MAX, VX_MAX);
  wz = clampf(wz, -WZ_MAX, WZ_MAX);

  vx = VX_MAX * expo_curve(vx / VX_MAX, 0.30f);
  wz = WZ_MAX * expo_curve(wz / WZ_MAX, 0.50f);

  // reduce forward speed when turning hard
  float turn_mix = 1.0f - 0.5f * (fabsf(wz)/WZ_MAX);
  vx *= clampf(turn_mix, 0.5f, 1.0f);

  if (fabsf(vx) < DEADBAND) vx = 0.0f;
  if (fabsf(wz) < DEADBAND) wz = 0.0f;

  cmd_vx = vx;
  cmd_wz = wz;
  last_cmd_ms = millis();
}

// ======================= Entities ================================================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  // messages (allocate arrays)
  debug_motor_msg.data.capacity = 4; debug_motor_msg.data.size = 4;
  debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));

  encoder_msg16.data.capacity = 4; encoder_msg16.data.size = 4;
  encoder_msg16.data.data = (int16_t*)malloc(4*sizeof(int16_t));

  raw_counts_msg.data.capacity = 4; raw_counts_msg.data.size = 4;
  raw_counts_msg.data.data = (int32_t*)malloc(4*sizeof(int32_t));

  wheel_dist4_msg.data.capacity = 4; wheel_dist4_msg.data.size = 4;
  wheel_dist4_msg.data.data = (float*)malloc(4*sizeof(float));

  wheel_vel4_msg.data.capacity = 4; wheel_vel4_msg.data.size = 4;
  wheel_vel4_msg.data.data = (float*)malloc(4*sizeof(float));

  std_msgs__msg__Float32__init(&distance_msg);
  std_msgs__msg__Int32__init(&counter_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);
  nav_msgs__msg__Odometry__init(&odom_msg);

  // init node
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));   // ให้ตรงกับฝั่ง Pi

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_skid_controller", "", &support));

  // publishers
  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));

  RCCHECK(rclc_publisher_init_best_effort(&encoder_pub16, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/motor_feedback/encoders16"));

  RCCHECK(rclc_publisher_init_best_effort(&raw_counts_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/encoders/raw_counts"));

  RCCHECK(rclc_publisher_init_best_effort(&odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/wheel_odom"));

  RCCHECK(rclc_publisher_init_best_effort(&distance_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/wheel_distance"));

  RCCHECK(rclc_publisher_init_best_effort(&wheel_dist4_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_distance_per_wheel"));

  RCCHECK(rclc_publisher_init_best_effort(&wheel_vel4_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/wheel_velocity_per_wheel"));

  // subscriber
  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/man/cmd_move"));

  // timers
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCb));    // 50 Hz
  RCCHECK(rclc_timer_init_default(&counter_timer, &support, RCL_MS_TO_NS(1000), counterCb));

  // executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &twistCb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));

  syncTime();
  last_cmd_ms = millis();
  last_e1 = encoder1_count; last_e2 = encoder2_count;
  last_e3 = encoder3_count; last_e4 = encoder4_count;
  return true;
}

bool destroyEntities(){
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&debug_motor_pub, &node);
  rcl_publisher_fini(&encoder_pub16, &node);
  rcl_publisher_fini(&raw_counts_pub, &node);
  rcl_publisher_fini(&odom_pub, &node);
  rcl_publisher_fini(&distance_pub, &node);
  rcl_publisher_fini(&wheel_dist4_pub, &node);
  rcl_publisher_fini(&wheel_vel4_pub, &node);
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&counter_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if (debug_motor_msg.data.data) free(debug_motor_msg.data.data);
  if (encoder_msg16.data.data) free(encoder_msg16.data.data);
  if (raw_counts_msg.data.data) free(raw_counts_msg.data.data);
  if (wheel_dist4_msg.data.data) free(wheel_dist4_msg.data.data);
  if (wheel_vel4_msg.data.data) free(wheel_vel4_msg.data.data);
  return true;
}

// ======================= Control ================================================
void controlStep(float dt){
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
    tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
  } else {
    float vx = cmd_vx;   // m/s
    float wz = cmd_wz;   // rad/s

    float v_left  = vx - wz * (TRACK_WIDTH * 0.5f);
    float v_right = vx + wz * (TRACK_WIDTH * 0.5f);

    float rpm_left  = clampf(v_to_rpm(v_left),  -MAX_RPM, MAX_RPM);
    float rpm_right = clampf(v_to_rpm(v_right), -MAX_RPM, MAX_RPM);

    tgt_m1_rpm = rpm_left;  // LF
    tgt_m2_rpm = rpm_right; // RF
    tgt_m3_rpm = rpm_left;  // LR
    tgt_m4_rpm = rpm_right; // RR
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

// ----- Encoder → Odometry & per-wheel topics -----
void computeAndPublishOdom(float dt){
  // read deltas
  long e1 = encoder1_count, e2 = encoder2_count, e3 = encoder3_count, e4 = encoder4_count;
  long d1 = e1 - last_e1, d2 = e2 - last_e2, d3 = e3 - last_e3, d4 = e4 - last_e4;
  last_e1 = e1; last_e2 = e2; last_e3 = e3; last_e4 = e4;

  // ticks per wheel revolution
  const float ticks_per_rev_wheel = (ENC_USE_X4 ? 4.0f : 1.0f) * (float)TICKS_PER_REV / GEAR_RATIO;

  auto ticks_to_s = [&](long dticks)->float{
    return ( (2.0f * (float)M_PI * WHEEL_RADIUS) * ((float)dticks / ticks_per_rev_wheel) );
  };

  // per-wheel distances (this cycle)
  float s1 = ticks_to_s(d1); // LF
  float s2 = ticks_to_s(d2); // RF
  float s3 = ticks_to_s(d3); // LR
  float s4 = ticks_to_s(d4); // RR

  // per-wheel instantaneous velocities (m/s)
  float v1 = s1 / dt;
  float v2 = s2 / dt;
  float v3 = s3 / dt;
  float v4 = s4 / dt;

  // accumulate per-wheel distances
  s1_acc += fabsf(s1);
  s2_acc += fabsf(s2);
  s3_acc += fabsf(s3);
  s4_acc += fabsf(s4);

  // side averages
  float sL = 0.5f*(s1 + s3);
  float sR = 0.5f*(s2 + s4);

  // body velocity
  float v  = (sR + sL) * 0.5f / dt;
  float wz = (sR - sL) / TRACK_WIDTH / dt;

  // integrate pose
  float ds = (sR + sL)*0.5f;
  if (fabsf(wz) < 1e-6f){
    odom_x += ds * cosf(odom_yaw);
    odom_y += ds * sinf(odom_yaw);
  }else{
    float r = v / wz;
    float dtheta = (sR - sL) / TRACK_WIDTH;
    odom_x += r * (sinf(odom_yaw + dtheta) - sinf(odom_yaw));
    odom_y += r * (-cosf(odom_yaw + dtheta) + cosf(odom_yaw));
    odom_yaw = norm_angle(odom_yaw + dtheta);
  }
  odom_yaw = norm_angle(odom_yaw);

  // total distance
  distance_accum += fabsf(ds);

  // ---- publish odometry ----
  odom_msg.header.frame_id.data = const_cast<char*>("odom");
  odom_msg.child_frame_id.data  = const_cast<char*>("base_link");
  odom_msg.header.stamp.sec     = (int32_t)(millis()/1000);
  odom_msg.header.stamp.nanosec = (uint32_t)((millis()%1000)*1000000UL);

  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;

  float cy = cosf(odom_yaw*0.5f), sy = sinf(odom_yaw*0.5f);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sy;
  odom_msg.pose.pose.orientation.w = cy;

  odom_msg.twist.twist.linear.x  = v;
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.angular.z = wz;

  RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));

  // ---- publish total distance ----
  distance_msg.data = distance_accum;
  RCSOFTCHECK(rcl_publish(&distance_pub, &distance_msg, NULL));

  // ---- publish per-wheel accum distances ----
  wheel_dist4_msg.data.data[0] = s1_acc;
  wheel_dist4_msg.data.data[1] = s2_acc;
  wheel_dist4_msg.data.data[2] = s3_acc;
  wheel_dist4_msg.data.data[3] = s4_acc;
  RCSOFTCHECK(rcl_publish(&wheel_dist4_pub, &wheel_dist4_msg, NULL));

  // ---- publish per-wheel velocities (m/s) ----
  wheel_vel4_msg.data.data[0] = v1;
  wheel_vel4_msg.data.data[1] = v2;
  wheel_vel4_msg.data.data[2] = v3;
  wheel_vel4_msg.data.data[3] = v4;
  RCSOFTCHECK(rcl_publish(&wheel_vel4_pub, &wheel_vel4_msg, NULL));

  // ---- publish raw counts (int32) ----
  raw_counts_msg.data.data[0] = (int32_t)e1;
  raw_counts_msg.data.data[1] = (int32_t)e2;
  raw_counts_msg.data.data[2] = (int32_t)e3;
  raw_counts_msg.data.data[3] = (int32_t)e4;
  RCSOFTCHECK(rcl_publish(&raw_counts_pub, &raw_counts_msg, NULL));
}

void publishData(){
  // motor duty (0..255) for quick debug
  debug_motor_msg.data.data[0] = rpm_to_duty(m1_rpm);
  debug_motor_msg.data.data[1] = rpm_to_duty(m2_rpm);
  debug_motor_msg.data.data[2] = rpm_to_duty(m3_rpm);
  debug_motor_msg.data.data[3] = rpm_to_duty(m4_rpm);
  RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

  // legacy 16-bit encoder snapshot (kept for compatibility)
  encoder_msg16.data.data[0] = (int16_t)(encoder1_count & 0xFFFF);
  encoder_msg16.data.data[1] = (int16_t)(encoder2_count & 0xFFFF);
  encoder_msg16.data.data[2] = (int16_t)(encoder3_count & 0xFFFF);
  encoder_msg16.data.data[3] = (int16_t)(encoder4_count & 0xFFFF);
  RCSOFTCHECK(rcl_publish(&encoder_pub16, &encoder_msg16, NULL));
}

// ======================= Time & Error ===========================================
void syncTime(){
  RCCHECK(rmw_uros_sync_session(10));
}
void rclErrorLoop(){
  const int LED_PIN=13; pinMode(LED_PIN,OUTPUT);
  while(true){ digitalWrite(LED_PIN,HIGH); delay(100); digitalWrite(LED_PIN,LOW); delay(100); }
}
