// ===================== main.cpp (ESP32 + micro-ROS + ESP32Encoder - Full Version) =====================
#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <ESP32Encoder.h>

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

// ---------------- Driver ----------------
#define DRIVER_CYTRON

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

// Quadrature encoders
#define ENC1_A  35
#define ENC1_B  34
#define ENC3_A  4
#define ENC3_B  16
#define ENC2_A  27
#define ENC2_B  14
#define ENC4_A  5
#define ENC4_B  17

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3

// ============ ปรับตามมอเตอร์ 37mm ============
const float WHEEL_RADIUS = 0.0325f;         // m (ล้อ 65mm เส้นผ่านศูนย์กลาง → รัศมี 32.5mm)
const float TRACK_WIDTH  = 0.300f;          // m (ระยะระหว่างล้อซ้าย-ขวา)

// *** ข้อมูลมอเตอร์ ***
const float ENCODER_PPR      = 11.0f;       // pulse per revolution (ของ encoder เปล่า)
const float GEAR_RATIO       = 169.0f;      // อัตราทดเกียร์ 1:169
const float EFFECTIVE_PPR    = ENCODER_PPR * GEAR_RATIO;  // 11 × 169 = 1859 pulse/rev (ที่เพลาออก)

const float MAX_RPM_NO_LOAD  = 60.0f;       // rpm (ไม่มีโหลด)
const float MAX_RPM_RATED    = 46.0f;       // rpm (มีโหลด)
const float MAX_RPM          = MAX_RPM_RATED; // ใช้ค่ามีโหลดเพื่อความปลอดภัย

// ลิมิตกระแส
const float MOTOR_CURRENT_NOLOAD = 0.15f;   // A
const float MOTOR_CURRENT_LOAD   = 0.65f;   // A
const float MOTOR_CURRENT_STALL  = 2.4f;    // A (ห้ามให้ถึงจุดนี้นานๆ)

// ความปลอดภัย
const float DEADBAND           = 0.02f;     // m/s / rad/s
const uint32_t CMD_TIMEOUT_MS  = 300;       // ms
const float SLEW_RPM_PER_SEC   = 100.0f;    // RPM/s (ลดลงเพื่อป้องกัน stall)

// ---------------- Encoder Class --------------
enum WheelIndex { W_M1=0, W_M2=1, W_M3=2, W_M4=3, W_COUNT=4 };

class QuadEncoderReader {
public:
  QuadEncoderReader() {
    pins_[W_M1][0] = ENC1_A; pins_[W_M1][1] = ENC1_B;
    pins_[W_M2][0] = ENC2_A; pins_[W_M2][1] = ENC2_B;
    pins_[W_M3][0] = ENC3_A; pins_[W_M3][1] = ENC3_B;
    pins_[W_M4][0] = ENC4_A; pins_[W_M4][1] = ENC4_B;
    
    // ทิศทางการหมุน (ปรับตามการติดตั้งจริง)
    inv_[W_M1] = 1;
    inv_[W_M2] = 1;
    inv_[W_M3] = 1;
    inv_[W_M4] = 1;
  }

  void begin(bool enable_pullups = false) {
    // ESP32Encoder 0.11.8 syntax
    ESP32Encoder::useInternalWeakPullResistors = enable_pullups ? puType::up : puType::none;
    
    // ใช้ attachHalfQuad เพราะมอเตอร์นี้มี encoder แบบ 2-channel quadrature
    enc_[W_M1].attachHalfQuad(pins_[W_M1][0], pins_[W_M1][1]);
    enc_[W_M2].attachHalfQuad(pins_[W_M2][0], pins_[W_M2][1]);
    enc_[W_M3].attachHalfQuad(pins_[W_M3][0], pins_[W_M3][1]);
    enc_[W_M4].attachHalfQuad(pins_[W_M4][0], pins_[W_M4][1]);
    
    for (int i=0; i<W_COUNT; ++i) {
      enc_[i].clearCount();
      last_counts_[i] = 0;
      pos_rad_[i] = 0.0f;
      vel_rad_s_[i] = 0.0f;
      vel_m_s_[i] = 0.0f;
      rpm_[i] = 0.0f;
      total_dist_m_[i] = 0.0f;
    }
    last_ts_ms_ = millis();
  }

  void reset() {
    for (int i=0; i<W_COUNT; ++i) {
      enc_[i].clearCount();
      last_counts_[i] = 0;
      pos_rad_[i] = 0.0f;
      vel_rad_s_[i] = 0.0f;
      vel_m_s_[i] = 0.0f;
      rpm_[i] = 0.0f;
      total_dist_m_[i] = 0.0f;
    }
    last_ts_ms_ = millis();
  }

  void update() {
    uint32_t now = millis();
    float dt = (now - last_ts_ms_) / 1000.0f;
    if (dt <= 0.0f) dt = 1e-3f;
    last_ts_ms_ = now;

    const float two_pi = 2.0f * M_PI;
    const float C = 2.0f * M_PI * WHEEL_RADIUS; // circumference (m)

    for (int i=0; i<W_COUNT; ++i) {
      long cur = enc_[i].getCount() * inv_[i];
      long d = cur - last_counts_[i];
      last_counts_[i] = cur;

      // คำนวณด้วย EFFECTIVE_PPR (รวมเกียร์แล้ว)
      float revolutions = (float)cur / EFFECTIVE_PPR;
      float delta_rev = (float)d / EFFECTIVE_PPR;

      // ระยะทาง (m)
      float delta_dist = delta_rev * C;
      total_dist_m_[i] += delta_dist;
      
      // มุม (rad)
      pos_rad_[i] = revolutions * two_pi;

      // ความเร็วเชิงมุม (rad/s)
      float rev_per_sec = delta_rev / dt;
      vel_rad_s_[i] = rev_per_sec * two_pi;

      // ความเร็วเชิงเส้น (m/s)
      vel_m_s_[i] = delta_dist / dt;

      // RPM (สำหรับ debug)
      rpm_[i] = rev_per_sec * 60.0f;
    }
  }

  float getPosRad(int i) const { return pos_rad_[i]; }
  float getVelRadS(int i) const { return vel_rad_s_[i]; }
  float getVelMS(int i) const { return vel_m_s_[i]; }
  float getRPM(int i) const { return rpm_[i]; }
  float getTotalDistM(int i) const { return total_dist_m_[i]; }
  long getCount(int i) const { return last_counts_[i]; }
  
  void setInversion(int i, int inv) { inv_[i] = inv; }

private:
  ESP32Encoder enc_[W_COUNT];
  int pins_[W_COUNT][2];
  int inv_[W_COUNT];
  
  long last_counts_[W_COUNT];
  float pos_rad_[W_COUNT];
  float vel_rad_s_[W_COUNT];
  float vel_m_s_[W_COUNT];
  float rpm_[W_COUNT];
  float total_dist_m_[W_COUNT];
  uint32_t last_ts_ms_;
};

// ---------------- Global Objects ----------
QuadEncoderReader encoders;

// ---------------- ROS entities ----------
rcl_publisher_t debug_motor_pub;
std_msgs__msg__Float32MultiArray debug_motor_msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Int16MultiArray encoder_msg;

rcl_publisher_t encoder_vel_pub;      // ⭐ ความเร็ว (m/s)
std_msgs__msg__Float32MultiArray encoder_vel_msg;

rcl_publisher_t encoder_dist_pub;     // ⭐ ระยะทาง (m)
std_msgs__msg__Float32MultiArray encoder_dist_msg;

rcl_publisher_t encoder_rpm_pub;
std_msgs__msg__Float32MultiArray encoder_rpm_msg;

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

// ---------------- Command state ----------
volatile float cmd_vx = 0.0f;
volatile float cmd_wz = 0.0f;
volatile uint32_t last_cmd_ms = 0;

// ---------------- Motor/RPM state -------
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// ---------- Helpers ----------
uint8_t rpm_to_duty(float rpm){
  float a = fabsf(rpm) / MAX_RPM; 
  if(a > 1) a = 1;
  return (uint8_t)(a * 255.0f);
}

float clampf(float v, float lo, float hi){ 
  return v < lo ? lo : (v > hi ? hi : v); 
}

float v_to_rpm(float v_mps){ 
  // v = ω × r  →  ω = v/r (rad/s)
  // RPM = ω × 60 / (2π)
  return (v_mps / WHEEL_RADIUS) * 60.0f / (2.0f * M_PI); 
}

// ---------- Motor control ----------
void setMotor1(float rpm){ 
  digitalWrite(L_DIR1, (rpm>=0)?HIGH:LOW); 
  ledcWrite(PWM_CH_M1, rpm_to_duty(rpm)); 
}
void setMotor2(float rpm){ 
  digitalWrite(R_DIR1, (rpm>=0)?HIGH:LOW); 
  ledcWrite(PWM_CH_M2, rpm_to_duty(rpm)); 
}
void setMotor3(float rpm){ 
  digitalWrite(L_DIR2, (rpm>=0)?HIGH:LOW); 
  ledcWrite(PWM_CH_M3, rpm_to_duty(rpm)); 
}
void setMotor4(float rpm){ 
  digitalWrite(R_DIR2, (rpm>=0)?HIGH:LOW); 
  ledcWrite(PWM_CH_M4, rpm_to_duty(rpm)); 
}

// ---------- Function prototypes ----------
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

  // Motor pins
  pinMode(L_DIR1, OUTPUT); pinMode(L_DIR2, OUTPUT);
  pinMode(R_DIR1, OUTPUT); pinMode(R_DIR2, OUTPUT);
  digitalWrite(L_DIR1, LOW); digitalWrite(L_DIR2, LOW);
  digitalWrite(R_DIR1, LOW); digitalWrite(R_DIR2, LOW);

  // PWM setup
  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_PWM1, PWM_CH_M1);
  ledcAttachPin(R_PWM1, PWM_CH_M2);
  ledcAttachPin(L_PWM2, PWM_CH_M3);
  ledcAttachPin(R_PWM2, PWM_CH_M4);

  // Initialize encoders (GPIO 34-39 ไม่มี internal pullup)
  encoders.begin(false);

  Serial.println("========================================");
  Serial.println("ESP32 Differential Drive Controller");
  Serial.println("Motor: 37mm Geared Motor with Encoder");
  Serial.printf("Encoder PPR: %.0f (raw) × %.0f (gear) = %.0f (effective)\n", 
                ENCODER_PPR, GEAR_RATIO, EFFECTIVE_PPR);
  Serial.printf("Max RPM: %.0f (rated with load)\n", MAX_RPM);
  Serial.printf("Wheel Radius: %.3f m\n", WHEEL_RADIUS);
  Serial.println("========================================");
}

void loop(){
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, 
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;
      
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if(state==WAITING_AGENT) destroyEntities();
      break;
      
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, 
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state==AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
      
    case AGENT_DISCONNECTED:
      destroyEntities(); 
      state = WAITING_AGENT; 
      break;
  }
}

// ======================= Callbacks ================================================
void controlCb(rcl_timer_t*, int64_t){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  float dt = (now - last_ms) / 1000.0f; 
  if(dt<=0) dt = 0.02f;
  last_ms = now;
  
  encoders.update();
  controlStep(dt);
  publishData();
}

void counterCb(rcl_timer_t*, int64_t){
  static int32_t c=0; 
  counter_msg.data = ++c;
  RCSOFTCHECK(rcl_publish(&counter_pub, &counter_msg, NULL));
}

void twistCb(const void *msgin){
  const auto *m = (const geometry_msgs__msg__Twist*)msgin;

  float vx = (float)m->linear.x;
  float wz = (float)m->angular.z;

  if (fabsf(vx) < DEADBAND) vx = 0.0f;
  if (fabsf(wz) < DEADBAND) wz = 0.0f;

  cmd_vx = vx;
  cmd_wz = wz;
  last_cmd_ms = millis();
}

// ======================= Entities ================================================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  // Allocate arrays
  debug_motor_msg.data.capacity = 4; 
  debug_motor_msg.data.size = 4;
  debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));
  
  encoder_msg.data.capacity = 4; 
  encoder_msg.data.size = 4;
  encoder_msg.data.data = (int16_t*)malloc(4*sizeof(int16_t));
  
  encoder_vel_msg.data.capacity = 4;
  encoder_vel_msg.data.size = 4;
  encoder_vel_msg.data.data = (float*)malloc(4*sizeof(float));

  encoder_dist_msg.data.capacity = 4;
  encoder_dist_msg.data.size = 4;
  encoder_dist_msg.data.data = (float*)malloc(4*sizeof(float));

  encoder_rpm_msg.data.capacity = 4;
  encoder_rpm_msg.data.size = 4;
  encoder_rpm_msg.data.data = (float*)malloc(4*sizeof(float));
  
  std_msgs__msg__Int32__init(&counter_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_diff_controller", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
    "/motor_debug/duty"));
    
  RCCHECK(rclc_publisher_init_best_effort(&encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), 
    "/motor_feedback/encoders"));
    
  RCCHECK(rclc_publisher_init_best_effort(&encoder_vel_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
    "/motor_feedback/velocity"));

  RCCHECK(rclc_publisher_init_best_effort(&encoder_dist_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
    "/motor_feedback/distance"));

  RCCHECK(rclc_publisher_init_best_effort(&encoder_rpm_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
    "/motor_feedback/rpm"));
    
  RCCHECK(rclc_publisher_init_best_effort(&counter_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
    "/esp32/debug/counter"));

  // Subscription
  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "/man/cmd_move"));

  // Timers
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCb));
  RCCHECK(rclc_timer_init_default(&counter_timer, &support, RCL_MS_TO_NS(1000), counterCb));

  // Executor
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
  rcl_publisher_fini(&encoder_vel_pub, &node);
  rcl_publisher_fini(&encoder_dist_pub, &node);
  rcl_publisher_fini(&encoder_rpm_pub, &node);
  rcl_publisher_fini(&counter_pub, &node);
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&counter_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if (debug_motor_msg.data.data) free(debug_motor_msg.data.data);
  if (encoder_msg.data.data) free(encoder_msg.data.data);
  if (encoder_vel_msg.data.data) free(encoder_vel_msg.data.data);
  if (encoder_dist_msg.data.data) free(encoder_dist_msg.data.data);
  if (encoder_rpm_msg.data.data) free(encoder_rpm_msg.data.data);
  
  return true;
}

// ======================= Control ================================================
void controlStep(float dt){
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
    tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
  } else {
    float vx = cmd_vx;
    float wz = cmd_wz;

    float v_left  = vx - wz * (TRACK_WIDTH * 0.5f);
    float v_right = vx + wz * (TRACK_WIDTH * 0.5f);

    float rpm_left  = clampf(v_to_rpm(v_left),  -MAX_RPM, MAX_RPM);
    float rpm_right = clampf(v_to_rpm(v_right), -MAX_RPM, MAX_RPM);

    tgt_m1_rpm = rpm_left;
    tgt_m2_rpm = rpm_right;
    tgt_m3_rpm = rpm_left;
    tgt_m4_rpm = rpm_right;
  }

  // Slew-rate limiting (ป้องกัน stall)
  auto slew = [&](float cur, float tgt) -> float {
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
  // ========== Motor duty cycle (0-255) ==========
  debug_motor_msg.data.data[0] = rpm_to_duty(m1_rpm);
  debug_motor_msg.data.data[1] = rpm_to_duty(m2_rpm);
  debug_motor_msg.data.data[2] = rpm_to_duty(m3_rpm);
  debug_motor_msg.data.data[3] = rpm_to_duty(m4_rpm);
  RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

  // ========== Encoder counts (ticks) ==========
  encoder_msg.data.data[0] = (int16_t)(encoders.getCount(W_M1) & 0xFFFF);
  encoder_msg.data.data[1] = (int16_t)(encoders.getCount(W_M2) & 0xFFFF);
  encoder_msg.data.data[2] = (int16_t)(encoders.getCount(W_M3) & 0xFFFF);
  encoder_msg.data.data[3] = (int16_t)(encoders.getCount(W_M4) & 0xFFFF);
  RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

  // ========== ⭐ Velocity (m/s) ==========
  encoder_vel_msg.data.data[0] = encoders.getVelMS(W_M1);
  encoder_vel_msg.data.data[1] = encoders.getVelMS(W_M2);
  encoder_vel_msg.data.data[2] = encoders.getVelMS(W_M3);
  encoder_vel_msg.data.data[3] = encoders.getVelMS(W_M4);
  RCSOFTCHECK(rcl_publish(&encoder_vel_pub, &encoder_vel_msg, NULL));

  // ========== ⭐ Distance (m) ==========
  encoder_dist_msg.data.data[0] = encoders.getTotalDistM(W_M1);
  encoder_dist_msg.data.data[1] = encoders.getTotalDistM(W_M2);
  encoder_dist_msg.data.data[2] = encoders.getTotalDistM(W_M3);
  encoder_dist_msg.data.data[3] = encoders.getTotalDistM(W_M4);
  RCSOFTCHECK(rcl_publish(&encoder_dist_pub, &encoder_dist_msg, NULL));

  // ========== RPM (for debugging) ==========
  encoder_rpm_msg.data.data[0] = encoders.getRPM(W_M1);
  encoder_rpm_msg.data.data[1] = encoders.getRPM(W_M2);
  encoder_rpm_msg.data.data[2] = encoders.getRPM(W_M3);
  encoder_rpm_msg.data.data[3] = encoders.getRPM(W_M4);
  RCSOFTCHECK(rcl_publish(&encoder_rpm_pub, &encoder_rpm_msg, NULL));
}

// ======================= Time & Error ===========================================
void syncTime(){
  RCCHECK(rmw_uros_sync_session(10));
}

void rclErrorLoop(){
  const int LED_PIN = 2;
  pinMode(LED_PIN, OUTPUT);
  while(true){ 
    digitalWrite(LED_PIN, HIGH); 
    delay(100); 
    digitalWrite(LED_PIN, LOW); 
    delay(100); 
  }
}