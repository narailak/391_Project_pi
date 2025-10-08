// =========================== main.cpp (ESP32 + micro-ROS)
// - Differential drive (4 DC motors + 4x encoders via ESP32Encoder) via /man/cmd_move (Twist)
// - Stepper TB6600 absolute angle via /man/cmd_tao (Int16 deg 0..359) + fb /man/cmd_tao/fb
// - One micro-ROS node/executor/state machine
// - Safety: stop if agent disconnects / cmd timeout
// - PWM: 20 kHz, LEDC ch0..3
// - ROS_DOMAIN_ID = 96
// - UPDATED: Full Quadrature Mode for 4x resolution
// ===================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>

#include <ESP32Encoder.h>               // ใช้ ESP32Encoder แทน ISR
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

#include "soc/gpio_reg.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------- Utils/Macros ----------
void rclErrorLoop();
#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ rclErrorLoop(); } }
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

// ============================================================================
//                             DC MOTORS + ENCODERS (ESP32Encoder)
// ============================================================================
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

// *** พารามิเตอร์ขับเคลื่อนตามไฟล์ด้านบน ***
const float WHEEL_RADIUS = 0.0325f;         // m (ล้อ 65mm -> รัศมี 32.5mm)
const float TRACK_WIDTH  = 0.300f;          // m
const float MAX_RPM      = 46.0f;           // rpm (มีโหลด)

// (ออปชันสำหรับคำนวณเพิ่มภายหลัง)
const float ENCODER_PPR   = 11.0f;
const float GEAR_RATIO    = 169.0f;
const float EFFECTIVE_PPR = ENCODER_PPR * GEAR_RATIO * 4.0f; // x4 for Full Quad

const float    DEADBAND          = 0.02f;    // m/s or rad/s
const uint32_t CMD_TIMEOUT_MS    = 300;      // drive time-out
const float    SLEW_RPM_PER_SEC  = 100.0f;   // ramp

// ---------------- Encoder Class --------------
enum WheelIndex { W_M1=0, W_M2=1, W_M3=2, W_M4=3, W_COUNT=4 };

class QuadEncoderReader {
public:
  QuadEncoderReader() {
    pins_[W_M1][0] = ENC1_A; pins_[W_M1][1] = ENC1_B;
    pins_[W_M2][0] = ENC2_A; pins_[W_M2][1] = ENC2_B;
    pins_[W_M3][0] = ENC3_A; pins_[W_M3][1] = ENC3_B;
    pins_[W_M4][0] = ENC4_A; pins_[W_M4][1] = ENC4_B;
    inv_[W_M1]=1; inv_[W_M2]=1; inv_[W_M3]=1; inv_[W_M4]=1;
  }
  void begin(bool enable_pullups=false){
    ESP32Encoder::useInternalWeakPullResistors = enable_pullups ? puType::up : puType::none;
    // *** CHANGED: attachFullQuad for 4x resolution ***
    enc_[W_M1].attachFullQuad(pins_[W_M1][0], pins_[W_M1][1]);
    enc_[W_M2].attachFullQuad(pins_[W_M2][0], pins_[W_M2][1]);
    enc_[W_M3].attachFullQuad(pins_[W_M3][0], pins_[W_M3][1]);
    enc_[W_M4].attachFullQuad(pins_[W_M4][0], pins_[W_M4][1]);
    for(int i=0;i<W_COUNT;++i){ enc_[i].clearCount(); last_counts_[i]=0; }
    last_ts_ms_=millis();
  }
  void update(){
    uint32_t now=millis(); float dt=(now-last_ts_ms_)/1000.0f; if(dt<=0) dt=1e-3f; last_ts_ms_=now;
    (void)dt; // ยังไม่ใช้อนุพันธ์
    for(int i=0;i<W_COUNT;++i){
      long cur=enc_[i].getCount()*inv_[i];
      long d=cur-last_counts_[i];
      (void)d;                 // เก็บไว้ใช้ภายหลังหากต้องคำนวณความเร็ว
      last_counts_[i]=cur;
    }
  }
  long getCount(int i) const { return last_counts_[i]; }
  void setInversion(int i, int inv){ inv_[i]=inv; }
private:
  ESP32Encoder enc_[W_COUNT];
  int pins_[W_COUNT][2]; int inv_[W_COUNT];
  long last_counts_[W_COUNT];
  uint32_t last_ts_ms_;
};

QuadEncoderReader encoders;

// ROS entities (DC)
rcl_publisher_t debug_motor_pub;     // Float32MultiArray (duty 4 ล้อ)
std_msgs__msg__Float32MultiArray debug_motor_msg;

rcl_publisher_t encoder_pub;         // Int16MultiArray (wrap)
std_msgs__msg__Int16MultiArray encoder_msg;

rcl_publisher_t counter_pub;         // Int32
std_msgs__msg__Int32 counter_msg;

rcl_subscription_t cmd_sub;          // Twist /man/cmd_move
geometry_msgs__msg__Twist cmd_msg;

rcl_timer_t control_timer, counter_timer;

// Command state
volatile float    cmd_vx = 0.0f;  // m/s
volatile float    cmd_wz = 0.0f;  // rad/s
volatile uint32_t drive_last_cmd_ms = 0;

// Motor/RPM state
float m1_rpm=0, m2_rpm=0, m3_rpm=0, m4_rpm=0;
float tgt_m1_rpm=0, tgt_m2_rpm=0, tgt_m3_rpm=0, tgt_m4_rpm=0;

// Helpers (DC)
uint8_t rpm_to_duty(float rpm){
  float a = fabsf(rpm)/MAX_RPM; if(a>1)a=1;
  return (uint8_t)(a*255.0f);
}
float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
float v_to_rpm(float v_mps){ return (v_mps / WHEEL_RADIUS) * 60.0f / (2.0f*(float)M_PI); }

// Motor out
void setMotor1(float rpm){ digitalWrite(L_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M1, rpm_to_duty(rpm)); }
void setMotor2(float rpm){ digitalWrite(R_DIR1, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M2, rpm_to_duty(rpm)); }
void setMotor3(float rpm){ digitalWrite(L_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M3, rpm_to_duty(rpm)); }
void setMotor4(float rpm){ digitalWrite(R_DIR2, (rpm>=0)?HIGH:LOW); ledcWrite(PWM_CH_M4, rpm_to_duty(rpm)); }

// ============================================================================
//                                STEPPER (TB6600)  (ตรรกะเดิม)
// ============================================================================
static const int  PIN_PUL = 13;
static const int  PIN_DIR = 18;
static const int  PIN_ENA = 15;
static const bool ENA_ACTIVE_LOW = true; // TB6600: LOW=Enable

// Stepper config
static const int   BASE_STEPS_PER_REV = 200;   // 1.8°
static const int   MICROSTEP          = 16;    // DIP on TB6600
static const float GEAR_RATIO_STEPPER = 1.0f;
static const long  STEPS_PER_REV      = (long)(BASE_STEPS_PER_REV * MICROSTEP * GEAR_RATIO_STEPPER);

// Speed (ครึ่งคาบพัลส์ us) => f ≈ 1/(2*HALF_PERIOD_US) 
static volatile uint32_t HALF_PERIOD_US = 300; // มากขึ้นทำให้ช้าลง 

// Behavior
static const bool HOLD_TORQUE = true;  // true=ค้างแรงบิดเมื่อถึงเป้า

// Filtering
static const uint32_t STP_CMD_DEBOUNCE_MS            = 120;
static const int      STP_CMD_MIN_DELTA_DEG          = 2;
static const int      STP_CMD_DEADBAND_TO_TARGET_DEG = 1;

// ROS entities (Stepper)
rcl_subscription_t   stp_sub_cmd;       // /man/cmd_tao (Int16)
std_msgs__msg__Int16 stp_cmd_msg;

rcl_publisher_t      stp_pub_fb;        // /man/cmd_tao/fb (Int16)
std_msgs__msg__Int16 stp_fb_msg;

// Stepper run/position state
static volatile int8_t  STP_RUN_DIR = 0;               // +1/-1/0
static volatile bool    stp_pul_high = false;
static volatile long    stp_steps_remaining = 0;
static volatile long    stp_current_steps   = 0;       // absolute counter
static volatile long    stp_target_steps_abs = LONG_MIN;

static uint32_t         stp_last_cmd_ms = 0;           // debounce time
static int              stp_last_cmd_deg_processed = -999;

hw_timer_t* stp_tmr = nullptr;
portMUX_TYPE stp_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Helpers (Stepper) —— ตรรกะเดิม
static inline void stp_enable_driver(bool enable){
  if (ENA_ACTIVE_LOW) digitalWrite(PIN_ENA, enable ? LOW : HIGH);
  else                digitalWrite(PIN_ENA, enable ? HIGH : LOW);
}
static inline void stp_set_direction(int8_t dir){ digitalWrite(PIN_DIR, (dir == 1) ? LOW : HIGH); }
static inline int stp_normalize_deg(int d){ int x=d%360; if(x<0)x+=360; return x; }
static inline int16_t stp_angle_int_from_steps(long s){
  long steps = s % STEPS_PER_REV; if(steps<0) steps+=STEPS_PER_REV;
  float degf = ((float)steps * 360.0f) / (float)STEPS_PER_REV;
  long a = lroundf(degf); if(a>=360)a-=360; if(a<0)a+=360; return (int16_t)a;
}
static inline void stp_publish_angle_now(){
  stp_fb_msg.data = stp_angle_int_from_steps(stp_current_steps);
  RCSOFTCHECK(rcl_publish(&stp_pub_fb, &stp_fb_msg, NULL));
}
static inline long stp_steps_from_degrees_int(int deg){
  deg = stp_normalize_deg(deg);
  long num = (long)deg * (long)STEPS_PER_REV;
  long s   = num / 360; // floor
  if (s >= STEPS_PER_REV) s -= STEPS_PER_REV;
  if (s < 0)              s += STEPS_PER_REV;
  return s;
}
static inline long stp_shortest_delta(long current, long target_mod){
  long cur_mod = current % STEPS_PER_REV; if (cur_mod < 0) cur_mod += STEPS_PER_REV;
  long diff = target_mod - cur_mod;
  if (diff >  (STEPS_PER_REV/2)) diff -= STEPS_PER_REV;
  if (diff < -(STEPS_PER_REV/2)) diff += STEPS_PER_REV;
  return diff;
}
static inline long stp_get_current_steps_atomic(){ portENTER_CRITICAL_ISR(&stp_spinlock); long s=stp_current_steps; portEXIT_CRITICAL_ISR(&stp_spinlock); return s; }
static inline void stp_start_move_steps(long delta_steps){
  if (delta_steps == 0){
    portENTER_CRITICAL_ISR(&stp_spinlock); STP_RUN_DIR=0; stp_steps_remaining=0; portEXIT_CRITICAL_ISR(&stp_spinlock);
    if (!HOLD_TORQUE) stp_enable_driver(false);
    return;
  }
  int8_t dir = (delta_steps > 0) ? 1 : -1; long todo = labs(delta_steps);
  stp_set_direction(dir); stp_enable_driver(true);
  portENTER_CRITICAL_ISR(&stp_spinlock); STP_RUN_DIR=dir; stp_steps_remaining=todo; portEXIT_CRITICAL_ISR(&stp_spinlock);
}
static inline int stp_circular_abs_deg_diff(int a, int b){
  int da = abs(stp_normalize_deg(a) - stp_normalize_deg(b)); return (da > 180) ? (360 - da) : da;
}
static void stp_go_to_degree_filtered(int deg_in){
  const uint32_t now = millis(); const int d = stp_normalize_deg(deg_in);
  if (now - stp_last_cmd_ms < STP_CMD_DEBOUNCE_MS) return;
  if (stp_last_cmd_deg_processed != -999){
    if (stp_circular_abs_deg_diff(d, stp_last_cmd_deg_processed) < STP_CMD_MIN_DELTA_DEG) return;
  }
  if (stp_target_steps_abs != LONG_MIN){
    int target_deg_now = stp_angle_int_from_steps(stp_target_steps_abs);
    if (stp_circular_abs_deg_diff(d, target_deg_now) <= STP_CMD_DEADBAND_TO_TARGET_DEG){
      stp_last_cmd_deg_processed = target_deg_now; stp_last_cmd_ms = now; return;
    }
  }
  long target_abs = stp_steps_from_degrees_int(d);
  long cur        = stp_get_current_steps_atomic();
  if (stp_target_steps_abs == target_abs && stp_steps_remaining > 0){
    stp_last_cmd_deg_processed = d; stp_last_cmd_ms = now; return;
  }
  long delta_now = stp_shortest_delta(cur, target_abs);
  if (delta_now == 0){ stp_target_steps_abs = target_abs; stp_last_cmd_deg_processed=d; stp_last_cmd_ms=now; return; }
  stp_target_steps_abs = target_abs; stp_start_move_steps(delta_now);
  stp_last_cmd_deg_processed = d; stp_last_cmd_ms = now;
}
void IRAM_ATTR stp_onTimer(){
  if (STP_RUN_DIR == 0 || stp_steps_remaining <= 0){
    if (stp_pul_high){ REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); stp_pul_high=false; }
    return;
  }
  uint32_t mask = (1U << PIN_PUL);
  if (!stp_pul_high){
    REG_WRITE(GPIO_OUT_W1TS_REG, mask); stp_pul_high = true;
  } else {
    REG_WRITE(GPIO_OUT_W1TC_REG, mask); stp_pul_high = false;
    stp_steps_remaining--;
    if (STP_RUN_DIR == 1) stp_current_steps++; else if (STP_RUN_DIR == -1) stp_current_steps--;
    if (stp_steps_remaining <= 0){
      STP_RUN_DIR = 0;
      if (stp_target_steps_abs != LONG_MIN) stp_current_steps = stp_target_steps_abs;
      if (!HOLD_TORQUE) stp_enable_driver(false);
    }
  }
  timerAlarmWrite(stp_tmr, HALF_PERIOD_US, true);
}
static void stp_cmd_cb(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  const int target_deg = m->data;
  stp_go_to_degree_filtered(target_deg);
  stp_publish_angle_now();
}

// ============================================================================
//                          micro-ROS common entities
// ============================================================================
rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

// Protos
void syncTime();
bool createEntities();
bool destroyEntities();
void controlStep(float dt);
void publishData();
void controlCb(rcl_timer_t*, int64_t);
void counterCb(rcl_timer_t*, int64_t);
void twistCb(const void *msgin);

// ======================= Control (DC) ==========================================
void controlStep(float dt){
  if (millis() - drive_last_cmd_ms > CMD_TIMEOUT_MS){
    tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
  } else {
    float vx = cmd_vx;   // m/s
    float wz = cmd_wz;   // rad/s
    float v_left  = vx - wz * (TRACK_WIDTH * 0.5f);
    float v_right = vx + wz * (TRACK_WIDTH * 0.5f);
    float rpm_left  = clampf(v_to_rpm(v_left),  -MAX_RPM, MAX_RPM);
    float rpm_right = clampf(v_to_rpm(v_right), -MAX_RPM, MAX_RPM);
    tgt_m1_rpm = rpm_left;  tgt_m3_rpm = rpm_left;
    tgt_m2_rpm = rpm_right; tgt_m4_rpm = rpm_right;
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
  // debug duty (0..255)
  debug_motor_msg.data.data[0] = rpm_to_duty(m1_rpm);
  debug_motor_msg.data.data[1] = rpm_to_duty(m2_rpm);
  debug_motor_msg.data.data[2] = rpm_to_duty(m3_rpm);
  debug_motor_msg.data.data[3] = rpm_to_duty(m4_rpm);
  RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

  // encoders (wrap int16) จาก ESP32Encoder
  encoder_msg.data.data[0] = (int16_t)(encoders.getCount(W_M1) & 0xFFFF);
  encoder_msg.data.data[1] = (int16_t)(encoders.getCount(W_M2) & 0xFFFF);
  encoder_msg.data.data[2] = (int16_t)(encoders.getCount(W_M3) & 0xFFFF);
  encoder_msg.data.data[3] = (int16_t)(encoders.getCount(W_M4) & 0xFFFF);
  RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
}

void controlCb(rcl_timer_t*, int64_t){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  float dt = (now - last_ms) / 1000.0f; if(dt<=0) dt = 0.02f;
  last_ms = now;

  // update encoders แบบไฟล์ด้านบน
  encoders.update();

  controlStep(dt);
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
  if (fabsf(vx) < DEADBAND) vx = 0.0f;
  if (fabsf(wz) < DEADBAND) wz = 0.0f;
  cmd_vx = vx; cmd_wz = wz;
  drive_last_cmd_ms = millis();
}

// ======================= Entities =============================================
bool createEntities(){
  allocator = rcl_get_default_allocator();

  // Init dynamic arrays
  debug_motor_msg.data.capacity = 4; debug_motor_msg.data.size = 4;
  debug_motor_msg.data.data = (float*)malloc(4*sizeof(float));
  encoder_msg.data.capacity = 4; encoder_msg.data.size = 4;
  encoder_msg.data.data = (int16_t*)malloc(4*sizeof(int16_t));
  std_msgs__msg__Int32__init(&counter_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));   // ROS_DOMAIN_ID=96

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_best_effort(&debug_motor_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motor_debug/duty"));
  RCCHECK(rclc_publisher_init_best_effort(&encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "/motor_feedback/encoders"));
  RCCHECK(rclc_publisher_init_best_effort(&counter_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/debug/counter"));
  RCCHECK(rclc_publisher_init_best_effort(&stp_pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/man/cmd_tao/fb"));

  // Subscriptions
  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/man/cmd_move"));
  RCCHECK(rclc_subscription_init_default(&stp_sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/man/cmd_tao"));

  // Timers (DC)
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(20), controlCb));    // 50 Hz
  RCCHECK(rclc_timer_init_default(&counter_timer, &support, RCL_MS_TO_NS(1000), counterCb));

  // Executor (2 subs + 2 timers)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &twistCb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &stp_sub_cmd, &stp_cmd_msg, &stp_cmd_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &counter_timer));

  syncTime();
  drive_last_cmd_ms = millis();
  return true;
}

bool destroyEntities(){
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&debug_motor_pub, &node);
  rcl_publisher_fini(&encoder_pub, &node);
  rcl_publisher_fini(&counter_pub, &node);
  rcl_publisher_fini(&stp_pub_fb, &node);

  rcl_subscription_fini(&cmd_sub, &node);
  rcl_subscription_fini(&stp_sub_cmd, &node);

  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&counter_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if (debug_motor_msg.data.data) free(debug_motor_msg.data.data);
  if (encoder_msg.data.data) free(encoder_msg.data.data);
  return true;
}

// ======================= Time & Error =========================================
void syncTime(){ RCCHECK(rmw_uros_sync_session(10)); }
void rclErrorLoop(){
  const int LED_PIN=2;
  pinMode(LED_PIN,OUTPUT);
  while(true){ digitalWrite(LED_PIN,HIGH); delay(100); digitalWrite(LED_PIN,LOW); delay(100); }
}

// ======================= Arduino setup/loop ===================================
static inline void smartPinMode(int pin){
  if (pin>=34 && pin<=39) pinMode(pin, INPUT);
  else                    pinMode(pin, INPUT_PULLUP);
}

void setup(){
  // Serial + micro-ROS transport
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // DC pins
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

  // Encoder inputs (ไม่ใช้ ISR)
  smartPinMode(ENC1_A); smartPinMode(ENC1_B);
  smartPinMode(ENC2_A); smartPinMode(ENC2_B);
  smartPinMode(ENC3_A); smartPinMode(ENC3_B);
  smartPinMode(ENC4_A); smartPinMode(ENC4_B);

  // เริ่ม ESP32Encoder with Full Quadrature Mode
  encoders.begin(false);   // GPIO34-39 ไม่มี internal pull-up

  // Stepper pins & timer (ตรรกะเดิม)
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  REG_WRITE(GPIO_OUT_W1TC_REG, (1U << PIN_PUL)); // PUL=LOW
  digitalWrite(PIN_DIR, LOW);
  stp_enable_driver(false); // เริ่มต้นปลด ENA (เปิดเมื่อมีคำสั่ง)

  // Timer for stepper: 80 MHz / 80 = 1 MHz → 1 tick = 1 µs
  stp_tmr = timerBegin(0, 80, true);
  timerAttachInterrupt(stp_tmr, &stp_onTimer, true);
  timerAlarmWrite(stp_tmr, HALF_PERIOD_US, true);
  timerAlarmEnable(stp_tmr);

  Serial.println("[INFO] ESP32 Robot Node: Diff(FullQuad 4x)+Stepper(TAO) Ready");
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
      if(state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1500,
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state == AGENT_CONNECTED){
        // spin non-blocking so timers/stepper ISR run smoothly
        rclc_executor_spin_some(&executor, 0);

        // Stepper feedback every ~100 ms
        EXECUTE_EVERY_N_MS(100, stp_publish_angle_now(););
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();

      // Safety: stop DC motors
      tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
      setMotor1(0); setMotor2(0); setMotor3(0); setMotor4(0);

      // Safety: stop stepper & disable
      portENTER_CRITICAL_ISR(&stp_spinlock);
      STP_RUN_DIR = 0;
      stp_steps_remaining = 0;
      portEXIT_CRITICAL_ISR(&stp_spinlock);
      stp_enable_driver(false);

      Serial.println("[WARN] Agent disconnected -> STOP");
      state = WAITING_AGENT;
      break;
  }
}