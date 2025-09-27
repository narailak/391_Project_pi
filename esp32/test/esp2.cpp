// ================= ESP32 + micro-ROS: 3x Servo + Stepper(TB6600) + Dual Limit + TB6612FNG (One Sketch) =================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// ========================== Shared Helpers / Macros ==========================
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state = WAITING_AGENT;

// ========================== micro-ROS globals ==========================
rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

// ========================== Servo Bundle ==========================
struct ServoChan {
  Servo   servo;
  int     pin;
  int     min_us;
  int     max_us;

  const char* sub_topic;
  const char* pub_topic;

  rcl_subscription_t     sub;
  std_msgs__msg__Int16   sub_msg;
  rcl_publisher_t        pub;
  std_msgs__msg__Int16   pub_msg;

  int16_t  last_angle = -1;
  uint32_t last_hb_ms = 0;

  enum Mode { ANY_0_180, ONLY_0_90, ONLY_0_180 } mode;
};

static ServoChan make_gripper(){
  ServoChan ch{};
  ch.pin=16; ch.min_us=500; ch.max_us=2500;
  ch.sub_topic="/man/cmd_gripper"; ch.pub_topic="/man/cmd_gripper/rpm";
  ch.mode=ServoChan::ANY_0_180; return ch;
}
static ServoChan make_dril_servo(){
  ServoChan ch{};
  ch.pin=23; ch.min_us=500; ch.max_us=2500;
  ch.sub_topic="/man/cmd_servo_dril"; ch.pub_topic="/man/cmd_servo_dril/rpm";
  ch.mode=ServoChan::ONLY_0_90; return ch;
}
static ServoChan make_sw180(){
  ServoChan ch{};
  ch.pin=17; ch.min_us=500; ch.max_us=2500;
  ch.sub_topic="/man/cmd_servo_switch180"; ch.pub_topic="/man/cmd_servo_switch180/rpm";
  ch.mode=ServoChan::ONLY_0_180; return ch;
}

static ServoChan CH_GRIPPER = make_gripper();
static ServoChan CH_DRIL_SERVO = make_dril_servo();
static ServoChan CH_SW180 = make_sw180();

static inline int angle_to_us(const ServoChan& ch, int angle){
  long span = (long)ch.max_us - (long)ch.min_us;
  long us   = (long)ch.min_us + (long)angle * span / 180L;
  if(us < ch.min_us) us = ch.min_us;
  if(us > ch.max_us) us = ch.max_us;
  return (int)us;
}
static inline void move_servo_angle(ServoChan& ch, int angle){
  ch.servo.writeMicroseconds(angle_to_us(ch, angle));
}
static inline void publish_fb_servo(ServoChan& ch, int16_t angle){
  ch.pub_msg.data = angle; RCSOFTCHECK(rcl_publish(&ch.pub, &ch.pub_msg, NULL));
}
static inline bool accept_angle(const ServoChan& ch, int angle_in, int& angle_out){
  switch(ch.mode){
    case ServoChan::ANY_0_180: angle_out = constrain(angle_in, 0, 180); return true;
    case ServoChan::ONLY_0_90: if(angle_in==0||angle_in==90){angle_out=angle_in; return true;} return false;
    case ServoChan::ONLY_0_180: if(angle_in==0||angle_in==180){angle_out=angle_in; return true;} return false;
  } return false;
}
static void sub_cb_gripper(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin; int a;
  if(accept_angle(CH_GRIPPER,(int)m->data,a)){ move_servo_angle(CH_GRIPPER,a); CH_GRIPPER.last_angle=a; publish_fb_servo(CH_GRIPPER,a); }
}
static void sub_cb_dril_servo(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin; int a;
  if(accept_angle(CH_DRIL_SERVO,(int)m->data,a)){ move_servo_angle(CH_DRIL_SERVO,a); CH_DRIL_SERVO.last_angle=a; publish_fb_servo(CH_DRIL_SERVO,a); }
}
static void sub_cb_sw180(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin; int a;
  if(accept_angle(CH_SW180,(int)m->data,a)){ move_servo_angle(CH_SW180,a); CH_SW180.last_angle=a; publish_fb_servo(CH_SW180,a); }
}

// ========================== Stepper (TB6600) ==========================
static const int PIN_PUL = 25;
static const int PIN_DIR = 26;
static const int PIN_ENA = 27;
static const int LIMIT1_PIN = 18; // ซ้าย/-1
static const int LIMIT2_PIN = 13; // ขวา/+1
static volatile uint32_t HALF_PERIOD_US = 20;

rcl_subscription_t sub_cmd_linear;
std_msgs__msg__Int16   cmd_msg_linear;
rcl_publisher_t  pub_fb_linear;
std_msgs__msg__Int16   fb_msg_linear;

static volatile int8_t RUN_DIR = 0; // +1/-1/0
static volatile bool   pul_high = false;
hw_timer_t* tmr = nullptr;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool   limit_hit_flag = false;
static volatile int8_t limit_side     = 0;

static inline bool limit1_pressed() { return digitalRead(LIMIT1_PIN)==LOW; }
static inline bool limit2_pressed() { return digitalRead(LIMIT2_PIN)==LOW; }
static inline void enable_driver(bool en_low=true){ digitalWrite(PIN_ENA, en_low?LOW:HIGH); }
static inline void set_direction(int8_t dir){ digitalWrite(PIN_DIR, (dir==1)?LOW:HIGH); }
static inline bool can_run_dir(int8_t dir){
  if(dir==+1 && limit2_pressed()) return false;
  if(dir==-1 && limit1_pressed()) return false;
  return true;
}
static inline void publish_echo_linear(int16_t v){
  fb_msg_linear.data=v; RCSOFTCHECK(rcl_publish(&pub_fb_linear,&fb_msg_linear,NULL));
}
static inline void start_run(int8_t dir){
  if(dir!=1 && dir!=-1) return;
  if(!can_run_dir(dir)){ fb_msg_linear.data=(dir==+1)?+2:-2; RCSOFTCHECK(rcl_publish(&pub_fb_linear,&fb_msg_linear,NULL)); return; }
  set_direction(dir); enable_driver(true);
  portENTER_CRITICAL_ISR(&spinlock); RUN_DIR=dir; portEXIT_CRITICAL_ISR(&spinlock);
}
static inline void stop_now_core(){
  portENTER_CRITICAL_ISR(&spinlock); RUN_DIR=0; portEXIT_CRITICAL_ISR(&spinlock);
  REG_WRITE(GPIO_OUT_W1TC_REG, (1U<<PIN_PUL)); pul_high=false; enable_driver(false);
}
static inline void stop_now_from_cmd(){ stop_now_core(); }

void IRAM_ATTR onTimer(){
  if(RUN_DIR==0){ if(pul_high){ REG_WRITE(GPIO_OUT_W1TC_REG,(1U<<PIN_PUL)); pul_high=false; } return; }
  if((RUN_DIR==+1 && gpio_get_level((gpio_num_t)LIMIT2_PIN)==0) ||
     (RUN_DIR==-1 && gpio_get_level((gpio_num_t)LIMIT1_PIN)==0)){
    portENTER_CRITICAL_ISR(&spinlock); RUN_DIR=0; portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG,(1U<<PIN_PUL)); pul_high=false; limit_hit_flag=true;
    limit_side = (gpio_get_level((gpio_num_t)LIMIT2_PIN)==0)?+1:-1; return;
  }
  uint32_t mask=(1U<<PIN_PUL);
  if(!pul_high){ REG_WRITE(GPIO_OUT_W1TS_REG,mask); pul_high=true; }
  else{ REG_WRITE(GPIO_OUT_W1TC_REG,mask); pul_high=false; }
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
}
void IRAM_ATTR onLimit1(){
  if(gpio_get_level((gpio_num_t)LIMIT1_PIN)==0){
    portENTER_CRITICAL_ISR(&spinlock); RUN_DIR=0; portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG,(1U<<PIN_PUL)); pul_high=false; limit_hit_flag=true; limit_side=-1;
  }
}
void IRAM_ATTR onLimit2(){
  if(gpio_get_level((gpio_num_t)LIMIT2_PIN)==0){
    portENTER_CRITICAL_ISR(&spinlock); RUN_DIR=0; portEXIT_CRITICAL_ISR(&spinlock);
    REG_WRITE(GPIO_OUT_W1TC_REG,(1U<<PIN_PUL)); pul_high=false; limit_hit_flag=true; limit_side=+1;
  }
}
static void cmd_cb_linear(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin; const int16_t cmd=m->data; publish_echo_linear(cmd);
  if(cmd==1){ start_run(-1); Serial.println("[LINEAR] cmd=+1 -> run CCW (continuous)"); }
  else if(cmd==-1){ start_run(+1); Serial.println("[LINEAR] cmd=-1 -> run CW (continuous)"); }
  else if(cmd==0){ stop_now_from_cmd(); Serial.println("[LINEAR] cmd=0 -> STOP"); }
}

// ========================== TB6612FNG (ช่อง A, ดริลมอเตอร์) ==========================
// พิน TB6612 (ปรับตามการต่อจริง)
#define TB_AIN1   19
#define TB_AIN2   21
#define TB_PWMA   22
#define TB_STBY   5

// PWM (LEDC) — ใช้ channel สูง เลี่ยงชนกับ ESP32Servo
#define PWM_FREQ      20000    // 20 kHz
#define PWM_RES_BITS  8        // 0..255
#define PWM_CHANNEL   15       // *** เลือก 15 เพื่อลดโอกาสชนกับ Servo ***

// ROS ดริลมอเตอร์
rcl_subscription_t   sub_cmd_dril_motor;
std_msgs__msg__Int16 cmd_msg_dril_motor;
rcl_publisher_t      pub_fb_dril_motor;
std_msgs__msg__Int16 fb_msg_dril_motor;

// สถานะ/HB
static int16_t  last_cmd_percent = 0;   // 0..100 ที่สั่ง
static uint32_t last_hb_ms_dril = 0;

static inline void motor_coast(){
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(TB_AIN1, LOW);
  digitalWrite(TB_AIN2, LOW);
}
static inline void motor_forward(uint8_t pwm){
  digitalWrite(TB_AIN1, HIGH);
  digitalWrite(TB_AIN2, LOW);
  ledcWrite(PWM_CHANNEL, pwm);
}
// ถ้าต้องการ reverse ในอนาคต:
// static inline void motor_reverse(uint8_t pwm){ digitalWrite(TB_AIN1, LOW); digitalWrite(TB_AIN2, HIGH); ledcWrite(PWM_CHANNEL, pwm); }

static inline void publish_fb_dril_motor(int16_t percent){
  fb_msg_dril_motor.data = percent;
  RCSOFTCHECK(rcl_publish(&pub_fb_dril_motor, &fb_msg_dril_motor, NULL));
}
static void cmd_cb_dril_motor(const void* msgin){
  const auto* m=(const std_msgs__msg__Int16*)msgin;
  int val=(int)m->data;
  if(val<=0){
    motor_coast(); last_cmd_percent=0; publish_fb_dril_motor(last_cmd_percent); return;
  }
  if(val>100) val=100;
  uint8_t duty = (uint8_t) map(val, 1, 100, 0, 255); // 1..100 → 0..255
  motor_forward(duty);
  last_cmd_percent=(int16_t)val;
  publish_fb_dril_motor(last_cmd_percent);
}

// ========================== micro-ROS Entities (All-in-one) ==========================
static bool createEntities(){
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  // ROS_DOMAIN_ID = 96
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_multi_peripheral_node", "", &support));

  // --- Servo pubs ---
  RCCHECK(rclc_publisher_init_best_effort(&CH_GRIPPER.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&CH_DRIL_SERVO.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.pub_topic));
  RCCHECK(rclc_publisher_init_best_effort(&CH_SW180.pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.pub_topic));

  // --- Servo subs ---
  RCCHECK(rclc_subscription_init_default(&CH_GRIPPER.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_GRIPPER.sub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_DRIL_SERVO.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_DRIL_SERVO.sub_topic));
  RCCHECK(rclc_subscription_init_default(&CH_SW180.sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),CH_SW180.sub_topic));

  // --- Stepper sub/pub ---
  RCCHECK(rclc_subscription_init_default(&sub_cmd_linear,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/man/cmd_linear"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_fb_linear,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/man/cmd_linear/fb"));

  // --- Dril motor (TB6612) sub/pub ---
  RCCHECK(rclc_subscription_init_default(&sub_cmd_dril_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/man/moter_dril"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_fb_dril_motor,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Int16),"/man/moter_dril/rpm"));

  // --- Executor: 5 subs (3 servo + 1 linear + 1 dril motor) ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_GRIPPER.sub,&CH_GRIPPER.sub_msg,&sub_cb_gripper,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_DRIL_SERVO.sub,&CH_DRIL_SERVO.sub_msg,&sub_cb_dril_servo,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&CH_SW180.sub,&CH_SW180.sub_msg,&sub_cb_sw180,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_cmd_linear,&cmd_msg_linear,&cmd_cb_linear,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,&sub_cmd_dril_motor,&cmd_msg_dril_motor,&cmd_cb_dril_motor,ON_NEW_DATA));

  // --- Init servo positions & feedback once ---
  CH_GRIPPER.last_angle=0; move_servo_angle(CH_GRIPPER,0); publish_fb_servo(CH_GRIPPER,0);
  CH_DRIL_SERVO.last_angle=0; move_servo_angle(CH_DRIL_SERVO,0); publish_fb_servo(CH_DRIL_SERVO,0);
  CH_SW180.last_angle=0; move_servo_angle(CH_SW180,0); publish_fb_servo(CH_SW180,0);

  // --- Init dril motor feedback ---
  last_cmd_percent=0; publish_fb_dril_motor(last_cmd_percent);

  return true;
}

static bool destroyEntities(){
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  // stepper
  (void)rcl_subscription_fini(&sub_cmd_linear, &node);
  (void)rcl_publisher_fini(&pub_fb_linear, &node);

  // dril motor
  (void)rcl_subscription_fini(&sub_cmd_dril_motor, &node);
  (void)rcl_publisher_fini(&pub_fb_dril_motor, &node);

  // servos
  (void)rcl_subscription_fini(&CH_GRIPPER.sub, &node);
  (void)rcl_subscription_fini(&CH_DRIL_SERVO.sub, &node);
  (void)rcl_subscription_fini(&CH_SW180.sub, &node);

  (void)rcl_publisher_fini(&CH_GRIPPER.pub, &node);
  (void)rcl_publisher_fini(&CH_DRIL_SERVO.pub, &node);
  (void)rcl_publisher_fini(&CH_SW180.pub, &node);

  rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

// ========================== Arduino setup/loop ==========================
void setup(){
  // --- Stepper GPIO ---
  pinMode(PIN_PUL, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_PUL, LOW); digitalWrite(PIN_DIR, LOW); enable_driver(false);

  // --- Limit switches ---
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT1_PIN), onLimit1, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT2_PIN), onLimit2, FALLING);

  // --- Serial & transport ---
  Serial.begin(115200); delay(50);
  set_microros_serial_transports(Serial);
  Serial.println("[INFO] ESP32 Multi (Servo+Stepper+TB6612) Ready");

  // --- Stepper timer ---
  tmr = timerBegin(0, 80, true);         // 1 tick = 1 µs
  timerAttachInterrupt(tmr, &onTimer, true);
  timerAlarmWrite(tmr, HALF_PERIOD_US, true);
  timerAlarmEnable(tmr);

  // --- Attach servos ---
  CH_GRIPPER.servo.attach(CH_GRIPPER.pin, CH_GRIPPER.min_us, CH_GRIPPER.max_us);
  CH_DRIL_SERVO.servo.attach(CH_DRIL_SERVO.pin, CH_DRIL_SERVO.min_us, CH_DRIL_SERVO.max_us);
  CH_SW180.servo.attach(CH_SW180.pin, CH_SW180.min_us, CH_SW180.max_us);

  // --- TB6612 pins & PWM ---
  pinMode(TB_AIN1, OUTPUT);
  pinMode(TB_AIN2, OUTPUT);
  pinMode(TB_PWMA, OUTPUT);
  pinMode(TB_STBY, OUTPUT);
  digitalWrite(TB_STBY, HIGH);   // enable driver
  motor_coast();                 // เริ่มที่หยุด

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES_BITS);   // channel 15
  ledcAttachPin(TB_PWMA, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // --- Move servos to 0° initially ---
  move_servo_angle(CH_GRIPPER, 0);
  move_servo_angle(CH_DRIL_SERVO, 0);
  move_servo_angle(CH_SW180, 0);
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
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state == AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // --- Heartbeats ---
        uint32_t now = millis();
        if(CH_GRIPPER.last_angle != -1 && (now - CH_GRIPPER.last_hb_ms) >= 300){
          publish_fb_servo(CH_GRIPPER, CH_GRIPPER.last_angle);
          CH_GRIPPER.last_hb_ms = now;
        }
        if(CH_DRIL_SERVO.last_angle != -1 && (now - CH_DRIL_SERVO.last_hb_ms) >= 300){
          publish_fb_servo(CH_DRIL_SERVO, CH_DRIL_SERVO.last_angle);
          CH_DRIL_SERVO.last_hb_ms = now;
        }
        if(CH_SW180.last_angle != -1 && (now - CH_SW180.last_hb_ms) >= 300){
          publish_fb_servo(CH_SW180, CH_SW180.last_angle);
          CH_SW180.last_hb_ms = now;
        }
        if((now - last_hb_ms_dril) >= 300){
          publish_fb_dril_motor(last_cmd_percent);
          last_hb_ms_dril = now;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      stop_now_core();   // หยุดสเต็ปเปอร์เพื่อความปลอดภัย
      motor_coast();     // ปล่อยดริลมอเตอร์
      state = WAITING_AGENT;
      break;
  }

  // ---- Post-ISR: handle limit hit ----
  if (limit_hit_flag) {
    limit_hit_flag = false;
    stop_now_core();
    fb_msg_linear.data = (limit_side >= 0) ? +2 : -2; // +2 = ชนขวา, -2 = ชนซ้าย
    RCSOFTCHECK(rcl_publish(&pub_fb_linear, &fb_msg_linear, NULL));
    Serial.printf("[LIMIT] Hit %s -> STOP\n", (limit_side >= 0) ? "RIGHT(+1)" : "LEFT(-1)");
  }
}
