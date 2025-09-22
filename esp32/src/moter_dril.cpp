#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// ================== Config: TB6612FNG (ช่อง A) ==================
#define TB_AIN1   25    // เปลี่ยนตามการต่อจริง
#define TB_AIN2   26    // เปลี่ยนตามการต่อจริง
#define TB_PWMA   27    // PWM pin (ESP32 LEDC)
#define TB_STBY   14    // STBY (HIGH = enable). ถ้าไม่ใช้ สามารถผูกกับ VCC ผ่าน R ได้

// PWM config (LEDC)
#define PWM_FREQ      20000      // 20 kHz เงียบหู
#define PWM_RES_BITS  8          // 8-bit (0..255)
#define PWM_CHANNEL   0

// micro-ROS settings
#define ROS_DOMAIN_ID 96
#define SUB_TOPIC     "/man/moter_dril"
#define PUB_TOPIC     "/man/moter_dril/rpm"

// ================== Helpers / Macros ==================
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state;

// ================== micro-ROS globals ==================
rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

rcl_subscription_t   sub_cmd;
std_msgs__msg__Int16 cmd_msg;

rcl_publisher_t      pub_fb;
std_msgs__msg__Int16 fb_msg;

// ================== State ==================
static int16_t  last_cmd_percent = 0;   // 0..100 ที่ “สั่ง”
static uint32_t last_hb_ms = 0;

// ================== Motor utils ==================
static inline void motor_coast(){
  // ปล่อยล้อ (coast)
  ledcWrite(PWM_CHANNEL, 0);
  digitalWrite(TB_AIN1, LOW);
  digitalWrite(TB_AIN2, LOW);
}

static inline void motor_forward(uint8_t pwm){
  // หมุนทิศทาง "ไปข้างหน้า"
  digitalWrite(TB_AIN1, HIGH);
  digitalWrite(TB_AIN2, LOW);
  ledcWrite(PWM_CHANNEL, pwm);
}

// (ถ้าต้องการ reverse ภายหลัง ให้ใช้แบบนี้)
// static inline void motor_reverse(uint8_t pwm){
//   digitalWrite(TB_AIN1, LOW);
//   digitalWrite(TB_AIN2, HIGH);
//   ledcWrite(PWM_CHANNEL, pwm);
// }

// ================== Feedback ==================
static inline void publish_fb(int16_t percent){
  fb_msg.data = percent;                 // ส่งกลับค่าที่ “สั่ง” 0..100
  RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
}

// ================== Callback ==================
static void cmd_cb(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int val = (int)m->data;

  // ปรับขอบเขต
  if(val <= 0){
    motor_coast();
    last_cmd_percent = 0;
    publish_fb(last_cmd_percent);
    return;
  }
  if(val > 100) val = 100;  // clamp

  // map 1..100 -> 0..255 (เริ่มจาก 0 เพื่อสั่งเบาได้)
  uint8_t duty = (uint8_t) map(val, 1, 100, 0, 255);

  // เดินหน้าเสมอ
  motor_forward(duty);

  last_cmd_percent = (int16_t)val;
  publish_fb(last_cmd_percent);
}

// ================== micro-ROS Entities ==================
static bool createEntities(){
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_tb6612_node", "", &support));

  // Subscriber: /man/moter_dril (std_msgs/Int16)
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    SUB_TOPIC));

  // Publisher (best-effort): /man/moter_dril/rpm (std_msgs/Int16)
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    PUB_TOPIC));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_cb, ON_NEW_DATA));

  // Initial feedback
  last_cmd_percent = 0;
  publish_fb(last_cmd_percent);

  return true;
}

static bool destroyEntities(){
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  (void)rcl_subscription_fini(&sub_cmd, &node);
  (void)rcl_publisher_fini(&pub_fb, &node);
  rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

// ================== Arduino setup/loop ==================
void setup(){
  // Serial + micro-ROS transport
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // TB6612FNG pins
  pinMode(TB_AIN1, OUTPUT);
  pinMode(TB_AIN2, OUTPUT);
  pinMode(TB_PWMA, OUTPUT);
  pinMode(TB_STBY, OUTPUT);
  digitalWrite(TB_STBY, HIGH);     // enable driver
  motor_coast();                   // เริ่มที่หยุด

  // PWM setup (LEDC)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(TB_PWMA, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  state = WAITING_AGENT;
}

void loop(){
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if(state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state == AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // heartbeat: ส่งค่าล่าสุดกลับทุก ~300ms
        uint32_t now = millis();
        if(now - last_hb_ms >= 300){
          publish_fb(last_cmd_percent);
          last_hb_ms = now;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}
