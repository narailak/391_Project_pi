#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// -------- Config --------
#define SERVO_PIN     21       // เปลี่ยนเป็นขาอื่นที่รองรับ PWM ได้
#define SERVO_MIN_US  500      // pulse min
#define SERVO_MAX_US  2500     // pulse max

// -------- Helpers / Macros --------
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

// -------- Globals --------
Servo servo;

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
static ConnState state;

static int16_t   last_angle = -1;
static uint32_t  last_hb_ms = 0;

// -------- Servo control --------
static inline int angle_to_us(int angle){
  // map: 0 -> SERVO_MIN_US, 180 -> SERVO_MAX_US (เชิงเส้น)
  long span = (long)SERVO_MAX_US - (long)SERVO_MIN_US;
  long us   = (long)SERVO_MIN_US + (long)angle * span / 180L;
  if(us < SERVO_MIN_US) us = SERVO_MIN_US;
  if(us > SERVO_MAX_US) us = SERVO_MAX_US;
  return (int)us;
}

static inline void move_servo_angle(int angle){
  servo.writeMicroseconds(angle_to_us(angle));
}

// -------- Feedback --------
static inline void publish_fb(int16_t angle){
  fb_msg.data = angle;                 // ส่งกลับเฉพาะ "ตำแหน่งที่สั่ง"
  RCSOFTCHECK(rcl_publish(&pub_fb, &fb_msg, NULL));
}

// -------- Callback --------
static void cmd_cb(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int angle = (int)m->data;

  // Clamp 0..180
  if(angle < 0)   angle = 0;
  if(angle > 180) angle = 180;

  move_servo_angle(angle);
  last_angle = (int16_t)angle;
  publish_fb(last_angle);
}

// -------- micro-ROS Entities --------
static bool createEntities(){
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  // ROS_DOMAIN_ID = 96

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_gripper_node", "", &support));

  // Subscriber: /man/cmd_gripper (std_msgs/Int16)
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_gripper"));

  // Publisher (best-effort): /man/cmd_gripper/rpm (std_msgs/Int16)
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/man/cmd_gripper/rpm"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_cb, ON_NEW_DATA));

  // Initial: set to 0°
  last_angle = 0;
  move_servo_angle(last_angle);
  publish_fb(last_angle);

  return true;
}

static bool destroyEntities(){
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd, &node);
  rcl_publisher_fini(&pub_fb, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

// -------- Arduino setup/loop --------
void setup(){
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // Servo init
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  move_servo_angle(0);  // เริ่มที่ 0°
}

void loop(){
  switch(state){
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if(state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      // ตรวจ agent เป็นระยะ
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if(state == AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // heartbeat ทุก ~300ms (ส่งตำแหน่งล่าสุดกลับ)
        uint32_t now = millis();
        if(last_angle != -1 && (now - last_hb_ms) >= 300){
          publish_fb(last_angle);
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
