#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

// ========== Helpers / Macros ==========
#define RCCHECK(fn) do { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ while(1){ delay(100); } } } while(0)
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) \
  do{ static int64_t t__=-1; if(t__==-1)t__=uxr_millis(); \
      if(uxr_millis()-t__>(MS)){ X; t__=uxr_millis(); } }while(0)

enum ConnState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static ConnState state;

// ========== micro-ROS globals ==========
rclc_executor_t   executor;
rclc_support_t    support;
rcl_allocator_t   allocator;
rcl_node_t        node;
rcl_init_options_t init_options;

// ========== One-channel bundle ==========
struct ServoChan {
  // Hardware
  Servo   servo;
  int     pin;
  int     min_us;     // ไม่ใช้ร่วมกับตัวอื่น
  int     max_us;     // ไม่ใช้ร่วมกับตัวอื่น

  // Topics
  const char* sub_topic;
  const char* pub_topic;

  // ROS entities
  rcl_subscription_t     sub;
  std_msgs__msg__Int16   sub_msg;
  rcl_publisher_t        pub;
  std_msgs__msg__Int16   pub_msg;

  // State
  int16_t  last_angle = -1;
  uint32_t last_hb_ms = 0;

  // Policy (กฎค่าเข้า)
  enum Mode { ANY_0_180, ONLY_0_90, ONLY_0_180 } mode;
};

// ========== Factory helpers ==========
static ServoChan make_gripper(){
  ServoChan ch{};                                  /* zero-init ทั้งก้อน */
  ch.pin       = 16;                               /* pin */
  ch.min_us    = 500;                              /* min_us */
  ch.max_us    = 2500;                             /* max_us */
  ch.sub_topic = "/man/cmd_gripper";               /* sub_topic */
  ch.pub_topic = "/man/cmd_gripper/rpm";           /* pub_topic */
  ch.mode      = ServoChan::ANY_0_180;             /* mode */
  ch.last_angle = -1; ch.last_hb_ms = 0;           /* state init */
  return ch;
}

// (เดิม make_dig → เปลี่ยนเป็น make_dril)
static ServoChan make_dril(){
  ServoChan ch{};
  ch.pin       = 23;                               /* pin */
  ch.min_us    = 500;                              /* min_us */
  ch.max_us    = 2500;                             /* max_us */
  ch.sub_topic = "/man/cmd_servo_dril";            /* sub_topic */
  ch.pub_topic = "/man/cmd_servo_dril/rpm";        /* pub_topic */
  ch.mode      = ServoChan::ONLY_0_90;             /* mode: รับเฉพาะ 0/90 */
  ch.last_angle = -1; ch.last_hb_ms = 0;
  return ch;
}

static ServoChan make_sw180(){
  ServoChan ch{};
  ch.pin       = 17;                               /* pin */
  ch.min_us    = 500;                              /* min_us */
  ch.max_us    = 2500;                             /* max_us */
  ch.sub_topic = "/man/cmd_servo_switch180";       /* sub_topic */
  ch.pub_topic = "/man/cmd_servo_switch180/rpm";   /* pub_topic */
  ch.mode      = ServoChan::ONLY_0_180;            /* mode: รับเฉพาะ 0/180 */
  ch.last_angle = -1; ch.last_hb_ms = 0;
  return ch;
}

// ========== Three channels (config แยกเต็มที่) ==========
static ServoChan CH_GRIPPER = make_gripper();
static ServoChan CH_DRIL    = make_dril();   // เดิม CH_DIG → CH_DRIL
static ServoChan CH_SW180   = make_sw180();

// ========== Utilities ==========
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

static inline void publish_fb(ServoChan& ch, int16_t angle){
  ch.pub_msg.data = angle;                          /* ส่งกลับเฉพาะ "ตำแหน่งที่สั่ง" */
  RCSOFTCHECK(rcl_publish(&ch.pub, &ch.pub_msg, NULL));
}

static inline bool accept_angle(const ServoChan& ch, int angle_in, int& angle_out){
  switch(ch.mode){
    case ServoChan::ANY_0_180:
      angle_out = angle_in;
      if(angle_out < 0) angle_out = 0;
      if(angle_out > 180) angle_out = 180;
      return true;
    case ServoChan::ONLY_0_90:
      if(angle_in == 0 || angle_in == 90){ angle_out = angle_in; return true; }
      return false;
    case ServoChan::ONLY_0_180:
      if(angle_in == 0 || angle_in == 180){ angle_out = angle_in; return true; }
      return false;
  }
  return false;
}

// ========== Callback stubs ==========
static void sub_cb_gripper(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int a;
  if(accept_angle(CH_GRIPPER, (int)m->data, a)){
    move_servo_angle(CH_GRIPPER, a);
    CH_GRIPPER.last_angle = (int16_t)a;
    publish_fb(CH_GRIPPER, CH_GRIPPER.last_angle);
  }
}

// เดิม sub_cb_dig → เปลี่ยนเป็น sub_cb_dril
static void sub_cb_dril(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int a;
  if(accept_angle(CH_DRIL, (int)m->data, a)){
    move_servo_angle(CH_DRIL, a);
    CH_DRIL.last_angle = (int16_t)a;
    publish_fb(CH_DRIL, CH_DRIL.last_angle);
  }
}

static void sub_cb_sw180(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  int a;
  if(accept_angle(CH_SW180, (int)m->data, a)){
    move_servo_angle(CH_SW180, a);
    CH_SW180.last_angle = (int16_t)a;
    publish_fb(CH_SW180, CH_SW180.last_angle);
  }
}

// ========== micro-ROS entities ==========
static bool createEntities(){
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));  /* ROS_DOMAIN_ID = 96 */

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_multi_servo_node", "", &support));

  // --- Publishers (best-effort) ---
  RCCHECK(rclc_publisher_init_best_effort(
    &CH_GRIPPER.pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_GRIPPER.pub_topic));

  RCCHECK(rclc_publisher_init_best_effort(
    &CH_DRIL.pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_DRIL.pub_topic));

  RCCHECK(rclc_publisher_init_best_effort(
    &CH_SW180.pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_SW180.pub_topic));

  // --- Subscribers ---
  RCCHECK(rclc_subscription_init_default(
    &CH_GRIPPER.sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_GRIPPER.sub_topic));

  // Subscriber: /man/cmd_servo_dril (std_msgs/Int16)
  RCCHECK(rclc_subscription_init_default(
    &CH_DRIL.sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_DRIL.sub_topic));

  RCCHECK(rclc_subscription_init_default(
    &CH_SW180.sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    CH_SW180.sub_topic));

  // --- Executor (3 subs) ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &CH_GRIPPER.sub, &CH_GRIPPER.sub_msg, &sub_cb_gripper, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &CH_DRIL.sub,    &CH_DRIL.sub_msg,    &sub_cb_dril,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &CH_SW180.sub,   &CH_SW180.sub_msg,   &sub_cb_sw180,   ON_NEW_DATA));

  // --- Initialize servos (ไปที่ 0° และส่งฟีดแบ็กครั้งแรก) ---
  CH_GRIPPER.last_angle = 0;  move_servo_angle(CH_GRIPPER, 0);  publish_fb(CH_GRIPPER, 0);
  CH_DRIL.last_angle    = 0;  move_servo_angle(CH_DRIL, 0);     publish_fb(CH_DRIL, 0);
  CH_SW180.last_angle   = 0;  move_servo_angle(CH_SW180, 0);    publish_fb(CH_SW180, 0);

  return true;
}

static bool destroyEntities(){
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  (void)rcl_subscription_fini(&CH_GRIPPER.sub, &node);
  (void)rcl_subscription_fini(&CH_DRIL.sub, &node);
  (void)rcl_subscription_fini(&CH_SW180.sub, &node);

  (void)rcl_publisher_fini(&CH_GRIPPER.pub, &node);
  (void)rcl_publisher_fini(&CH_DRIL.pub, &node);
  (void)rcl_publisher_fini(&CH_SW180.pub, &node);

  rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  rclc_support_fini(&support);
  return true;
}

// ========== Arduino setup/loop ==========
void setup(){
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // Attach แยกช่อง พร้อม min/max ของตัวเอง (ไม่ใช้ร่วมกัน)
  CH_GRIPPER.servo.attach(CH_GRIPPER.pin, CH_GRIPPER.min_us, CH_GRIPPER.max_us);
  CH_DRIL.servo.attach(CH_DRIL.pin, CH_DRIL.min_us, CH_DRIL.max_us);
  CH_SW180.servo.attach(CH_SW180.pin, CH_SW180.min_us, CH_SW180.max_us);

  // เริ่มที่ 0° ทั้งหมด (เงียบ)
  move_servo_angle(CH_GRIPPER, 0);
  move_servo_angle(CH_DRIL, 0);
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
      // ตรวจ agent เป็นระยะ
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK==rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if(state == AGENT_CONNECTED){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // heartbeat ทุก ~300ms: ส่งตำแหน่งล่าสุดของแต่ละตัวกลับ
        uint32_t now = millis();
        if(CH_GRIPPER.last_angle != -1 && (now - CH_GRIPPER.last_hb_ms) >= 300){
          publish_fb(CH_GRIPPER, CH_GRIPPER.last_angle);
          CH_GRIPPER.last_hb_ms = now;
        }
        if(CH_DRIL.last_angle != -1 && (now - CH_DRIL.last_hb_ms) >= 300){
          publish_fb(CH_DRIL, CH_DRIL.last_angle);
          CH_DRIL.last_hb_ms = now;
        }
        if(CH_SW180.last_angle != -1 && (now - CH_SW180.last_hb_ms) >= 300){
          publish_fb(CH_SW180, CH_SW180.last_angle);
          CH_SW180.last_hb_ms = now;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}
