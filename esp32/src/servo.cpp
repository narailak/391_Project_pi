#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#define RCCHECK(fn) { rcl_ret_t rc=(fn); if(rc!=RCL_RET_OK){ Serial.printf("RCL err %d @ %s:%d\n", rc, __FILE__, __LINE__); while(1){delay(100);} } }
#define RCSOFTCHECK(fn) (void)(fn)
#define EXECUTE_EVERY_N_MS(MS, X) do{ static int64_t t=-1; if(t==-1)t=uxr_millis(); if(uxr_millis()-t>(MS)){ X; t=uxr_millis(); } }while(0)

#define SERVO_PIN     18
#define SERVO_MIN_US  1000   // ส่วนใหญ่ SG90/MG90S ชอบช่วง 1000–2000us
#define SERVO_MAX_US  2000

Servo servo;

rcl_subscription_t sub_cmd;
std_msgs__msg__Int16MultiArray cmd_msg;

rcl_publisher_t pub_fb;
std_msgs__msg__Int16MultiArray fb_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

static int16_t last_cmd = -1;
static int16_t last_us  = -1;
static uint32_t last_hb = 0;

static inline void move_servo_us(int us){
  us = us < SERVO_MIN_US ? SERVO_MIN_US : (us > SERVO_MAX_US ? SERVO_MAX_US : us);
  servo.writeMicroseconds(us);
  Serial.printf("[servo] pulse=%dus\n", us);
}

void publish_fb(int16_t cmd, int16_t pulse){
  fb_msg.data.size = 2;
  fb_msg.data.data[0] = cmd;
  fb_msg.data.data[1] = pulse;
  rcl_ret_t rc = rcl_publish(&pub_fb, &fb_msg, NULL);
  Serial.printf("[fb] publish cmd=%d pulse=%d rc=%d\n", cmd, pulse, (int)rc);
}

void cmd_cb(const void* msgin){
  const auto* m = (const std_msgs__msg__Int16MultiArray*)msgin;
  if(m->data.size==0){ Serial.println("[servo] empty msg"); return; }

  const int v = m->data.data[0];
  int target_us = -1;

  // map แบบตรง: 500 -> 1000us (≈0°), 1500 -> 2000us (≈180°)
  if(v==500 || v==0)        target_us = 1000;
  else if(v==1500 || v==1)  target_us = 2000;

  if(target_us>0){
    move_servo_us(target_us);
    last_cmd = (int16_t)v;
    last_us  = (int16_t)target_us;
    publish_fb(last_cmd, last_us);
  }else{
    Serial.printf("[servo] unknown cmd=%d (ignored)\n", v);
  }
}

bool createEntities(){
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));   // DOMAIN ID = 96

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  // buffers
  std_msgs__msg__Int16MultiArray__init(&cmd_msg);
  rosidl_runtime_c__int16__Sequence__init(&cmd_msg.data, 4);

  std_msgs__msg__Int16MultiArray__init(&fb_msg);
  rosidl_runtime_c__int16__Sequence__init(&fb_msg.data, 4);

  // sub
  RCCHECK(rclc_subscription_init_default(
    &sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/man/cmd_slap"));

  // pub (best-effort)
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_fb, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/man/cmd_slap/rpm"));

  // executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &cmd_cb, ON_NEW_DATA));

  // initial feedback
  last_cmd = 500; last_us = 1000;
  publish_fb(last_cmd, last_us);

  Serial.println("connected: Sub /man/cmd_slap, Pub /man/cmd_slap/rpm (best-effort)");
  return true;
}

bool destroyEntities(){
  rmw_context_t* rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&sub_cmd, &node);
  rcl_publisher_fini(&pub_fb, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  if(cmd_msg.data.data) rosidl_runtime_c__int16__Sequence__fini(&cmd_msg.data);
  std_msgs__msg__Int16MultiArray__fini(&cmd_msg);

  if(fb_msg.data.data) rosidl_runtime_c__int16__Sequence__fini(&fb_msg.data);
  std_msgs__msg__Int16MultiArray__fini(&fb_msg);

  return true;
}

void setup(){
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // servo init + sweep test (ฮาร์ดแวร์ต้องขยับตรงนี้ ถ้าไม่ขยับ = สาย/ไฟ)
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  Serial.println("Boot sweep test...");
  move_servo_us(1200); delay(400);
  move_servo_us(1800); delay(400);
  move_servo_us(1500);

  Serial.println("waiting agent...");
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
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // heartbeat ทุก 300ms ถ้ามีค่า
        uint32_t now = millis();
        if(last_cmd!=-1 && last_us!=-1 && now-last_hb>=300){
          publish_fb(last_cmd, last_us);
          last_hb = now;
        }
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}
