#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef ESP32_HARDWARE2
    #include <sensor_msgs/msg/imu.h>
    #include <imu_bno055.h>
#endif


#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>
#include <differential_swerve_module/differential_swerve_module.h>


#include <esp32_Encoder.h>    
#include <ESP32Servo.h>



#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
    #define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
    #define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)
    
    //------------------------------ < Define > -------------------------------------//

rcl_publisher_t Modlue1_publisher;
rcl_publisher_t Modlue2_publisher;
rcl_publisher_t Modlue3_publisher;
rcl_publisher_t debug_hall_sensor1_publisher;
rcl_publisher_t debug_hall_sensor2_publisher;
rcl_publisher_t debug_hall_sensor3_publisher;

rcl_subscription_t movement_mode_subscriber;
rcl_subscription_t cmd_vel_subscriber;

#ifdef ESP32_HARDWARE1
rcl_subscription_t module3_subscriber;
std_msgs__msg__Float32MultiArray module3_received_msg;
#elif ESP32_HARDWARE2
// rcl_timer_t imu_timer;

rcl_publisher_t imu_publisher;

rcl_subscription_t module1_subscriber;
rcl_subscription_t module2_subscriber;
rcl_subscription_t arm_position_servo_subscriber;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray module1_received_msg;
std_msgs__msg__Float32MultiArray module2_received_msg;
geometry_msgs__msg__Twist arm_pos_msg;

#endif

std_msgs__msg__Bool hall_sensor1_msg;
std_msgs__msg__Bool hall_sensor2_msg;
std_msgs__msg__Bool hall_sensor3_msg;

std_msgs__msg__String movement_mode_msg;

std_msgs__msg__Float32MultiArray module1_msg;
std_msgs__msg__Float32MultiArray module2_msg;
std_msgs__msg__Float32MultiArray module3_msg;

geometry_msgs__msg__Twist debug_wheel_motorRPM_msg;
geometry_msgs__msg__Twist debug_wheel_encoder_tick_msg;
geometry_msgs__msg__Twist debug_wheel_motor_msg;
geometry_msgs__msg__Twist debug_wheel_encoder_msg;
geometry_msgs__msg__Twist moveMotor_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

long long ticks_L_front = 0;
long long ticks_R_front = 0;
long long ticks_L_rear_left = 0;
long long ticks_R_rear_left = 0;
long long ticks_L_rear_right = 0;
long long ticks_R_rear_right = 0;


float angle_front = 0.0;
float angle_rear_left= 0.0;
float angle_rear_right = 0.0;
float target_angle_front = 0.0;
float target_angle_rear_left = 0.0;
float target_angle_rear_right = 0.0;

float rpm_front_L = 0;
float rpm_front_R = 0;
float rpm_rear_left_L = 0;
float rpm_rear_left_R = 0;
float rpm_rear_right_L = 0;
float rpm_rear_right_R = 0;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;
static unsigned long last_pub = 0;
static int disconnect_count = 0;

bool hall_sensor1;
bool hall_sensor2; 
bool hall_sensor3;

float V_x = 0.0;
float V_y = 0.0;
float Omega_z = 0.0;

String movement_mode = "mps"; 
float motor1_target = 0.0;
float motor2_target = 0.0;
float motor3_target = 0.0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

std::vector<Controller> motors;
Servo Arm_Servo[2];

#ifdef ESP32_HARDWARE1

    PIDF front_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF front_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF rear_left_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF rear_left_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF rear_right_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF rear_right_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);

// Move motor
    Controller motor1(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
    Controller motor2(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
    Controller motor3(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BRAKE, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
    Controller motor4(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_BRAKE, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

    esp32_Encoder Encoder1(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, MOTOR1_ENCODER_RATIO);
    esp32_Encoder Encoder2(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, MOTOR2_ENCODER_RATIO);
    esp32_Encoder Encoder3(MOTOR3_ENCODER_PIN_A, MOTOR3_ENCODER_PIN_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV, MOTOR3_ENCODER_RATIO);
    esp32_Encoder Encoder4(MOTOR4_ENCODER_PIN_A, MOTOR4_ENCODER_PIN_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV, MOTOR4_ENCODER_RATIO);

#elif ESP32_HARDWARE2

    PIDF rear_right_L_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);
    PIDF rear_right_R_pidf(PWM_Min, PWM_Max, Wheel_Motor_KP, Wheel_Motor_KI, Wheel_Motor_I_Min, Wheel_Motor_I_Max, Wheel_Motor_KD, Wheel_Motor_KF, Wheel_Motor_ERROR_TOLERANCE);

    // Move motor
    Controller motor5(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR5_INV, MOTOR5_BRAKE, MOTOR5_PWM, MOTOR5_IN_A, MOTOR5_IN_B);
    Controller motor6(Controller::PRIK_KEE_NOO, PWM_FREQUENCY, PWM_BITS, MOTOR6_INV, MOTOR6_BRAKE, MOTOR6_PWM, MOTOR6_IN_A, MOTOR6_IN_B);

    esp32_Encoder Encoder5(MOTOR5_ENCODER_PIN_A, MOTOR5_ENCODER_PIN_B, COUNTS_PER_REV3, MOTOR5_ENCODER_INV, MOTOR5_ENCODER_RATIO);
    esp32_Encoder Encoder6(MOTOR6_ENCODER_PIN_A, MOTOR6_ENCODER_PIN_B, COUNTS_PER_REV4, MOTOR6_ENCODER_INV, MOTOR6_ENCODER_RATIO);

    IMU_BNO055 bno055;

    const int servoPins[2] = {SERVO_BASE_PIN, SERVO_TOP_PIN};
    
    #endif

#ifdef ESP32_HARDWARE1
void module3Callback(const void *msgin) {
    angle_rear_right = module3_received_msg.data.data[2];
    target_angle_rear_right = module3_received_msg.data.data[4];
}
#elif ESP32_HARDWARE2
void module1Callback(const void *msgin) {
    angle_front = module1_received_msg.data.data[2];
    target_angle_front = module1_received_msg.data.data[4];
}

void module2Callback(const void *msgin) {
    angle_rear_left = module2_received_msg.data.data[2];
    target_angle_rear_left = module2_received_msg.data.data[4];
}

void Arm_position(const void * msgin){
    
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    arm_pos_msg = *msg;

    float MainArm = arm_pos_msg.linear.x; // Main Arm Position
    float SubArm = arm_pos_msg.angular.x; // Sub Arm Position

    Serial.print("MainArm: ");
    Serial.print(MainArm);
    Serial.print(" | SubArm: ");
    Serial.println(SubArm);


    int Pos_main_arm = map(MainArm, 0, 270, 500, 2500); // convert to 0 - 270 degrees to 500 - 2500 pulse 
    int Pos_sub_arm = map(SubArm, 0, 180, 480, 2580); // convert to 0 - 270 degrees to 480 - 2580 pulse

    Arm_Servo[0].writeMicroseconds(Pos_main_arm); // write the pulse to servo
    Arm_Servo[1].writeMicroseconds(Pos_sub_arm); // write the pulse

    
}
#endif
       
void setupComponent() {
    #ifdef ESP32_HARDWARE1
        motors = {motor1, motor2 , motor3, motor4};
        pinMode(Hall_Sensor1, INPUT);
        pinMode(Hall_Sensor2, INPUT);

    #elif ESP32_HARDWARE2
        // pinMode(IMU_RST, OUTPUT);
        // pinMode(IMU_INT, INPUT);

        // digitalWrite(IMU_RST, LOW);
        // delay(10);
        // digitalWrite(IMU_RST, HIGH);
        // delay(50);

        bno055.init();
        motors = {motor5, motor6};
        pinMode(Hall_Sensor3, INPUT_PULLUP);

        Arm_Servo[0].setPeriodHertz(50);
        Arm_Servo[0].attach(servoPins[0], 500, 2500);
        Arm_Servo[0].write(0);

        Arm_Servo[1].setPeriodHertz(50);
        Arm_Servo[1].attach(servoPins[1], 480, 2580);
        Arm_Servo[1].write(0);
    #endif
}


PIDF Angle_Wheel1_pidf(PWM_Min, PWM_Max, Wheel_Spin_Motor_KP, Wheel_Spin_Motor_KI, Wheel_Spin_Motor_I_Min, Wheel_Spin_Motor_I_Max, Wheel_Spin_Motor_KD, Wheel_Spin_Motor_KF, Wheel_Spin_Motor_ERROR_TOLERANCE);
PIDF Angle_Wheel2_pidf(PWM_Min, PWM_Max, Wheel_Spin_Motor_KP, Wheel_Spin_Motor_KI, Wheel_Spin_Motor_I_Min, Wheel_Spin_Motor_I_Max, Wheel_Spin_Motor_KD, Wheel_Spin_Motor_KF, Wheel_Spin_Motor_ERROR_TOLERANCE);
PIDF Angle_Wheel3_pidf(PWM_Min, PWM_Max, Wheel_Spin_Motor_KP, Wheel_Spin_Motor_KI, Wheel_Spin_Motor_I_Min, Wheel_Spin_Motor_I_Max, Wheel_Spin_Motor_KD, Wheel_Spin_Motor_KF, Wheel_Spin_Motor_ERROR_TOLERANCE);


DifferentialSwerveModule module_front(COUNTS_PER_REV1, GEAR_Ratio, 45.0, 10.0, 0.051, 0);
DifferentialSwerveModule module_rear_left(COUNTS_PER_REV1, GEAR_Ratio, 45.0, 10.0, 0.051, 1);
DifferentialSwerveModule module_rear_right(COUNTS_PER_REV1, GEAR_Ratio, 45.0, 10.0, 0.051, 2);
//------------------------------ < Fuction Prototype > ------------------------------//
void setzero();
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();

void publishData();
void getEncoderData();
// void RotageWheel();
void MovePower(float, float, float, float, float, float);
void calculate_Stering();
bool Check_setzero();
//------------------------------ < Main > -------------------------------------//

void setup()
{

    Serial.begin(115200);
    #ifdef MICROROS_WIFI
        
        IPAddress agent_ip(AGENT_IP);
        uint16_t agent_port = AGENT_PORT;
        set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip, agent_port);
    #else
        setupComponent();
        rosidl_runtime_c__float32__Sequence__init(&module1_msg.data, 5);
        rosidl_runtime_c__float32__Sequence__init(&module2_msg.data, 5);
        rosidl_runtime_c__float32__Sequence__init(&module3_msg.data, 5);
        set_microros_serial_transports(Serial);
    #endif
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    

    // EXECUTE_EVERY_N_MS(1000, digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)););
    
    switch (state)
    {
    case WAITING_AGENT:
        // EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #if defined(ESP32)
            EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #else
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        #endif
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        #if defined(ESP32)
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        #else
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        #endif
        if (state == AGENT_CONNECTED)
        {
            #if defined(ESP32)
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
            #else
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            #endif
        }
        break;
    case AGENT_DISCONNECTED:
        MovePower(0, 0, 0, 0, 0, 0);
        destroyEntities();
        disconnect_count = 0;
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//



void calculate_Stering() {
    bool check = Check_setzero();
    // bool check = false;

    #ifdef ESP32_HARDWARE1
    ticks_L_front = Encoder1.read();
    ticks_R_front = Encoder2.read();
    ticks_L_rear_left = Encoder3.read();
    ticks_R_rear_left = Encoder4.read();
    
    
    debug_wheel_encoder_tick_msg.linear.x = ticks_L_front;
    debug_wheel_encoder_tick_msg.linear.y = ticks_R_front;
    debug_wheel_encoder_tick_msg.linear.z = ticks_L_rear_left;
    debug_wheel_encoder_tick_msg.angular.x = ticks_R_rear_left;
    
    
    #elif ESP32_HARDWARE2
    ticks_L_rear_right = Encoder5.read();
    ticks_R_rear_right = Encoder6.read();
    
    debug_wheel_encoder_tick_msg.angular.y = ticks_L_rear_right;
    debug_wheel_encoder_tick_msg.angular.z = ticks_R_rear_right;
    
    #endif
    
    module1_msg.data.size = 5;
    module2_msg.data.size = 5;
    module3_msg.data.size = 5;
    
    // default values
    module1_msg.data.data[0] = 0.0f;
    module1_msg.data.data[1] = 0.0f;
    module1_msg.data.data[2] = 0.0f;
    module1_msg.data.data[3] = 0.0f;
    module2_msg.data.data[0] = 0.0f;
    module2_msg.data.data[1] = 0.0f;
    module2_msg.data.data[2] = 0.0f;
    module2_msg.data.data[3] = 0.0f;
    module3_msg.data.data[0] = 0.0f;
    module3_msg.data.data[1] = 0.0f;
    module3_msg.data.data[2] = 0.0f;
    module3_msg.data.data[3] = 0.0f;
    
    #ifdef ESP32_HARDWARE1
        angle_front      = module_front.update_angle(ticks_L_front, ticks_R_front);
        angle_rear_left  = module_rear_left.update_angle(ticks_L_rear_left, ticks_R_rear_left);
        auto module_front_cmd = module_front.kinematics(V_x, V_y, Omega_z);
        auto module_rear_left_cmd = module_rear_left.kinematics(V_x, V_y, Omega_z);
        
        target_angle_front = module_front_cmd[0].second;
        target_angle_rear_left = module_rear_left_cmd[0].second;


        // module1_msg.data.size = 4;
        // module1_msg.data.data[0] = ticks_L_front;
        // module1_msg.data.data[1] = ticks_R_front;
        module1_msg.data.data[0] = ticks_L_front;
        module1_msg.data.data[1] = ticks_R_front;
        module1_msg.data.data[2] = angle_front ;
        module1_msg.data.data[3] = check ? 1.0f : 0.0f;
        module1_msg.data.data[4] = target_angle_front;
        
        module2_msg.data.data[0] = ticks_L_rear_left;
        module2_msg.data.data[1] = ticks_R_rear_left;
        module2_msg.data.data[2] = angle_rear_left;
        module2_msg.data.data[3] = check ? 1.0f : 0.0f;
        module2_msg.data.data[4] = target_angle_rear_left;

    #elif ESP32_HARDWARE2
        angle_rear_right = module_rear_right.update_angle(ticks_L_rear_right, ticks_R_rear_right);
        auto module_rear_right_cmd = module_rear_right.kinematics(V_x, V_y, Omega_z);
        target_angle_rear_right = module_rear_right_cmd[0].second;

        module3_msg.data.data[0] = ticks_L_rear_right;
        module3_msg.data.data[1] = ticks_R_rear_right;
        // module3_msg.data.data[2] = angle_rear_right * (M_PI / 180.0f);
        module3_msg.data.data[2] = angle_rear_right;
        module3_msg.data.data[3] = check ? 1.0f : 0.0f;
        module3_msg.data.data[4] = target_angle_rear_right;
    #endif
    
    

    
    if(check) {
        return;
    }
    
    // debug_wheel_encoder_msg.angular.z = V_x;
    
    float front_L_speed = 0;
    float front_R_speed = 0;
    float rear_left_L_speed = 0;
    float rear_left_R_speed = 0;
    float rear_right_L_speed = 0;
    float rear_right_R_speed = 0;
    float angle1_correction = 0;
    float angle2_correction = 0;
    float angle3_correction = 0;
    
    
    #ifdef ESP32_HARDWARE1
    // Update the angle of each module based on encoder ticks
    
    
    // Calculate the kinematics for each module
    
    // Calculate the RPM for each wheel
    float speed_front_L_rpm = MPSToRPM(module_front_cmd[0].first, WHEEL_DIAMETER);
    float speed_front_R_rpm = MPSToRPM(module_front_cmd[0].first, WHEEL_DIAMETER);
    float speed_rearLeft_L_rpm = MPSToRPM(module_rear_left_cmd[0].first, WHEEL_DIAMETER);
    float speed_rearLeft_R_rpm = MPSToRPM(module_rear_left_cmd[0].first, WHEEL_DIAMETER);
    
    
    // Calculate the angle correction for each wheel
    angle1_correction = Angle_Wheel1_pidf.compute_with_error(WrapDegs(target_angle_front - angle_front));
    angle2_correction = Angle_Wheel2_pidf.compute_with_error(WrapDegs(target_angle_rear_left - angle_rear_left));
    // module2_msg.data.data[0] = angle1_correction;
    // module2_msg.data.data[1] = WrapDegs(target_angle_front - angle_front);
    
    
    // Calculate the PWM for each wheel based on the RPM and angle correction
    //Modle Front
    float speed_front_L_pwm = front_L_pidf.compute(speed_front_L_rpm, rpm_front_L);
    float speed_front_R_pwm = front_R_pidf.compute(speed_front_R_rpm, rpm_front_R);
    //Modle Rear Left
    float speed_rearLeft_L_pwm = rear_left_L_pidf.compute(speed_rearLeft_L_rpm, rpm_rear_left_L);
    float speed_rearLeft_R_pwm = rear_left_R_pidf.compute(speed_rearLeft_R_rpm, rpm_rear_left_R);
    
    
    // Calculate the maximum PWM for each wheel to normalize the speed
    //Modle Front
    float front_L_d = max(abs(speed_front_L_pwm) + abs(angle1_correction), (float) PWM_Max);
    float front_R_d = max(abs(speed_front_R_pwm) + abs(angle1_correction), (float) PWM_Max);
    //Modle Rear Left
    float rear_left_L_d = max(abs(speed_rearLeft_L_pwm) + abs(angle2_correction), (float) PWM_Max);
    float rear_left_R_d = max(abs(speed_rearLeft_R_pwm) + abs(angle2_correction), (float) PWM_Max);
    
    // Combine steering and driving commands into final PWM output (normalized)
    //Modle Front
    front_L_speed = ((speed_front_L_pwm + angle1_correction)/ front_L_d) * PWM_Max ;
    front_R_speed = ((speed_front_R_pwm - angle1_correction)/ front_R_d) * PWM_Max ;
    //Modle Rear Left
    rear_left_L_speed = ((speed_rearLeft_L_pwm + angle2_correction)/ rear_left_L_d) * PWM_Max ;
    rear_left_R_speed = ((speed_rearLeft_R_pwm - angle2_correction)/ rear_left_R_d) * PWM_Max ;
    
    #elif ESP32_HARDWARE2
    
    float speed_rearRight_L_rpm = MPSToRPM(module_rear_right_cmd[0].first, WHEEL_DIAMETER);
    float speed_rearRight_R_rpm = MPSToRPM(module_rear_right_cmd[0].first, WHEEL_DIAMETER);
    
    angle3_correction = Angle_Wheel2_pidf.compute_with_error(WrapDegs(target_angle_rear_right - angle_rear_right));

    
    //Modle Rear Right
    
    //Modle Rear Right
    float speed_rearRight_L_pwm = rear_right_L_pidf.compute(speed_rearRight_L_rpm, rpm_rear_right_L);
    float speed_rearRight_R_pwm = rear_right_R_pidf.compute(speed_rearRight_R_rpm, rpm_rear_right_R);
    
    float rear_right_L_d = max(abs(speed_rearRight_L_pwm) + abs(angle3_correction), (float) PWM_Max);
    float rear_right_R_d = max(abs(speed_rearRight_R_pwm) + abs(angle3_correction), (float) PWM_Max);

    //Modle Rear Right
    rear_right_L_speed = ((speed_rearRight_L_pwm + angle3_correction)/ rear_right_L_d) * PWM_Max ;
    rear_right_R_speed = ((speed_rearRight_R_pwm - angle3_correction)/ rear_right_R_d) * PWM_Max ;
    
    #endif
    // Move the motors with the calculated speeds

    if (!(AtTargetRange(angle_front, target_angle_front, 5.0) &&
          AtTargetRange(angle_rear_left, target_angle_rear_left, 5.0) &&
          AtTargetRange(angle_rear_right, target_angle_rear_right, 5.0)))
    {
        MovePower(  angle1_correction, -angle1_correction,
                    angle2_correction, -angle2_correction ,
                    angle3_correction, -angle3_correction  );
        
    } else {
        MovePower(  front_L_speed       ,front_R_speed,
                    rear_left_L_speed   ,rear_left_R_speed ,
                    rear_right_L_speed  ,   rear_right_R_speed  );
    }

    // MovePower(  -0   ,  0,
    //             0   ,  -0,
    //             angle3_correction, -angle3_correction  );
    }
    
    
    bool Check_setzero() {
        
        if (abs(V_x) <= 0.05 && abs(V_y) <= 0.05 && abs(Omega_z) <= 0.05) {
            setzero();
            return true;
        } else {
            return false;
        }
        
    }
    void read_hall_sensor(){
        #ifdef ESP32_HARDWARE1
            hall_sensor1 = (digitalRead(Hall_Sensor1) == LOW);
            hall_sensor2 = (digitalRead(Hall_Sensor2) == LOW);
        #elif ESP32_HARDWARE2
            hall_sensor3 = (digitalRead(Hall_Sensor3) == LOW);
        #endif

    hall_sensor1_msg.data = hall_sensor1;
    hall_sensor2_msg.data = hall_sensor2;
    hall_sensor3_msg.data = hall_sensor3;
} 

void setzero(){
    
    float search_speed_L = 600.0;  // ปรับความเร็วตามต้องการ
    float search_speed_R = -600.0;

    #ifdef ESP32_HARDWARE1
    if (hall_sensor1) {
        motor1.spin(0);
        motor2.spin(0);
        Encoder1.reset();
        Encoder2.reset();
        ticks_L_front = 0;
        ticks_R_front = 0;
        module_front.zero_angle();
    }else{
        motor1.spin(search_speed_L);
        motor2.spin(search_speed_R);
    }
    
    if (hall_sensor2) {
        motor3.spin(0);
        motor4.spin(0);
        Encoder3.reset();
        Encoder4.reset();
        ticks_L_rear_left = 0;
        ticks_R_rear_left = 0;
        module_rear_left.zero_angle();
    }else{
        motor3.spin(search_speed_L);
        motor4.spin(search_speed_R);
    }

    #elif ESP32_HARDWARE2

    if (hall_sensor3) {
        motor5.spin(0);
        motor6.spin(0);
        Encoder5.reset();
        Encoder6.reset();
        ticks_L_rear_right = 0;
        ticks_R_rear_right = 0;
        module_rear_right.zero_angle();
    }else{
        motor5.spin(search_speed_L);
        motor6.spin(search_speed_R);
    }
    
    #endif


    
    }

void MovePower(float Motor1Speed, float Motor2Speed, float Motor3Speed, float Motor4Speed, float Motor5Speed, float Motor6Speed)
{
    #ifdef ESP32_HARDWARE1
        motor1.spin(Motor1Speed);
        motor2.spin(Motor2Speed);
        motor3.spin(Motor3Speed);
        motor4.spin(Motor4Speed);

    #elif ESP32_HARDWARE2
        motor5.spin(Motor5Speed);
        motor6.spin(Motor6Speed);     

    #endif   
}


void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        getEncoderData();
        read_hall_sensor(); 
        calculate_Stering();
        publishData();
    }
}

#ifdef ESP32_HARDWARE2
// void imuCallback(rcl_timer_t *timer, int64_t last_call_time)
// {
// }

void imu_data(){
    bno055.getIMUData(imu_msg);

    struct timespec time_stamp = getTime();
    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg.header.frame_id.data = "imu_link";

    imu_msg.angular_velocity_covariance[0] = 0.0001;
    imu_msg.angular_velocity_covariance[4] = 0.0001;
    imu_msg.angular_velocity_covariance[8] = 0.0001;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[8] = 0.0025;
}
#endif

void cmd_vel_callback(const void * msgin) 
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    V_x = msg->linear.x;
    V_y = msg->linear.y;
    Omega_z = msg->angular.z;
}
void wheelMoveCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor1_target = moveMotor_msg.linear.x;
    motor2_target = moveMotor_msg.linear.y;
    motor3_target = moveMotor_msg.linear.z;
}

void movementModeCallback(const void *msgin)
{
    prev_cmd_time = millis();
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    movement_mode = movement_mode_msg.data.data;  // Copy it to global for later use
}


bool createEntities()
{
    allocator = rcl_get_default_allocator();
    
    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);
    
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    
    // create node
    RCCHECK(rclc_node_init_default(&node, "differential_swerve_basemove_hardware", "", &support));

    // Publishers
    #ifdef ESP32_HARDWARE1
    
        RCCHECK(rclc_publisher_init_best_effort(
            &Modlue1_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module1"));

        RCCHECK(rclc_publisher_init_best_effort(
            &debug_hall_sensor1_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "debug/hall_sensor1"));
    
        RCCHECK(rclc_publisher_init_best_effort(
            &debug_hall_sensor2_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "debug/hall_sensor2"));


        RCCHECK(rclc_publisher_init_best_effort(
            &Modlue2_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module2"));

        RCCHECK(rclc_subscription_init_best_effort(
            &module3_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module3"));
        
    #elif ESP32_HARDWARE2

        RCCHECK(rclc_publisher_init_best_effort(
            &Modlue3_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module3"));

        RCCHECK(rclc_publisher_init_best_effort(
            &debug_hall_sensor3_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "debug/hall_sensor3"));

        RCCHECK(rclc_publisher_init_best_effort(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu/data"));
            
        RCCHECK(rclc_subscription_init_best_effort(
            &module1_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module1"));

        RCCHECK(rclc_subscription_init_best_effort(
            &module2_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug/module2"));

        RCCHECK(rclc_subscription_init_default(
            &arm_position_servo_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/servo_position"));
    
        // const unsigned int imu_timeout = 70;
        // RCCHECK(rclc_timer_init_default(
        //     &imu_timer,
        //     &support,
        //     RCL_MS_TO_NS(imu_timeout),
        //     &imuCallback));

        #endif

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_cmd_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/cmd_vel/esp"));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_move_wheel_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/wheel/encoder_rpm_esp"));

    RCCHECK(rclc_subscription_init_default(
        &movement_mode_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/movement_mode"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
        
    // create timer for control loop 1000/80 Hz
    const unsigned int control_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        &controlCallback));
        
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
        
    #ifdef ESP32_HARDWARE1
        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &module3_subscriber,
            &module3_received_msg,
            &module3Callback,
            ON_NEW_DATA));
        
    #elif ESP32_HARDWARE2
        // RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &module1_subscriber,
            &module1_received_msg,
            &module1Callback,
            ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &module2_subscriber,
            &module2_received_msg,
            &module2Callback,
            ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(
            &executor,
            &arm_position_servo_subscriber,
            &arm_pos_msg,
            &Arm_position,
            ON_NEW_DATA));
    #endif
    
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &movement_mode_subscriber,
        &movement_mode_msg,
        &movementModeCallback,
        ON_NEW_DATA));

    // synchronize time with the agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_subscription_fini(&movement_mode_subscriber, &node);
    rcl_publisher_fini(&debug_cmd_vel_publisher, &node);
    rcl_publisher_fini(&debug_move_wheel_encoder_publisher, &node);
    
    #ifdef ESP32_HARDWARE1
    rcl_publisher_fini(&Modlue1_publisher, &node);
    rcl_publisher_fini(&Modlue2_publisher, &node);
    rcl_publisher_fini(&debug_hall_sensor1_publisher, &node);
    rcl_publisher_fini(&debug_hall_sensor2_publisher, &node);
    rcl_subscription_fini(&module3_subscriber, &node);

    #elif ESP32_HARDWARE2
        rcl_publisher_fini(&Modlue3_publisher, &node);
        rcl_publisher_fini(&imu_publisher, &node);
        rcl_publisher_fini(&debug_hall_sensor3_publisher, &node);
        rcl_subscription_fini(&arm_position_servo_subscriber, &node);
        rcl_subscription_fini(&module1_subscriber, &node);
        rcl_subscription_fini(&module2_subscriber, &node);

        Arm_Servo[0].detach();
        Arm_Servo[1].detach();
        
    #endif

    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void getEncoderData()
{
    #ifdef ESP32_HARDWARE1
    // Get encoder data
    rpm_front_L = Encoder1.getRPM();
    rpm_front_R = Encoder2.getRPM();
    rpm_rear_left_L = Encoder3.getRPM();
    rpm_rear_left_R = Encoder4.getRPM();

    debug_wheel_motorRPM_msg.linear.x = rpm_front_L;
    debug_wheel_motorRPM_msg.linear.y = rpm_front_R;
    debug_wheel_motorRPM_msg.linear.z = rpm_rear_left_L;
    debug_wheel_motorRPM_msg.angular.x = rpm_rear_left_R;
    #elif ESP32_HARDWARE2
    rpm_rear_right_L = Encoder5.getRPM();
    rpm_rear_right_R = Encoder6.getRPM();

    debug_wheel_motorRPM_msg.angular.y = rpm_rear_right_L;
    debug_wheel_motorRPM_msg.angular.z = rpm_rear_right_R;
    #endif
    // debug_wheel_encoder_msg.linear.x = rpm_front_L;
    // debug_wheel_encoder_msg.linear.y = rpm_front_R;
    // debug_wheel_encoder_msg.linear.z = 0.0;
    // debug_wheel_encoder_msg.linear.z = Encoder3.getRPM();

}

void publishData()
{
    debug_wheel_motor_msg.linear.x = cmd_vel_msg.linear.x;
    debug_wheel_motor_msg.linear.y = cmd_vel_msg.linear.y;
    debug_wheel_motor_msg.angular.z = cmd_vel_msg.angular.z;
    struct timespec time_stamp = getTime();
    // rcl_publish(&debug_wheel_motor_RPM_publisher, &debug_wheel_motorRPM_msg, NULL);
    // rcl_publish(&debug_wheel_encoder_tick_publisher, &debug_wheel_encoder_tick_msg, NULL);
    rcl_publish(&debug_cmd_vel_publisher, &debug_wheel_motor_msg, NULL);
    rcl_publish(&debug_move_wheel_encoder_publisher, &debug_wheel_encoder_msg, NULL);

    #ifdef ESP32_HARDWARE1
        rcl_publish(&debug_hall_sensor1_publisher, &hall_sensor1_msg, NULL);
        rcl_publish(&debug_hall_sensor2_publisher, &hall_sensor2_msg, NULL);
        rcl_publish(&Modlue1_publisher, &module1_msg, NULL);
        rcl_publish(&Modlue2_publisher, &module2_msg, NULL);
    #elif ESP32_HARDWARE2
        imu_data();
        rcl_publish(&debug_hall_sensor3_publisher, &hall_sensor3_msg, NULL);
        rcl_publish(&Modlue3_publisher, &module3_msg, NULL);
        rcl_publish(&imu_publisher, &imu_msg, NULL);
    #endif
}

void syncTime()
{
    // get the current time from the agent
        unsigned long now = millis();
        RCCHECK(rmw_uros_sync_session(10));
        unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
        tp.tv_sec = now / 1000;
        tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        delay(1000);
    }
}