    // ===================== main.cpp (ESP32 + micro-ROS + ESP32Encoder) =====================
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

    // ---------------- Robot params ----------
    const float WHEEL_RADIUS = 0.060f;        // m
    const float TRACK_WIDTH  = 0.300f;        // m
    const float MAX_RPM      = 60.0f;         // RPM limit
    const float PPR          = 600.0f;        // Pulses Per Revolution
    const float WHEEL_DIAMETER = WHEEL_RADIUS * 2.0f;

    // Safety
    const float DEADBAND           = 0.02f;
    const uint32_t CMD_TIMEOUT_MS  = 300;
    const float SLEW_RPM_PER_SEC   = 200.0f;

    // ---------------- Encoder Class (based on ESP32Encoder) --------------
    enum WheelIndex { W_M1=0, W_M2=1, W_M3=2, W_M4=3, W_COUNT=4 };

    class QuadEncoderReader {
    public:
    QuadEncoderReader() {
        pins_[W_M1][0] = ENC1_A; pins_[W_M1][1] = ENC1_B;
        pins_[W_M2][0] = ENC2_A; pins_[W_M2][1] = ENC2_B;
        pins_[W_M3][0] = ENC3_A; pins_[W_M3][1] = ENC3_B;
        pins_[W_M4][0] = ENC4_A; pins_[W_M4][1] = ENC4_B;
        
        // Inversion (1=normal, -1=reverse)
        inv_[W_M1] = 1;
        inv_[W_M2] = 1;
        inv_[W_M3] = 1;
        inv_[W_M4] = 1;
    }

    void begin(bool enable_pullups = false) {
        ESP32Encoder::useInternalWeakPullResistors = enable_pullups ? UP : DOWN;
        
        enc_[W_M1].attachHalfQuad(pins_[W_M1][0], pins_[W_M1][1]);
        enc_[W_M2].attachHalfQuad(pins_[W_M2][0], pins_[W_M2][1]);
        enc_[W_M3].attachHalfQuad(pins_[W_M3][0], pins_[W_M3][1]);
        enc_[W_M4].attachHalfQuad(pins_[W_M4][0], pins_[W_M4][1]);
        
        for (int i=0; i<W_COUNT; ++i) {
        enc_[i].clearCount();
        last_counts_[i] = 0;
        pos_rad_[i] = 0.0f;
        vel_rad_s_[i] = 0.0f;
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
        const float C = M_PI * WHEEL_DIAMETER; // circumference

        for (int i=0; i<W_COUNT; ++i) {
        long cur = enc_[i].getCount() * inv_[i];
        long d = cur - last_counts_[i];
        last_counts_[i] = cur;

        total_dist_m_[i] += ((float)d / PPR) * C;
        pos_rad_[i] = ((float)cur / PPR) * two_pi;

        float rev_dt = ((float)d / PPR) / dt;
        vel_rad_s_[i] = rev_dt * two_pi;
        }
    }

    float getPosRad(int i) const { return pos_rad_[i]; }
    float getVelRadS(int i) const { return vel_rad_s_[i]; }
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
    float total_dist_m_[W_COUNT];
    uint32_t last_ts_ms_;
    };

    // ---------------- Global Encoder Object ----------
    QuadEncoderReader encoders;

    // ---------------- ROS entities ----------
    rcl_publisher_t debug_motor_pub;
    std_msgs__msg__Float32MultiArray debug_motor_msg;

    rcl_publisher_t encoder_pub;
    std_msgs__msg__Int16MultiArray encoder_msg;

    rcl_publisher_t encoder_vel_pub;  // NEW: velocity publisher
    std_msgs__msg__Float32MultiArray encoder_vel_msg;

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
    float a = fabsf(rpm)/MAX_RPM; 
    if(a>1) a=1;
    return (uint8_t)(a*255.0f);
    }

    float clampf(float v, float lo, float hi){ 
    return v<lo ? lo : (v>hi ? hi : v); 
    }

    float v_to_rpm(float v_mps){ 
    return (v_mps / WHEEL_RADIUS) * 60.0f / (2.0f*M_PI); 
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

    // Initialize encoders with ESP32Encoder
    encoders.begin(false); // GPIO 34-39 don't support internal pullup

    Serial.println("ESP32 Differential + micro-ROS + ESP32Encoder Ready!");
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
    
    // Update encoder readings
    encoders.update();
    
    // Control motors
    controlStep(dt);
    
    // Publish data
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

    // Deadband
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
    
    std_msgs__msg__Int32__init(&counter_msg);
    geometry_msgs__msg__Twist__init(&cmd_msg);

    // Init options
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 96));

    // Support & Node
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
    
    return true;
    }

    // ======================= Control ================================================
    void controlStep(float dt){
    // Timeout check
    if (millis() - last_cmd_ms > CMD_TIMEOUT_MS){
        tgt_m1_rpm = tgt_m2_rpm = tgt_m3_rpm = tgt_m4_rpm = 0.0f;
    } else {
        // Differential kinematics
        float vx = cmd_vx;
        float wz = cmd_wz;

        float v_left  = vx - wz * (TRACK_WIDTH * 0.5f);
        float v_right = vx + wz * (TRACK_WIDTH * 0.5f);

        float rpm_left  = clampf(v_to_rpm(v_left),  -MAX_RPM, MAX_RPM);
        float rpm_right = clampf(v_to_rpm(v_right), -MAX_RPM, MAX_RPM);

        tgt_m1_rpm = rpm_left;   // Left front
        tgt_m2_rpm = rpm_right;  // Right front
        tgt_m3_rpm = rpm_left;   // Left rear
        tgt_m4_rpm = rpm_right;  // Right rear
    }

    // Slew-rate limiting
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
    // Motor duty cycle
    debug_motor_msg.data.data[0] = rpm_to_duty(m1_rpm);
    debug_motor_msg.data.data[1] = rpm_to_duty(m2_rpm);
    debug_motor_msg.data.data[2] = rpm_to_duty(m3_rpm);
    debug_motor_msg.data.data[3] = rpm_to_duty(m4_rpm);
    RCSOFTCHECK(rcl_publish(&debug_motor_pub, &debug_motor_msg, NULL));

    // Encoder counts (int16 wrap-around for compatibility)
    encoder_msg.data.data[0] = (int16_t)(encoders.getCount(W_M1) & 0xFFFF);
    encoder_msg.data.data[1] = (int16_t)(encoders.getCount(W_M2) & 0xFFFF);
    encoder_msg.data.data[2] = (int16_t)(encoders.getCount(W_M3) & 0xFFFF);
    encoder_msg.data.data[3] = (int16_t)(encoders.getCount(W_M4) & 0xFFFF);
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

    // Encoder velocities (rad/s)
    encoder_vel_msg.data.data[0] = encoders.getVelRadS(W_M1);
    encoder_vel_msg.data.data[1] = encoders.getVelRadS(W_M2);
    encoder_vel_msg.data.data[2] = encoders.getVelRadS(W_M3);
    encoder_vel_msg.data.data[3] = encoders.getVelRadS(W_M4);
    RCSOFTCHECK(rcl_publish(&encoder_vel_pub, &encoder_vel_msg, NULL));
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