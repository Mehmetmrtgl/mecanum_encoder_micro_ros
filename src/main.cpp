#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <std_msgs/msg/float32_multi_array.h>

#define ENCODER_A_1 2  // Enkoderin A kanalı (interrupt pin)
#define ENCODER_B_1 15  // Enkoderin B kanalı

#define ENCODER_A_2 13  // Enkoderin A kanalı (interrupt pin)
#define ENCODER_B_2 12  // Enkoderin B kanalı

#define ENCODER_A_3 26  // Enkoderin A kanalı (interrupt pin)
#define ENCODER_B_3 27  // Enkoderin B kanalı

#define ENCODER_A_4 18  // Enkoderin A kanalı (interrupt pin)
#define ENCODER_B_4 19  // Enkoderin B kanalı

volatile long encoder_count_1 = 0;
volatile long encoder_count_2 = 0;
volatile long encoder_count_3 = 0;
volatile long encoder_count_4 = 0;

int counts_per_rev_ = 700;

unsigned long last_time;
long previous_encoder_count_1 = 0;
long previous_encoder_count_2 = 0;
long previous_encoder_count_3 = 0;
long previous_encoder_count_4 = 0;

unsigned long prev_update_time_;
int64_t prev_encoder_ticks_;

float rpm_1 = 0;
float rpm_2 = 0;
float rpm_3 = 0;
float rpm_4 = 0;


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t encoder_publisher;
rcl_publisher_t rpm_publisher;

std_msgs__msg__Float32MultiArray rpm_msg;

rcl_timer_t timer;
rclc_executor_t executor;

std_msgs__msg__Int32MultiArray encoder_msg;

// Hata kontrol makrosu
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { /* Hata durumu yönetilebilir */ }}

void error_loop() {
    while (1) {
        delay(500);
    }
}

void IRAM_ATTR readEncoder1() {
    encoder_count_1 += (GPIO.in & (1<< ENCODER_B_1)) ? 1 : -1;
}

void IRAM_ATTR readEncoder2() {
    encoder_count_2 += (GPIO.in & (1<< ENCODER_B_2)) ? 1 : -1;

}

void IRAM_ATTR readEncoder3() {
    encoder_count_3 += (GPIO.in & (1<< ENCODER_B_3)) ? 1 : -1;

}

void IRAM_ATTR readEncoder4() {
    encoder_count_4 += (GPIO.in & (1<< ENCODER_B_4)) ? 1 : -1;

}

float getRPM(int64_t encoder_ticks, int64_t previous_encoder_ticks, double dtm) {
    if (counts_per_rev_ <= 0) return 0.0;  // Make sure counts per revolution is set properly

    int64_t delta_ticks = encoder_ticks - previous_encoder_ticks;

    // Prevent extremely high or low RPM values if delta_ticks is too small
    if (dtm == 0) return 0.0;

    return ((double)delta_ticks / counts_per_rev_) / dtm;  // RPM calculation
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) {
        return;
    }

    
    unsigned long current_time = micros();
    unsigned long dt = current_time - prev_update_time_;  // delta time in microseconds
    if (dt == 0) return;  // Avoid division by zero if time difference is zero
    double dtm = (double)dt / 60000000.0;  // Convert microseconds to minutes

    rpm_1 = getRPM(encoder_count_1, previous_encoder_count_1, dtm);
    rpm_2 = getRPM(encoder_count_2, previous_encoder_count_2, dtm);
    rpm_3 = getRPM(encoder_count_3, previous_encoder_count_3, dtm);
    rpm_4 = getRPM(encoder_count_4, previous_encoder_count_4, dtm);

    rpm_msg.data.data[0] = rpm_1;
    rpm_msg.data.data[1] = rpm_2;
    rpm_msg.data.data[2] = rpm_3;
    rpm_msg.data.data[3] = rpm_4;

    RCSOFTCHECK(rcl_publish(&rpm_publisher, &rpm_msg, NULL));

    previous_encoder_count_1 = encoder_count_1;
    previous_encoder_count_2 = encoder_count_2;
    previous_encoder_count_3 = encoder_count_3;
    previous_encoder_count_4 = encoder_count_4;
    
    encoder_msg.data.data[0] = encoder_count_1;
    encoder_msg.data.data[1] = encoder_count_2;
    encoder_msg.data.data[2] = encoder_count_3;
    encoder_msg.data.data[3] = encoder_count_4;

    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

    prev_update_time_ = current_time;

}



void setup() {
    Serial.begin(921600);
    pinMode(ENCODER_A_1, INPUT_PULLDOWN);
    pinMode(ENCODER_B_1, INPUT_PULLDOWN);
    pinMode(ENCODER_A_2, INPUT_PULLDOWN);
    pinMode(ENCODER_B_2, INPUT_PULLDOWN);
    pinMode(ENCODER_A_3, INPUT_PULLDOWN);
    pinMode(ENCODER_B_3, INPUT_PULLDOWN);
    pinMode(ENCODER_A_4, INPUT_PULLDOWN);
    pinMode(ENCODER_B_4, INPUT_PULLDOWN);

    set_microros_serial_transports(Serial);
    delay(100);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_3), readEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_4), readEncoder4, RISING);

    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Node creation
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // Executor creation
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));


    RCCHECK(rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks"));

    // RPM publisher initialization
    RCCHECK(rclc_publisher_init_default(
        &rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "rpm_data"));

    encoder_msg.data.size = 4;  // 4 encoders
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data = (int32_t*)malloc(encoder_msg.data.capacity * sizeof(int32_t));

    rpm_msg.data.size = 4;  // 4 RPM values
    rpm_msg.data.capacity = 4;
    rpm_msg.data.data = (float*)malloc(rpm_msg.data.capacity * sizeof(float));
    
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
