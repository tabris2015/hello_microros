#include <stdio.h>

extern "C" {
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
}
#include "button.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "motor.hpp"
#include "ws2812.hpp"


using namespace pimoroni;
using namespace motor;
using namespace encoder;
using namespace plasma;

const uint LED_PIN = 25;
const uint A_PIN = 12;
const int COUNTS_PER_REV = 1428;
const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;

const uint SAMPLE_RATE_MS = 1000 / UPDATES;
const uint PUB_INTERVAL_MS = 100;


static constexpr pin_pair MOTOR_A_PINS{10, 11};
constexpr uint LED_DATA_PIN = 23;

Button button_a(12);
Button button_b(13);

Motor motor_a(MOTOR_A_PINS);


Encoder encoder_a(
        pio0,
        0,
        {4, 5},
        PIN_UNUSED,
        REVERSED_DIR,
        COUNTS_PER_REV,
        true);



float setpoint = 0.0f;

constexpr float KP = 0.2f;
constexpr float KI = 0.3f;
constexpr float KD = 0.0f;

PID pid_a = PID(KP, KI, KD, UPDATE_RATE);

WS2812 rgb_led(1, pio1, 0, LED_DATA_PIN);

rcl_publisher_t publisher;
rcl_subscription_t twist_subscriber;

std_msgs__msg__Float32MultiArray msg;
geometry_msgs__msg__Twist * twist_msg;

int led_state = 0;
float linear_x = 0;

float state_array[5] = {};

void pid_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    Encoder::Capture capture = encoder_a.capture();
    pid_a.setpoint = setpoint;

    float output = pid_a.calculate(capture.radians_per_second());
    motor_a.speed(output);

    static float data[5] = {setpoint, capture.radians_per_second(), output, 0, 0};

    msg.data.capacity = 5;
    msg.data.size = 5;
    msg.data.data = state_array;
    msg.data.data[0] = setpoint;
    msg.data.data[1] = capture.radians_per_second();
    msg.data.data[2] = output;
    msg.data.data[3] = linear_x;
    msg.data.data[4] = 0;

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    // gpio_put(LED_PIN, capture.count() % 4);

}


void twist_callback(const void * msgin)
{
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
    const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;

    linear_x = twist_msg->linear.x;
    rgb_led.set_rgb(0, (uint8_t)twist_msg->linear.x * 10, 0, 0);
}


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rgb_led.start(UPDATES);
    motor_a.init();
    encoder_a.init();

    rcl_timer_t pid_timer;
    rcl_timer_t publisher_timer;

    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "single_motor_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "robot_publisher");

    rclc_timer_init_default(
            &pid_timer,
            &support,
            RCL_MS_TO_NS(SAMPLE_RATE_MS),
            pid_timer_callback);

    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    twist_msg = geometry_msgs__msg__Twist__create();

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &pid_timer);
    rcl_ret_t rc = rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        twist_msg, 
        &twist_callback, 
        ON_NEW_DATA);
    if (RCL_RET_OK != rc) {
    gpio_put(LED_PIN, 1);
    // return -1;
    }
    // gpio_put(LED_PIN, 1);
    while (true)
    {
        if(button_a.read()) {
            setpoint += 1;
        }
        if(button_b.read()) {
            setpoint -= 1;
        }
        
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    return 0;
}
