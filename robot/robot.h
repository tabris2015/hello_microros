//
// Created by pepe on 02-05-24.
//
#pragma once

#include <stdint.h>
#include "pico/stdlib.h"
#include "button.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "ws2812.hpp"
#define COUNTS_PER_REV 1428
#define WHEEL_DISTANCE 0.1
#define WHEEL_RADIUS 0.03

struct pid_params {
    float kp;
    float ki;
    float kd;
    float rate;
};
struct wheel_state {
    float setpoint;
    float reading;
    float output;
};

struct robot_state {
    wheel_state l_wheel_state;
    wheel_state r_wheel_state;
};

class Robot {
public:
    Robot(
            uint led_pin,
            uint rgb_led_pin,
            uint button_pin,
            pimoroni::pin_pair l_motor_pins,
            pimoroni::pin_pair r_motor_pins,
            pimoroni::pin_pair l_encoder_pins,
            pimoroni::pin_pair r_encoder_pins,
            pid_params l_pid_params,
            pid_params r_pid_params
            ) :
    led_pin(led_pin),
    button(button_pin),
    rgb_led(1, pio1, 0, rgb_led_pin),
    l_motor(l_motor_pins), r_motor(r_motor_pins),
    l_encoder(pio0, 0, l_encoder_pins, PIN_UNUSED, REVERSED_DIR, COUNTS_PER_REV, true),
    r_encoder(pio0, 1, r_encoder_pins, PIN_UNUSED, REVERSED_DIR, COUNTS_PER_REV, true),
    l_pid(l_pid_params.kp, l_pid_params.ki, l_pid_params.kd, l_pid_params.rate),
    r_pid(r_pid_params.kp, r_pid_params.ki, r_pid_params.kd, r_pid_params.rate),
    l_setpoint_rad_s(0), r_setpoint_rad_s(0),
    wheel_distance(WHEEL_DISTANCE), wheel_radius(WHEEL_RADIUS)
    {
        gpio_init(led_pin);
        gpio_set_dir(led_pin, GPIO_OUT);
        rgb_led.start(100);
        l_encoder.init();
        r_encoder.init();
        l_motor.init();
        r_motor.init();

    };
    void on();
    void off();
    robot_state pid_step();
    void set_rgb(uint8_t r, uint8_t g, uint8_t b);
    void set_wheel_speeds(float l_rad_s, float r_rad_s);
    void set_unicycle(float linear, float angular);

    float l_setpoint_rad_s;
    float r_setpoint_rad_s;

private:
    uint led_pin;
    uint button_pin;
    pimoroni::Button button;
    plasma::WS2812 rgb_led;
    motor::Motor l_motor;
    motor::Motor r_motor;
    encoder::Encoder l_encoder;
    encoder::Encoder r_encoder;
    PID l_pid;
    PID r_pid;
    float wheel_distance;
    float wheel_radius;
};