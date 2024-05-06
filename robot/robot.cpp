//
// Created by pepe on 02-05-24.
//
#include "robot.h"

void Robot::on() {
    gpio_put(led_pin, 1);
}

void Robot::off() {
    gpio_put(led_pin, 0);
}

void Robot::set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    rgb_led.set_rgb(0, r, g, b);
}

robot_state Robot::pid_step() {
    encoder::Encoder::Capture l_capture = l_encoder.capture();
    encoder::Encoder::Capture r_capture = r_encoder.capture();

    l_pid.setpoint = l_setpoint_rad_s;
    r_pid.setpoint = r_setpoint_rad_s;


    float l_value = l_capture.radians_per_second();
    float r_value = r_capture.radians_per_second();

    update_odometry(l_capture.delta(), r_capture.delta());


    float l_out = l_pid.calculate(l_value);
    float r_out = r_pid.calculate(r_value);

    l_motor.speed(l_out);
    r_motor.speed(r_out);

    robot_state state {
            {l_setpoint_rad_s, l_value, l_out},
            {r_setpoint_rad_s, r_value, r_out},
    };

    state.odometry = odometry;
    return state;
}

void Robot::set_wheel_speeds(float l_rad_s, float r_rad_s) {
    l_setpoint_rad_s = l_rad_s;
    r_setpoint_rad_s = r_rad_s;
}

void Robot::set_unicycle(float linear, float angular) {
    l_setpoint_rad_s = ((2 * linear) - (angular * wheel_distance)) / (2 * wheel_radius);
    r_setpoint_rad_s = ((2 * linear) + (angular * wheel_distance)) / (2 * wheel_radius);
}

void Robot::update_odometry(int32_t dl_ticks, int32_t dr_ticks) {
    float delta_l = (2 * M_PI * wheel_radius * dl_ticks) / COUNTS_PER_REV;
    float delta_r = (2 * M_PI * wheel_radius * dr_ticks) / COUNTS_PER_REV;

    // Compute center displacement
    float delta_center = (delta_r - delta_l) / wheel_distance;

    odometry.x_pos += delta_center * cosf(odometry.theta);
    odometry.y_pos += delta_center * sinf(odometry.theta);
    odometry.theta += (delta_r - delta_l) / wheel_distance;
    odometry.v = (wheel_radius / 2) * (delta_r + delta_l);
    odometry.w = (wheel_radius / wheel_distance) * (delta_r - delta_l);
}