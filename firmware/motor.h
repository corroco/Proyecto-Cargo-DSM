#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define PWM_A 3
#define DIR_A 4

#define PWM_B 5
#define DIR_B 6

#define MIN_SPEED_PWM 42 //TODO: check to match 0.1m/s
#define MAX_SPEED_PWM 122 //TODO: check to match 1m/s
#define MIN_SPEED 0.1
//securing the maximum speed to 1 m/s
#define MAX_SPEED 1


void motors_setup(void);

void motor_cmd_vel(int pwm_a, int pwm_b);

void motor_stop(void);

#endif
