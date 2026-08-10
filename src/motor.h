#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>


extern const int MOTOR_PIN1;
extern const int MOTOR_PIN2;
extern const int MOTOR_PIN3;
extern const int MOTOR_PIN4;

extern const int PWM_CHANNEL1;
extern const int PWM_CHANNEL2;
extern const int PWM_CHANNEL3;
extern const int PWM_CHANNEL4;

extern const int PWM_FREQ;
extern const int PWM_RES;

extern const int ARM_US;
extern const int SPIN_US;

uint32_t usToDuty(int us);

#endif // MOTOR_H