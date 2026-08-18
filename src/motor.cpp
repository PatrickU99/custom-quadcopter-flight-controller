#include <Arduino.h>
#include <Wire.h>
#include "motor.h"

const int MOTOR_PIN1   = 4;
const int MOTOR_PIN2   = 5;
const int MOTOR_PIN3   = 6;
const int MOTOR_PIN4   = 7;

const int PWM_CHANNEL1 = 0;
const int PWM_CHANNEL2 = 1;
const int PWM_CHANNEL3 = 2;
const int PWM_CHANNEL4 = 3;

const int PWM_FREQ    = 50;   // 50Hz standard servo/ESC refresh rate
const int PWM_RES     = 12;   // 12-bit - matches what the LEDC timer can actually hit at 50Hz
 
const int ARM_US      = 1000; // minimum throttle (us) - arms the ESC, also = safe idle
const int SPIN_US     = 1300; // steady low throttle to hold

uint32_t usToDuty(int us) {
    uint32_t period_us = 1000000UL / PWM_FREQ;
    uint32_t maxDuty = (1UL << PWM_RES) - 1;
    return (uint32_t)((uint64_t)us * maxDuty / period_us);
}

void motorSetup() {
    ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CHANNEL4, PWM_FREQ, PWM_RES);
  
    ledcAttachPin(MOTOR_PIN1, PWM_CHANNEL1);
    ledcAttachPin(MOTOR_PIN2, PWM_CHANNEL2);
    ledcAttachPin(MOTOR_PIN3, PWM_CHANNEL3);
    ledcAttachPin(MOTOR_PIN4, PWM_CHANNEL4);
  
    Serial.println("Arming... hold clear of the motor.");
  
    ledcWrite(PWM_CHANNEL1, usToDuty(ARM_US));
    ledcWrite(PWM_CHANNEL2, usToDuty(ARM_US));
    ledcWrite(PWM_CHANNEL3, usToDuty(ARM_US));
    ledcWrite(PWM_CHANNEL4, usToDuty(ARM_US));
}