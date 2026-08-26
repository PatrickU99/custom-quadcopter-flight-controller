#ifndef MPU_SETUP_H
#define MPU_SETUP_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>

void mpuSetup();


extern Adafruit_MPU6050 mpu;
extern sensors_event_t a, g, temp;

extern uint32_t gyro_last_update;

#endif // MPU_SETUP_H