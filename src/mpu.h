#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>

void mpuSetup();
void rotational_rates();
void median_offset();
void complimentary_filter();


extern Adafruit_MPU6050 mpu;
extern sensors_event_t a, g, temp;

extern uint32_t gyro_last_update;

#endif // MPU_H