#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>

void mpuSetup();
void mpuRead();

extern Adafruit_MPU6050 mpu;
extern sensors_event_t a, g, temp;
#endif // MPU_H