#include <Arduino.h>
#include <Wire.h>
#include "mpu_setup.h"
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;// Create an instance of the MPU6050 class
sensors_event_t a, g, temp; // Accelerometer, Gyroscope, Temperature events

bool error;

void mpuSetup() {
  Wire.begin(8, 9); // SDA, SCL
  error = mpu.begin(0x68, &Wire); // I2C address, Wire object. Returns true/false for success
  if (error == true) {
    Serial.println("MPU6050 found!"); // MPU6050 detected
  } else {
    Serial.println("MPU6050 not detected - check wiring.");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Set accelerometer range to +/- 2g
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyroscope range to +/- 250 degrees/second
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Set filter bandwidth to 21Hz
}


