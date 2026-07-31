#include <Arduino.h>
#include <Wire.h>
#include "mpu.h"
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

void mpuRead() {
    mpu.getEvent(&a, &g, &temp);
    Serial.println("Accelerometer:");
    Serial.print("X: "); Serial.print(a.acceleration.x - 0.79); Serial.print(" Y: "); Serial.print(a.acceleration.y + 0.01); Serial.print(" Z: "); Serial.println(a.acceleration.z - 9.08); // Adjusted for gravity and calibration (I got the first 100 readings and averaged them to find the offsets)
    Serial.println("Gyro:");
    Serial.print("X: "); Serial.print((g.gyro.x * 57.2958) + 3.54); Serial.print(" Y: "); Serial.print((g.gyro.y * 57.2958) - 0.12); Serial.print(" Z: "); Serial.println((g.gyro.z * 57.2958) + 0.29); // Convert from rad/s to deg/s and adjust for calibration (I got the first 100 readings and averaged them to find the offsets)
    delay(200); // Delay for readability
}

