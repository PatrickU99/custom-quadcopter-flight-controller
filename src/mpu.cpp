#include <Arduino.h>
#include <Wire.h>
#include "mpu.h"
#include <Adafruit_MPU6050.h>
#include <queue>

Adafruit_MPU6050 mpu;// Create an instance of the MPU6050 class
sensors_event_t a, g, temp; // Accelerometer, Gyroscope, Temperature events

// Adjusted for gravity (9.81 m/s^2) and calibration (I got the first 100 readings and averaged them to find the offsets)
float acceleration_x_offset; 
float acceleration_y_offset; 
float acceleration_z_offset; 

float acceleration_x_filtered; // Filtered acceleration values after median filtering
float acceleration_y_filtered;
float acceleration_z_filtered;

// adjusted for calibration (I got the first 100 readings and averaged them to find the offsets)
float gyro_x_offset = 3.54; 
float gyro_y_offset = -0.12; 
float gyro_z_offset = 0.29; 

float rad_to_deg = 57.2958; // Conversion factor from radians to degrees

float gyro_part = 0.995; // Complementary filter coefficient for gyro
float accel_part = 0.005; // Complementary filter coefficient for accelerometer

float delta_t; // Time difference between gyro readings in seconds
uint32_t gyro_last_update; // Store the last update time for gyro readings

std::vector<float> vector_x; // Vectors to hold the last 11 readings for median filtering
std::vector<float> vector_y; 
std::vector<float> vector_z; 

std::vector<float> sorted_vector_x; // Vectors to hold the sorted readings for median filtering
std::vector<float> sorted_vector_y; 
std::vector<float> sorted_vector_z;

int counter = 0; // Counter for the number of readings taken for median filtering recently. It will be used to index into the vectors and reset after 10 readings to maintain a rolling window of the last 11 readings.
bool error;

// Structures to hold the gyro rates and angles
struct Rates {
    float x;
    float y;
    float z;
};
struct Angles {
    float x;
    float y;
    float z;
};

Rates rates; // Structure to hold the gyro rates in degrees per second
Angles gyro_angles; // for angle approximations
Angles accel_angles; // for angle approximations
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


void rotational_rates() {
    mpu.getEvent(&a, &g, &temp);
    rates.x = (g.gyro.x * rad_to_deg) + gyro_x_offset; // Convert from rad/s to deg/s and adjust for calibration
    rates.y = (g.gyro.y * rad_to_deg) + gyro_y_offset;
    rates.z = (g.gyro.z * rad_to_deg) + gyro_z_offset;

    delta_t = (micros() - gyro_last_update) / 1000000.0; // Calculate time difference in seconds
    gyro_angles.x += rates.x * delta_t; // Calculate the change in angle for each axis
    gyro_angles.y += rates.y * delta_t;
    gyro_angles.z += rates.z * delta_t;
    
    gyro_last_update = micros(); // Update the last update time
    
}

void median_offset(){
    for (int i = 0; i < 11; i++){
        mpu.getEvent(&a, &g, &temp);
        vector_x.push_back(a.acceleration.x);
        vector_y.push_back(a.acceleration.y);
        vector_z.push_back(a.acceleration.z);
        delay(10); // Delay to allow for sensor stabilization
    }
    // Sort the vectors to find the median values for offset calculation
    sorted_vector_x = vector_x;
    std::sort(sorted_vector_x.begin(), sorted_vector_x.end());
    sorted_vector_y = vector_y;
    std::sort(sorted_vector_y.begin(), sorted_vector_y.end());
    sorted_vector_z = vector_z;
    std::sort(sorted_vector_z.begin(), sorted_vector_z.end());
    
    // Calculate the offsets based on the median values of the sorted vectors
    acceleration_x_offset = 0 - *std::next(sorted_vector_x.begin(), 5);
    acceleration_y_offset = 0 - *std::next(sorted_vector_y.begin(), 5);
    acceleration_z_offset = 9.81 - *std::next(sorted_vector_z.begin(), 5);
    
};

void median_filterization(){
    mpu.getEvent(&a, &g, &temp);
    
    // Store the current accelerometer readings in the vectors for median filtering
    vector_x[counter] = a.acceleration.x;
    vector_y[counter] = a.acceleration.y;
    vector_z[counter] = a.acceleration.z;
    
    // Sort the vectors to find the median values for filtering
    sorted_vector_x = vector_x;
    std::sort(sorted_vector_x.begin(), sorted_vector_x.end());
    sorted_vector_y = vector_y;
    std::sort(sorted_vector_y.begin(), sorted_vector_y.end());
    sorted_vector_z = vector_z;
    std::sort(sorted_vector_z.begin(), sorted_vector_z.end());

    // Calculate the filtered acceleration values based on the median of the sorted vectors and apply the offsets
    acceleration_x_filtered = *std::next(sorted_vector_x.begin(), 5) + acceleration_x_offset;
    acceleration_y_filtered = *std::next(sorted_vector_y.begin(), 5) + acceleration_y_offset;
    acceleration_z_filtered = *std::next(sorted_vector_z.begin(), 5) + acceleration_z_offset;
    
    // Calculate the accelerometer angles based on the filtered acceleration values
    accel_angles.x = atan2(acceleration_y_filtered, acceleration_z_filtered) * rad_to_deg; // Calculate roll angle
    accel_angles.y = atan2(-acceleration_x_filtered, sqrt(acceleration_y_filtered * acceleration_y_filtered + acceleration_z_filtered * acceleration_z_filtered)) * rad_to_deg; // Calculate pitch
    accel_angles.z = 0; // Yaw angle cannot be determined from accelerometer data
    
    if (counter < 10) {
        counter++;
    } else {
        counter = 0;
    }
};

void complimentary_filter(){
    rotational_rates();
    median_filterization();
    gyro_angles.x = gyro_part * gyro_angles.x + accel_part * accel_angles.x; // Apply complementary filter for roll
    gyro_angles.y = gyro_part * gyro_angles.y + accel_part * accel_angles.y; // Apply complementary filter for pitch
    gyro_angles.z = gyro_angles.z; 

    Serial.printf("Filtered Angles: X: %.2f, Y: %.2f, Z: %.2f\n", gyro_angles.x, gyro_angles.y, gyro_angles.z); // Print the filtered angles
};

