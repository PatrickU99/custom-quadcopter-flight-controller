#include <Arduino.h>
#include <Wire.h>
#include "mpu_setup.h"
#include <Adafruit_MPU6050.h>
#include "angle_pid.h"
#include <queue>
#include "rate_pid.h"


// Adjusted for gravity (9.81 m/s^2) and calibration (I got the first 100 readings and averaged them to find the offsets)
float acceleration_x_offset; 
float acceleration_y_offset; 
float acceleration_z_offset; 

float acceleration_x_filtered; // Filtered acceleration values after median filtering
float acceleration_y_filtered;
float acceleration_z_filtered;

// adjusted for calibration (I got the first 100 readings and averaged them to find the offsets)
float gyro_x_offset = 3.64; 
float gyro_y_offset = 0; 
float gyro_z_offset = 0.29; 

float rad_to_deg = 57.2958; // Conversion factor from radians to degrees

float gyro_part = 0.995; // Complementary filter coefficient for gyro
float accel_part = 0.005; // Complementary filter coefficient for accelerometer

uint32_t gyro_last_update; // Store the last update time for gyro readings

std::vector<float> vector_x; // Vectors to hold the last 11 readings for median filtering
std::vector<float> vector_y; 
std::vector<float> vector_z; 

std::vector<float> sorted_vector_x; // Vectors to hold the sorted readings for median filtering
std::vector<float> sorted_vector_y; 
std::vector<float> sorted_vector_z;

int counter = 0; // Counter for the number of readings taken for median filtering recently. It will be used to index into the vectors and reset after 10 readings to maintain a rolling window of the last 11 readings.


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

#define APR 0
#define AIR 0
#define ADR 0

#define APP 0
#define AIP 0
#define ADP 0

float dt;
float current_roll_angle;
float current_pitch_angle;

float integral_angle_roll = 0; // Integral term for roll PID
float proportional_angle_roll; // Proportional term for roll PID
float derivative_angle_roll; // Derivative term for roll PID
float roll_angle_correction; // Correction value for roll based on PID output
float last_error_angle_roll = 0; // Last error value for roll PID

float pitch_angle_correction; // Correction value for pitch based on PID output
float integral_angle_pitch = 0; // Integral term for pitch PID
float proportional_angle_pitch; // Proportional term for pitch PID
float derivative_angle_pitch; // Derivative term for pitch PID
float last_error_angle_pitch = 0; // Last error value for pitch PID

void rotational_rates() {
    mpu.getEvent(&a, &g, &temp);
    rates.x = (g.gyro.x * rad_to_deg) + gyro_x_offset; // Convert from rad/s to deg/s and adjust for calibration
    rates.y = (g.gyro.y * rad_to_deg) + gyro_y_offset;
    rates.z = (g.gyro.z * rad_to_deg) + gyro_z_offset;

    gyro_angles.x += rates.x * dt; // Calculate the change in angle for each axis
    gyro_angles.y += rates.y * dt;
    gyro_angles.z += rates.z * dt;
    
    
    
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

void angle_roll_pid(float desired_roll, int speed) {
    
    current_roll_angle = gyro_angles.y; // Convert from rad/s to deg/s
    
    float error = desired_roll - current_roll_angle; // Calculate the error between desired and current roll
    proportional_angle_roll = APR * error; // Calculate the proportional term
    
    if (speed + proportional_angle_roll + (AIR * (error * dt + integral_angle_roll)) < 2000 
    && speed + proportional_angle_roll + (AIR * (error * dt + integral_angle_roll)) > 1000 
    && speed - proportional_angle_roll + (AIR * (error * dt + integral_angle_roll)) < 2000 
    && speed - proportional_angle_roll + (AIR * (error * dt + integral_angle_roll)) > 1000) {
        
        integral_angle_roll += error * dt; // Update the integral term only if within bounds
    }

    derivative_angle_roll = ADR * ((error - last_error_angle_roll) / dt); // Calculate the derivative term


    roll_angle_correction = proportional_angle_roll + (AIR * integral_angle_roll) + derivative_angle_roll ; // Calculate the motor command based on PID output
    
    last_error_angle_roll = error; // Update the last error value for the next iteration
    
};
void angle_pitch_pid(float desired_pitch, int speed) {

    current_pitch_angle = gyro_angles.x; // Convert from rad/s to deg/s
    
    float error = desired_pitch - current_pitch_angle; // Calculate the error between desired and current pitch
    proportional_angle_pitch = APP * error; // Calculate the proportional term
    
    if (speed + proportional_angle_pitch + (AIP * (error * dt + integral_angle_pitch)) < 2000 
    && speed + proportional_angle_pitch + (AIP * (error * dt + integral_angle_pitch)) > 1000 
    && speed - proportional_angle_pitch + (AIP * (error * dt + integral_angle_pitch)) < 2000 
    && speed - proportional_angle_pitch + (AIP * (error * dt + integral_angle_pitch)) > 1000) {
        
        integral_angle_pitch += error * dt; // Update the integral term only if within bounds
    }

    derivative_angle_pitch = ADP * ((error - last_error_angle_pitch) / dt); // Calculate the derivative term


    pitch_angle_correction = proportional_angle_pitch + (AIP * integral_angle_pitch) + derivative_angle_pitch ; // Calculate the motor command based on PID output

    last_error_angle_pitch = error; // Update the last error value for the next iteration
    
};

void complimentary_filter(){
    rotational_rates();
    median_filterization();
    gyro_angles.x = gyro_part * gyro_angles.x + accel_part * accel_angles.x; // Apply complementary filter for roll
    gyro_angles.y = gyro_part * gyro_angles.y + accel_part * accel_angles.y; // Apply complementary filter for pitch
    gyro_angles.z = gyro_angles.z; 
    Serial.printf("Filtered Angles: X: %.2f, Y: %.2f, Roll_Correction: %.4f, Pitch_Correction: %.4f\n", gyro_angles.x, gyro_angles.y, roll_angle_correction, pitch_angle_correction); // Print the filtered angles
};

void main_pid(int speed, float desired_roll, float desired_pitch, float desired_yaw) {
    dt = (micros() - gyro_last_update) / 1000000.0; // Calculate the time difference in seconds
    gyro_last_update = micros(); // Update the last update time
    complimentary_filter();
    angle_roll_pid(desired_roll, speed);
    angle_pitch_pid(desired_pitch, speed);
    rate_loop(speed, roll_angle_correction, pitch_angle_correction, desired_yaw);
};
