#include <Arduino.h>
#include <Wire.h>
#include "mpu_setup.h"
#include <Adafruit_MPU6050.h>
#include "rate_pid.h"
#include "motor.h"
#include "angle_pid.h"

#define KPR 1
#define KIR 0.15
#define KDR 0.045

#define KPP 1
#define KIP 0.15
#define KDP 0.045

#define KPY 1.5
#define KIY 0.225
#define KDY 0.05

//offsets so that each motor is calibrated to the same rates
const int TRIM1 = 0;
const int TRIM2 = -300;
const int TRIM3 = -40;
const int TRIM4 = -205;


float integral_rate_roll = 0; // Integral term for roll PID
float proportional_rate_roll; // Proportional term for roll PID
float derivative_rate_roll; // Derivative term for roll PID
float roll_rate_correction; // Correction value for roll based on PID output
float last_error_rate_roll = 0; // Last error value for roll PID

float pitch_rate_correction; // Correction value for pitch based on PID output
float integral_rate_pitch = 0; // Integral term for pitch PID
float proportional_rate_pitch; // Proportional term for pitch PID
float derivative_rate_pitch; // Derivative term for pitch PID
float last_error_rate_pitch = 0; // Last error value for pitch PID

float yaw_rate_correction; // Correction value for pitch based on PID output
float integral_rate_yaw = 0; // Integral term for pitch PID
float proportional_rate_yaw; // Proportional term for pitch PID
float derivative_rate_yaw; // Derivative term for pitch PID
float last_error_rate_yaw = 0; // Last error value for pitch PID

float current_pitch_rate = 0.0;
float current_yaw_rate = 0.0;
float current_roll_rate = 0.0;

float roll_side_a; // Command to the motor based on PID output
float roll_side_b; // Command to the motor based on PID output
float pitch_side_a; // Command to the motor based on PID output
float pitch_side_b; // Command to the motor based on PID output
float yaw_side_a; // Command to the motor based on PID output
float yaw_side_b; // Command to the motor based on PID output

float last_scale = 1.0;

void rate_roll_pid(float desired_roll, int speed) {
    

    
    current_roll_rate = (g.gyro.y * 57.2958) + 0.15; // Convert from rad/s to deg/s
    
    
    float error = desired_roll - current_roll_rate; // Calculate the error between desired and current roll
    proportional_rate_roll = KPR * error; // Calculate the proportional term
    
    if (last_scale >= 0.99 && speed + proportional_rate_roll + (KIR * (error * dt + integral_rate_roll)) < 2000 
    && speed + proportional_rate_roll + (KIR * (error * dt + integral_rate_roll)) > 1000 
    && speed - proportional_rate_roll + (KIR * (error * dt + integral_rate_roll)) < 2000 
    && speed - proportional_rate_roll + (KIR * (error * dt + integral_rate_roll)) > 1000) {
        
        integral_rate_roll += error * dt; // Update the integral term only if within bounds
    }

    derivative_rate_roll = KDR * ((error - last_error_rate_roll) / dt); // Calculate the derivative term


    roll_rate_correction = proportional_rate_roll + (KIR * integral_rate_roll) + derivative_rate_roll ; // Calculate the motor command based on PID output
    roll_side_a = -roll_rate_correction; // Command for one side of the motor
    roll_side_b = roll_rate_correction; // Command for the other side of the
    
    last_error_rate_roll = error; // Update the last error value for the next iteration
   
}

void rate_pitch_pid(float desired_pitch, int speed) {
    

    current_pitch_rate = (g.gyro.x * 57.2958) + 3.49; // Convert from rad/s to deg/s
    
    
    float error = desired_pitch - current_pitch_rate; // Calculate the error between desired and current pitch
    proportional_rate_pitch = KPP * error; // Calculate the proportional term
    
    if (last_scale >= 0.99 && speed + proportional_rate_pitch + (KIP * (error * dt + integral_rate_pitch)) < 2000 
    && speed + proportional_rate_pitch + (KIP * (error * dt + integral_rate_pitch)) > 1000 
    && speed - proportional_rate_pitch + (KIP * (error * dt + integral_rate_pitch)) < 2000 
    && speed - proportional_rate_pitch + (KIP * (error * dt + integral_rate_pitch)) > 1000) {
        
        integral_rate_pitch += error * dt; // Update the integral term only if within bounds
    }

    derivative_rate_pitch = KDP * ((error - last_error_rate_pitch) / dt); // Calculate the derivative term


    pitch_rate_correction = proportional_rate_pitch + (KIP * integral_rate_pitch) + derivative_rate_pitch ; // Calculate the motor command based on PID output
    pitch_side_a = -pitch_rate_correction; // Command for one side of the motor
    pitch_side_b = pitch_rate_correction; // Command for the other side of the
    
    last_error_rate_pitch = error; // Update the last error value for the next iteration
    
}

void rate_yaw_pid(float desired_yaw, int speed) {
    

    current_yaw_rate = (g.gyro.z * 57.2958); // Convert from rad/s to deg/s
    
    float error = desired_yaw - current_yaw_rate; // Calculate the error between desired and current pitch
    proportional_rate_yaw = KPY * error; // Calculate the proportional term
    
    if (last_scale >= 0.99 && speed + proportional_rate_yaw + (KIY * (error * dt + integral_rate_yaw)) < 2000 
    && speed + proportional_rate_yaw + (KIY * (error * dt + integral_rate_yaw)) > 1000 
    && speed - proportional_rate_yaw + (KIY * (error * dt + integral_rate_yaw)) < 2000 
    && speed - proportional_rate_yaw + (KIY * (error * dt + integral_rate_yaw)) > 1000) {
        
        integral_rate_yaw += error * dt; // Update the integral term only if within bounds
    }

    derivative_rate_yaw = KDY * ((error - last_error_rate_yaw) / dt); // Calculate the derivative term


    yaw_rate_correction = proportional_rate_yaw + (KIY * integral_rate_yaw) + derivative_rate_yaw; // Calculate the motor command based on PID output
    yaw_side_a = -yaw_rate_correction; // Command for one side of the motor
    yaw_side_b = yaw_rate_correction; // Command for the other side of the
    
    last_error_rate_yaw = error; // Update the last error value for the next iteration
    
}


void rate_loop(int speed, float desired_roll, float desired_pitch, float desired_yaw) {
    
    rate_roll_pid(desired_roll, speed); // Call the PID controller with a desired roll of 0 degrees and the given speed
    rate_pitch_pid(desired_pitch, speed); // Call the PID controller with a desired pitch of 0 degrees and the given speed
    rate_yaw_pid(desired_yaw, speed);
    
    // Compute each motor's correction (offset from base speed), UNCLAMPED
    float corr1 = roll_side_b + pitch_side_a + yaw_side_a;
    float corr2 = roll_side_a + pitch_side_a + yaw_side_b;
    float corr3 = roll_side_a + pitch_side_b + yaw_side_a;
    float corr4 = roll_side_b + pitch_side_b + yaw_side_b;
 
    // Available headroom above/below the base speed before hitting 1000/2000
    float headroomAbove = 2000 - speed;
    float headroomBelow = speed - 1000;
 
    // Find the largest positive and largest negative correction demanded
    float maxPos = max(max(corr1, corr2), max(corr3, corr4));
    float maxNeg = min(min(corr1, corr2), min(corr3, corr4)); // negative or zero
 
    // Compute scale factor needed to keep every motor within range (never scale UP, only down)
    float scale = 1.0;
    if (maxPos > headroomAbove && maxPos > 0) {
        scale = min(scale, headroomAbove / maxPos);
    }
    if (maxNeg < -headroomBelow && maxNeg < 0) {
        scale = min(scale, -headroomBelow / maxNeg); // both negative, ratio is positive
    }
 
    // Apply the same scale to all four corrections - preserves the ratio between axes
    corr1 *= scale;
    corr2 *= scale;
    corr3 *= scale;
    corr4 *= scale;
 
    int m1 = constrain((int)(speed + corr1 + TRIM1), 1000, 2000); // final safety net
    int m2 = constrain((int)(speed + corr2 + TRIM2), 1000, 2000);
    int m3 = constrain((int)(speed + corr3 + TRIM3), 1000, 2000);
    int m4 = constrain((int)(speed + corr4 + TRIM4), 1000, 2000);
 
    ledcWrite(PWM_CHANNEL1, usToDuty(m1));
    ledcWrite(PWM_CHANNEL2, usToDuty(m2));
    ledcWrite(PWM_CHANNEL3, usToDuty(m3));
    ledcWrite(PWM_CHANNEL4, usToDuty(m4));
    
    last_scale = scale;
    Serial.printf("Yaw: %.4f Scale: %.2f\n", 
              current_yaw_rate, scale);
}
