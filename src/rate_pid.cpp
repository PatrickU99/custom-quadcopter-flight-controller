#include <Arduino.h>
#include <Wire.h>
#include "mpu.h"
#include <Adafruit_MPU6050.h>
#include "rate_pid.h"
#include "motor.h"

#define KPR 2
#define KIR 0.3
#define KDR 0.065

#define KPP 2
#define KIP 0.3
#define KDP 0.065

#define KPY 3
#define KIY 0.45
#define KDY 0.1

float dt; // Time difference between gyro readings in seconds
float integral_roll = 0; // Integral term for roll PID
float proportional_roll; // Proportional term for roll PID
float derivative_roll; // Derivative term for roll PID
float roll_correction; // Correction value for roll based on PID output
float last_error_roll = 0; // Last error value for roll PID

float pitch_correction; // Correction value for pitch based on PID output
float integral_pitch = 0; // Integral term for pitch PID
float proportional_pitch; // Proportional term for pitch PID
float derivative_pitch; // Derivative term for pitch PID
float last_error_pitch = 0; // Last error value for pitch PID

float yaw_correction; // Correction value for pitch based on PID output
float integral_yaw = 0; // Integral term for pitch PID
float proportional_yaw; // Proportional term for pitch PID
float derivative_yaw; // Derivative term for pitch PID
float last_error_yaw = 0; // Last error value for pitch PID

float roll_side_a; // Command to the motor based on PID output
float roll_side_b; // Command to the motor based on PID output
float pitch_side_a; // Command to the motor based on PID output
float pitch_side_b; // Command to the motor based on PID output
float yaw_side_a; // Command to the motor based on PID output
float yaw_side_b; // Command to the motor based on PID output


void rate_roll_pid(float desired_roll, int speed) {
    Serial.printf("dt: %.4f\n", dt);

    
    float current_roll = (g.gyro.y * 57.2958) + 0.15; // Convert from rad/s to deg/s
    Serial.printf("Current Roll: %.4f\n", current_roll);
    
    float error = desired_roll - current_roll; // Calculate the error between desired and current roll
    proportional_roll = KPR * error; // Calculate the proportional term
    
    if (speed + proportional_roll + (KIR * (error * dt + integral_roll)) < 2000 
    && speed + proportional_roll + (KIR * (error * dt + integral_roll)) > 1000 
    && speed - proportional_roll + (KIR * (error * dt + integral_roll)) < 2000 
    && speed - proportional_roll + (KIR * (error * dt + integral_roll)) > 1000) {
        
        integral_roll += error * dt; // Update the integral term only if within bounds
    }

    derivative_roll = KDR * ((error - last_error_roll) / dt); // Calculate the derivative term


    roll_correction = proportional_roll + (KIR * integral_roll) + derivative_roll ; // Calculate the motor command based on PID output
    roll_side_a = -roll_correction; // Command for one side of the motor
    roll_side_b = roll_correction; // Command for the other side of the
    
    last_error_roll = error; // Update the last error value for the next iteration
    gyro_last_update = micros();
}

void rate_pitch_pid(float desired_pitch, int speed) {
    Serial.printf("dt: %.4f\n", dt);

    float current_pitch = (g.gyro.x * 57.2958) + 3.49; // Convert from rad/s to deg/s
    Serial.printf("Current Pitch: %.4f\n", current_pitch);
    
    float error = desired_pitch - current_pitch; // Calculate the error between desired and current pitch
    proportional_pitch = KPP * error; // Calculate the proportional term
    
    if (speed + proportional_pitch + (KIP * (error * dt + integral_pitch)) < 2000 
    && speed + proportional_pitch + (KIP * (error * dt + integral_pitch)) > 1000 
    && speed - proportional_pitch + (KIP * (error * dt + integral_pitch)) < 2000 
    && speed - proportional_pitch + (KIP * (error * dt + integral_pitch)) > 1000) {
        
        integral_pitch += error * dt; // Update the integral term only if within bounds
    }

    derivative_pitch = KDP * ((error - last_error_pitch) / dt); // Calculate the derivative term


    pitch_correction = proportional_pitch + (KIP * integral_pitch) + derivative_pitch ; // Calculate the motor command based on PID output
    pitch_side_a = -pitch_correction; // Command for one side of the motor
    pitch_side_b = pitch_correction; // Command for the other side of the
    
    last_error_pitch = error; // Update the last error value for the next iteration
    gyro_last_update = micros();
}

void rate_yaw_pid(float desired_yaw, int speed) {
    Serial.printf("dt: %.4f\n", dt);

    float current_yaw = (g.gyro.z * 57.2958); // Convert from rad/s to deg/s
    Serial.printf("Current Yaw: %.4f\n", current_yaw);
    
    float error = desired_yaw - current_yaw; // Calculate the error between desired and current pitch
    proportional_yaw = KPY * error; // Calculate the proportional term
    
    if (speed + proportional_yaw + (KIY * (error * dt + integral_yaw)) < 2000 
    && speed + proportional_yaw + (KIY * (error * dt + integral_yaw)) > 1000 
    && speed - proportional_yaw + (KIY * (error * dt + integral_yaw)) < 2000 
    && speed - proportional_yaw + (KIY * (error * dt + integral_yaw)) > 1000) {
        
        integral_yaw += error * dt; // Update the integral term only if within bounds
    }

    derivative_yaw = KDY * ((error - last_error_yaw) / dt); // Calculate the derivative term


    yaw_correction = proportional_yaw + (KIY * integral_yaw) + derivative_yaw; // Calculate the motor command based on PID output
    yaw_side_a = -yaw_correction; // Command for one side of the motor
    yaw_side_b = yaw_correction; // Command for the other side of the
    
    last_error_yaw = error; // Update the last error value for the next iteration
    gyro_last_update = micros();
}


void rate_loop(int speed, float desired_roll, float desired_pitch, float desired_yaw) {
    dt = (micros() - gyro_last_update) / 1000000.0; // Calculate the time difference in seconds
    mpu.getEvent(&a, &g, &temp);
    rate_roll_pid(desired_roll, speed); // Call the PID controller with a desired roll of 0 degrees and the given speed
    rate_pitch_pid(desired_pitch, speed); // Call the PID controller with a desired pitch of 0 degrees and the given speed
    rate_yaw_pid(desired_yaw, speed);
    
    ledcWrite(PWM_CHANNEL1, usToDuty(constrain(roll_side_b + pitch_side_a + yaw_side_a + speed, 1000, 2000))); 
    ledcWrite(PWM_CHANNEL2, usToDuty(constrain(roll_side_a + pitch_side_a + yaw_side_b + speed, 1000, 2000))); 
    ledcWrite(PWM_CHANNEL3, usToDuty(constrain(roll_side_a + pitch_side_b + yaw_side_a + speed, 1000, 2000))); 
    ledcWrite(PWM_CHANNEL4, usToDuty(constrain(roll_side_b + pitch_side_b + yaw_side_b + speed, 1000, 2000)));
    Serial.printf("Motor Side A: %.2f\n", constrain(roll_side_b + pitch_side_a + yaw_side_a + speed, 1000, 2000));
    Serial.printf("Motor Side B: %.2f\n", constrain(roll_side_a + pitch_side_a + yaw_side_b + speed, 1000, 2000));
    Serial.printf("Motor Side C: %.2f\n", constrain(roll_side_a + pitch_side_b + yaw_side_a + speed, 1000, 2000));
    Serial.printf("Motor Side D: %.2f\n", constrain(roll_side_b + pitch_side_b + yaw_side_b + speed, 1000, 2000));
    Serial.printf("Speed: %d\n", speed);
    
}
