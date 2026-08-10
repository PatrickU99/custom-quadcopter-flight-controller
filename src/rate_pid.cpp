#include <Arduino.h>
#include <Wire.h>
#include "mpu.h"
#include <Adafruit_MPU6050.h>
#include "rate_pid.h"
#include "motor.h"

#define KP .3
#define KI .003


float dt;
float integral_roll = 0; // Integral term for roll PID
float output_roll; // Output of the roll PID controller
float proportional_roll; // Proportional term for roll PID
float motor_side_a; // Command to the motor based on PID output
float motor_side_b; // Command to the motor based on PID output
float roll_correction; // Correction value for roll based on PID output

void rate_roll_pid(float desired_roll, int speed) {
    dt = (micros() - gyro_last_update) / 1000000.0; // Calculate the time difference in seconds
    
    mpu.getEvent(&a, &g, &temp);
    float current_roll = (g.gyro.x * 57.2958) + 3.54; // Convert from rad/s to deg/s
    Serial.printf("Current Roll: %.2f, Desired Roll: %.2f\n", current_roll, desired_roll); // Print the current and desired roll values
    
    float error = desired_roll - current_roll; // Calculate the error between desired and current roll
    proportional_roll = KP * error; // Calculate the proportional term
    if (speed + proportional_roll + (KI * (error * dt + integral_roll)) < 2000 
    && speed + proportional_roll + (KI * (error * dt + integral_roll)) > 1000 
    && speed - proportional_roll + (KI * (error * dt + integral_roll)) < 2000 
    && speed - proportional_roll + (KI * (error * dt + integral_roll)) > 1000) {
        
        integral_roll += error * dt; // Update the integral term only if within bounds
    }

    roll_correction = proportional_roll + (KI * integral_roll); // Calculate the motor command based on PID output
    motor_side_a = speed - roll_correction; // Command for one side of the motor
    motor_side_b = speed + roll_correction; // Command for the other side of the
    
    motor_side_a =constrain(motor_side_a, 1000, 2000);
    motor_side_b =constrain(motor_side_b, 1000, 2000); // Constrain the motor command to be within the valid range
    gyro_last_update = micros();
}

void rate_roll_loop(int speed, float desired_roll) {

    rate_roll_pid(desired_roll, speed); // Call the PID controller with a desired roll of 0 degrees and the given speed
    ledcWrite(PWM_CHANNEL1, usToDuty(motor_side_a));
    ledcWrite(PWM_CHANNEL2, usToDuty(motor_side_b)); // Add a small offset to one motor to account for any imbalance
    ledcWrite(PWM_CHANNEL3, usToDuty(motor_side_b));
    ledcWrite(PWM_CHANNEL4, usToDuty(motor_side_a));
}


