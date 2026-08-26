#ifndef rate_pid_h
#define rate_pid_h
#include <Arduino.h>

void rate_roll_pid(float desired_roll, int speed);

void rate_pitch_pid(float desired_pitch, int speed);

void rate_loop(int speed, float desired_roll, float desired_pitch, float desired_yaw);


extern float integral_rate_roll; // Integral term for roll PID
extern float last_error_rate_roll; // Last error value for roll PID
extern float integral_rate_pitch; // Integral term for pitch PID
extern float last_error_rate_pitch; // Last error value for pitch PID
extern float integral_rate_yaw; // Integral term for pitch PID
extern float last_error_rate_yaw; // Last error value for pitch PID

#endif // rate_pid_h