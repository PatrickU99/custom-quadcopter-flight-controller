#ifndef rate_pid_h
#define rate_pid_h
#include <Arduino.h>

void rate_roll_pid(float desired_roll, int speed);

void rate_roll_loop(int speed, float desired_roll);


extern float integral_roll; // Integral term for roll PID
#endif // rate_pid_h