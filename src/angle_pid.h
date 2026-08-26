#ifndef ANGLE_PID_H
#define ANGLE_PID_H

void rotational_rates();
void median_offset();
void complimentary_filter();
void main_pid(int speed, float desired_roll, float desired_pitch, float desired_yaw);

extern float integral_angle_roll; // Integral term for roll PID
extern float last_error_angle_roll; // Last error value for roll PID
extern float integral_angle_pitch; // Integral term for pitch PID
extern float last_error_angle_pitch; // Last error value for pitch PID
extern float integral_angle_yaw; // Integral term for pitch PID
extern float last_error_angle_yaw; // Last error value for pitch PID
extern float dt;


#endif // ANGLE_PID_H