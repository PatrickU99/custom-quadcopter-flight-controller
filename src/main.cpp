
#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "mpu_setup.h"
#include "rate_pid.h"
#include "angle_pid.h"
 
#define BUTTON_D 1 // Pin for the button to start/stop the motors 
#define BUTTON_A 42
#define BUTTON_B 41
bool running = false; // becomes false once 'p' is pressed
static bool lastButtonState = true; // Track the last state of the buttonD to detect changes
static bool lastButtonStateA = true; // Track the last state of the buttonA to detect changes
static bool lastButtonStateB = true; // Track the last state of the buttonB to detect changes

int speed = 1300;

void setup() {
   
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) { }
  
  pinMode(BUTTON_D, INPUT_PULLDOWN); // Enable internal pull-down resistor for the button pin
  pinMode(BUTTON_A, INPUT_PULLDOWN);
  pinMode(BUTTON_B, INPUT_PULLDOWN);
  mpuSetup(); // Initialize the MPU6050 sensor
  median_offset(); // Calculate the median offsets for the accelerometer
  
  motorSetup(); // Initialize the motors`
  
  
  
  delay(4000);
  mpu.getEvent(&a, &g, &temp);
  gyro_last_update = micros();
  float current_pitch = (g.gyro.x * 57.2958) + 3.49; // Convert from rad/s to deg/s
  Serial.printf("Current Setup Pitch: %.4f\n", current_pitch);
  Serial.println("Armed. Press 's' to start. and press p to stop.");
  
}
 
void loop() {
  
  
  bool buttonPressed = (digitalRead(BUTTON_D) == HIGH); // Check if the button is pressed (active HIGH)
  bool accelerate_button = (digitalRead(BUTTON_A) == HIGH);
  bool decelerate_button = (digitalRead(BUTTON_B) == HIGH);  
    if (buttonPressed && !lastButtonState) {
      running = !running; // Toggle the running state when the button is pressed
      ledcWrite(PWM_CHANNEL1, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL2, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL3, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL4, usToDuty(ARM_US));
      integral_rate_roll = 0; // Reset the integral term when stopping
      last_error_rate_roll = 0; // Reset the last error when stopping
      integral_rate_pitch = 0; // Reset the integral term for pitch when stopping
      last_error_rate_pitch = 0; // Reset the last error for pitch when stopping
      integral_rate_yaw = 0; // Reset the integral term for pitch when stopping
      last_error_rate_yaw = 0; // Reset the last error for pitch when stopping

      integral_angle_roll = 0; // Reset the integral term when stopping
      last_error_angle_roll = 0; // Reset the last error when stopping
      integral_angle_pitch = 0; // Reset the integral term for pitch when stopping
      last_error_angle_pitch = 0; // Reset the last error for pitch when stopping

      speed = 1630;

      if (running) {
        gyro_last_update = micros();
      } 
      Serial.println(running ? "Started." : "Stopped. Motor at idle.");
    }
  
 
  if (running) {
    main_pid(speed, 0, 0, 0);
    if (accelerate_button && !lastButtonStateA) {
      speed = constrain(speed + 10, 1000, 2000);
    } else if (decelerate_button && !lastButtonStateB) {
      speed = constrain(speed - 10, 1000, 2000);
    }
  lastButtonStateB = decelerate_button;
  lastButtonStateA = accelerate_button;
  
  }
  lastButtonState = buttonPressed;
  delay(3); // Small delay to avoid bouncing issues with the button
  

}