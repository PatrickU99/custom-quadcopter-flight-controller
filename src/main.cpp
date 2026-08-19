
#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "mpu.h"
#include "rate_pid.h"
 
#define BUTTON_A 1 // Pin for the button to start/stop the motors 
bool running = false; // becomes false once 'p' is pressed
static bool lastButtonState = HIGH; // Track the last state of the button to detect changes

void setup() {
   
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) { }
  
  pinMode(BUTTON_A, INPUT_PULLDOWN); // Enable internal pull-up resistor for the button pin
  mpuSetup(); // Initialize the MPU6050 sensor
  //median_offset(); // Calculate the median offsets for the accelerometer
  
  motorSetup(); // Initialize the motors`
  
  
  
  delay(4000);
  mpu.getEvent(&a, &g, &temp);
  gyro_last_update = micros();
  float current_pitch = (g.gyro.x * 57.2958) + 3.49; // Convert from rad/s to deg/s
  Serial.printf("Current Setup Pitch: %.4f\n", current_pitch);
  Serial.println("Armed. Press 's' to start. and press p to stop.");
  
}
 
void loop() {
  //complimentary_filter(); // Apply the complementary filter to combine gyro and accelerometer data
  
  bool buttonPressed = (digitalRead(BUTTON_A) == HIGH); // Check if the button is pressed (active HIGH)
    if (buttonPressed && !lastButtonState) {
      running = !running; // Toggle the running state when the button is pressed
      ledcWrite(PWM_CHANNEL1, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL2, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL3, usToDuty(ARM_US));
      ledcWrite(PWM_CHANNEL4, usToDuty(ARM_US));
      integral_roll = 0; // Reset the integral term when stopping
      last_error_roll = 0; // Reset the last error when stopping
      integral_pitch = 0; // Reset the integral term for pitch when stopping
      last_error_pitch = 0; // Reset the last error for pitch when stopping
      integral_yaw = 0; // Reset the integral term for pitch when stopping
      last_error_yaw = 0; // Reset the last error for pitch when stopping


      if (running) {
        gyro_last_update = micros();
      } 
      Serial.println(running ? "Started." : "Stopped. Motor at idle.");
    }
  
 
  if (running) {
    rate_loop(1500, 0, 0, 0); // Call the rate loop function with the desired speed and roll
  }
  lastButtonState = buttonPressed;
  delay(3); // Small delay to avoid bouncing issues with the button
  

}