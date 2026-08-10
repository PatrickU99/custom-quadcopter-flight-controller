
#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "mpu.h"
#include "rate_pid.h"
 
#define BUTTON_A 1 // Pin for the button to start/stop the motors 
bool running = false; // becomes false once 'p' is pressed
static bool lastButtonState = HIGH; 

void setup() {
   
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000) { }
  
  pinMode(BUTTON_A, INPUT_PULLDOWN); // Enable internal pull-up resistor for the button pin
  mpuSetup(); // Initialize the MPU6050 sensor
  //median_offset(); // Calculate the median offsets for the accelerometer
  
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL4, PWM_FREQ, PWM_RES);
  
  ledcAttachPin(MOTOR_PIN1, PWM_CHANNEL1);
  ledcAttachPin(MOTOR_PIN2, PWM_CHANNEL2);
  ledcAttachPin(MOTOR_PIN3, PWM_CHANNEL3);
  ledcAttachPin(MOTOR_PIN4, PWM_CHANNEL4);
  
  Serial.println("Arming... hold clear of the motor.");
  
  ledcWrite(PWM_CHANNEL1, usToDuty(ARM_US));
  ledcWrite(PWM_CHANNEL2, usToDuty(ARM_US));
  ledcWrite(PWM_CHANNEL3, usToDuty(ARM_US));
  ledcWrite(PWM_CHANNEL4, usToDuty(ARM_US));
  
  delay(4000);
  gyro_last_update = micros();
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
      Serial.println("Stopped. Motor at idle.");
    }
  
 
  if (running) {
    
    rate_roll_loop(1500, 0); // Call the rate roll loop function with the desired speed and roll
  }
  lastButtonState = buttonPressed;
 delay(20); // ~50Hz refresh
  

}