/*
 * Drive.cpp
 *
 * Description:
 * Source file implementing functions for controlling a dual-motor drive system.

 * Developed by Mechanismic Inc.
 * Modified for improved motor power scaling and smoother joystick sensitivity.
 */
#include "Arduino.h"
#include "Drive.h"

// Adjustable motor power scaling factors
float leftMotorPowerScale = 1.0;  // Adjust the power of the left motor (e.g., 1.0 for 100%)
float rightMotorPowerScale = 1.0;   // Adjust the power of the right motor (e.g., 1.0 for 100%)

// Initialize motor pins as outputs
void setMotorPins() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  //Overclocks pins 9 and 10 
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
  ICR1 = 249;
  OCR1A = 125;
  OCR1B = 125;
}

// Converts throttle and steering inputs into PWM signals for motors
void drive(int throttle, int steering, int checkBlue) {
  // Print Debug Info
  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print("\tSteering: ");
  Serial.println(steering);

  // Apply exponential sensitivity adjustment for smoother joystick control
  if (checkBlue == HIGH) {
    throttle = adjustSensitivity(throttle, 2.0);  // Adjust throttle sensitivity (2.0 = medium)
    steering = adjustSensitivity(steering, 2.0);  // Adjust steering sensitivity (1.5 = medium)
  }

  // Brake if throttle is zero
  if (throttle == 0) {
    motorBrake();
    return;
  }

  // Set motor direction based on throttle value
  if (throttle > 0) {
    motorSetForward();  // Forward
  } else {
    motorSetBackward();  // Backward
  }

  // Map throttle to PWM range
  int mappedSpeed = map(abs(throttle), 0, 512, MINIMUM_MOTOR_SPEED, 255);
  int reducedSpeed = map(abs(steering), 0, 512, mappedSpeed, MINIMUM_MOTOR_SPEED);

  // Calculate motor speeds based on steering
  int leftMotorSpeed, rightMotorSpeed;
  if (steering > 0) {
    // Turn right: reduce right motor speed
    leftMotorSpeed = mappedSpeed;
    rightMotorSpeed = reducedSpeed;
  } else {
    // Turn left: reduce left motor speed
    leftMotorSpeed = reducedSpeed;
    rightMotorSpeed = mappedSpeed;
  }

  // Scale motor speeds by power factors
  leftMotorSpeed *= leftMotorPowerScale;
  rightMotorSpeed *= rightMotorPowerScale;

  // Ensure motor speeds are within valid range
  leftMotorSpeed = constrain(leftMotorSpeed, MINIMUM_MOTOR_SPEED, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, MINIMUM_MOTOR_SPEED, 255);

  // Apply motor speeds
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  // Debug Info
  Serial.print("Mapped Speed: ");
  Serial.print(mappedSpeed);
  Serial.print("\tReduced Speed: ");
  Serial.print(reducedSpeed);
  Serial.print("\tLeft Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("\tRight Motor Speed: ");
  Serial.println(rightMotorSpeed);
}

// Configures the motor controller to stop the motors
void motorBrake() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.println("Motors: Braked");
}

// Configures the motor controller to have the robot move forward
void motorSetForward() {
  /*if (crashDetect == HIGH){ //  if (crashDetect == HIGH && checkDistance(pinTrig, pinEcho, delayTime) >= 10.0){
  motorBrake();
  Serial.println("Crash Detect");
  Serial.println("Distance: ");
  }*/
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Motors: Forward");

}

// Configures the motor controller to have the robot move backward
void motorSetBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.println("Motors: Backward");
}


void driveROT(int throttleL, int throttleR, int checkBlue) {
  // Print Debug Info
  Serial.print("Throttle Right: ");
  Serial.print(throttleR);
  Serial.print("\tThrottle Left: ");
  Serial.print(throttleL);
  Serial.println();

  // Check if both throttles are zero (stop the motors)
  if (throttleR == 0 && throttleL == 0) {
    motorBrake();
    return;
  }

  // Variables for motor speeds
  int rightMotorSpeed = map(abs(throttleR), 0, 512, MINIMUM_MOTOR_SPEED, 255);
  int leftMotorSpeed = map(abs(throttleL), 0, 512, MINIMUM_MOTOR_SPEED, 255);

  // Scale motor speeds by power factors
  rightMotorSpeed *= rightMotorPowerScale;
  leftMotorSpeed *= leftMotorPowerScale;

  // Ensure motor speeds are within valid range
  rightMotorSpeed = constrain(rightMotorSpeed, MINIMUM_MOTOR_SPEED, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, MINIMUM_MOTOR_SPEED, 255);

  // Determine motor directions for pivoting
  if (throttleR > 0 && throttleL < 0) { 
    // Pivot to the right (right wheel forward, left wheel backward)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Pivoting Right");
  } else if (throttleR < 0 && throttleL > 0) { 
    // Pivot to the left (right wheel backward, left wheel forward)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Pivoting Left");
  }

  // Apply motor speeds
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);

  // Debug Info
  Serial.print("Left Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("\tRight Motor Speed: ");
  Serial.println(rightMotorSpeed);
}

// Adjusts the sensitivity of the joystick inputs using an exponential curve
int adjustSensitivity(int value, float factor) {
  // Apply exponential sensitivity curve for smoother response
  float normalizedValue = (float)value / 512.0;  // Normalize to range -1.0 to 1.0
  float adjustedValue = pow(abs(normalizedValue), factor) * (normalizedValue < 0 ? -1 : 1);
  return (int)(adjustedValue * 512.0);  // Scale back to original range
}