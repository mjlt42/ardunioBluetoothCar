/*
 * Drive.h
 *
 * Description:
 * Header file defining functions and constants for controlling a dual-motor drive system.
 *
 * Developed by Mechanismic Inc.
 * Free for use.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include "Arduino.h"

// MOTOR LEFT PINS
const int ENA = 9;
const int IN1 = 2;
const int IN2 = 4;

// MOTOR RIGHT PINS
const int ENB = 10;
const int IN3 = 12;
const int IN4 = 13;

// MOTOR PARAMETERS
// Minimum PWM value for analogWrite to spin motor when robot is on the ground.
const int MINIMUM_MOTOR_SPEED = 70;

// Function Prototypes
void setMotorPins();
void drive(int throttle, int steering, int checkBlue);
void driveROT(int throttleR, int throttleL, int checkBlue);
void motorBrake();
void motorSetForward();
void motorSetBackward();
int adjustSensitivity(int value, float factor);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
float checkDistance(int trigPin, int echoPin, int timeDelay);

#endif