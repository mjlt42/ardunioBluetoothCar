/*
Author: Matthew Lopez Tarsky (12/11/24)

Entire program (along with .cpp and .h)

controls the car with BlueTooth, detects crashes with the ultra sonic sensor, gets acceleration data with MPU, and writes to the SD

Help from Jason Weiss and Matthew Wiard

*/
/*
 * UNO_R3_DRIVE.ino
 *
 * Description:
 * Arduino main file to control a dual-motor drive system using BLE communication.

 * Developed by Mechanismic Inc.
 * Free for use.
 */
#include <TimerOne.h>
#include "Arduino.h"
#include "MicroBlue.h"
#include "Drive.h"
#include "MPU.h"
#include "SDcard.h"
#include "CrashDetect.h"

// Define BLE communication pins and create software serial for BLE
#include "SoftwareSerial.h"

long currentTime = millis();

//BlueTooth Setup
const int rXPin = 7;
const int tXPin = 8;
SoftwareSerial SSerial(rXPin, tXPin);

//Boolean Conditions ()
int blueTrue = HIGH;    //Set to LOW to disable Bluetooth control; Set to HIGH to enable Bluetooth control; Default is HIGH
int crashDetect = LOW;  //Set to LOW to disable crash detection; Set to HIGH to enable crash detection; Default is LOW
int dataCollect = LOW;  //Set to LOW to disable data collection; Set to HIGH to enable data collection; Default is LOW

//Line Tracker Sensor
const int IRpinRD = 0;
const int IRpinLD = 0;
int IRvalueRD;
int IRvalueLD;

extern float maxGX, maxGY;

// Create an instance of the MicroBlueManager for managing messages
MicroBlueManager manager(SSerial);

// Initialize setup function
void setup() {
  Serial.begin(9600);   // Initialize USB serial communication
  SSerial.begin(9600);  // Initialize software serial for BLE communication
  setupSD();
  setupMPU();
  pinMode(IRpinRD, INPUT);
  pinMode(IRpinLD, INPUT);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  //setMotorPins();  // Configure motor pins for output
}

// Main loop to read BLE messages and control motor drive
void loop() {
  // Read a message from BLE
  MicroBlueMessage msg = manager.read();
  Serial.print("BlueTrue: ");
  Serial.print(blueTrue);
  Serial.print("\n");

  // Print message details if both ID and Value are valid
  if (msg.hasId() && msg.hasValue()) {
    Serial.println(msg.toString());
  }

  if (msg.id == "h0") {
    if (msg.value == "1") {
      dataCollect = HIGH;
    }
  }

  if (msg.id == "sw0") {
    int check = 1;
    sscanf(msg.value.c_str(), "%d", &check);
    if (check == 0) crashDetect = LOW;
    else if (check == 1) crashDetect = HIGH;
  }

  if (msg.id == "t0") {
    int check = 1;
    sscanf(msg.value.c_str(), "%d", &check);
    if (check == 0) blueTrue = LOW;
    else if (check == 1) blueTrue = HIGH;
  }

  // Check for a specific message ID to control drive system
  if (msg.id == "d1" && blueTrue == HIGH) {
    Serial.println("d1");
    int throttle, steering;

    // Parse throttle and steering values from the message value string
    sscanf(msg.value.c_str(), "%d,%d", &steering, &throttle);

    // Adjust values to center at 0 (assuming incoming range of 0-1023)
    throttle -= 512;
    steering -= 512;

    // Control motors based on parsed throttle and steering values
    drive(throttle, steering, blueTrue);
  }

  if (blueTrue == LOW) {

    //Serial.println(blueTrue);
    int throttle = 10;
    int steering = 0;
    const int steerFactor = 400;
    const int throttleFactor = 0;
    IRvalueRD = digitalRead(IRpinRD);
    IRvalueLD = digitalRead(IRpinLD);
    Serial.print("IRvalueRD: ");
    Serial.print(IRvalueRD);
    Serial.print(":");
    Serial.print(digitalRead(IRpinRD));
    Serial.print("\n");
    Serial.print("IRvalueLD: ");
    Serial.print(IRvalueLD);
    Serial.print(":");
    Serial.print(digitalRead(IRpinLD));
    Serial.print("\n");
    //delay(2000);
    /*
    if (msg.id == "sl0") {
      int value = msg.value.toInt();
      throttle = map(value, 0, 100, 0, 255);
      Serial.println("throttle: ");
      Serial.println(throttle);
    }*/
    //LOW on white; HIGH ON BLACK

    /* if (IRvalueRD == 1 && IRvalueLD == 0) {  // Right sensor detects black; Left sensor detects white
  // Apply a short high-speed burst to overcome friction (Kick-Start)
  driveROT(-throttle * 1.2, throttle * 1.2, blueTrue);  // Burst for left turn (increased speed)
  delay(500);  // Short burst duration (50 ms)
  driveROT(-throttle, throttle, blueTrue);  // Normal rotation speed after burst
} 
else if (IRvalueLD == 1 && IRvalueRD == 0) {  // Left sensor detects black; Right sensor detects white
  // Apply a short high-speed burst to overcome friction (Kick-Start)
  driveROT(throttle * 1.2, -throttle * 1.2, blueTrue);  // Burst for right turn (increased speed)
  delay(50);  // Short burst duration (50 ms)
  driveROT(throttle, -throttle, blueTrue);  // Normal rotation speed after burst
} 
else if (IRvalueRD == 0 && IRvalueLD == 0) {  // Both sensors detect white
  // Apply a short high-speed burst to get moving forward (Kick-Start)
  drive(throttle * 1.2, steering, blueTrue);  // Increased speed to start moving
  delay(50);  // Short burst duration (50 ms)
  drive(throttle, steering, blueTrue);  // Normal driving speed after burst
} 
else if (IRvalueRD == 1 && IRvalueLD == 1) {  // Both sensors detect black
  // Stop completely if both sensors detect black
  drive(0, 0, blueTrue);  // Stop the motors
}*/


    if (IRvalueLD == 0 && IRvalueRD == 1) {     //Right sensor detects black; Left sensor detects white
      driveROT(-throttle*1.1, throttle*1.1, blueTrue);  // Pivots to the left (Wheels have opposing directions)
      delay(20);
      drive(0, 0, blueTrue);
    } else if (IRvalueLD == 1 && IRvalueRD == 0) {  //Left sensor detects black; Right sensor detects white;
      driveROT(throttle*1.1, -throttle*1.1, blueTrue);      // Pivots to the right (Wheels have opposing directions)
      delay(20);
      drive(0, 0, blueTrue);
    } else if (IRvalueLD == 0 && IRvalueRD == 0) {  //Both sensors detect white
      drive(throttle, steering, blueTrue);          // Drive forward
      delay(20);
      drive(0, 0, blueTrue);
    } else if (IRvalueLD == 1 && IRvalueRD == 1) {  //Both sensors detect black
      drive(0, 0, blueTrue);                        // Stop completely
      delay(20);
      drive(0, 0, blueTrue); 
    }
  }

  if (dataCollect == HIGH) {
    recordAccelRegisters();
    runSD(maxGX, maxGY);
    dataCollect = LOW;
  }
}
