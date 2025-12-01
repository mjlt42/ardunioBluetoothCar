#include "Arduino.h"
#include <Wire.h>
#include "MPU.h"

long accelX, accelY;
float gForceX, gForceY, maxGX = 0, maxGY = 0;

void setupMPU() {
 
  Wire.begin();
  Wire.beginTransmission(0b1101000);  //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                   //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);             //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1B);                   //Accessing the register 1B - Gyroscope Configuration(Sec.4.4)
  Wire.write(0x00000000);             //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1C);                   //Accessing the register 1C - Acccelerometer Configuration(Sec.4.5)
  Wire.write(0b00000000);             //Setting the accel to +/- 2g
  Wire.endTransmission();
}

/*
void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(100);
}
*/

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x3B);                   //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 4);  //Request Accel Registers
  while (Wire.available() < 4)
    ;
  accelX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
    accelY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  processAccelData();
}
void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;

  maxAccel(gForceX, &maxGX);
  maxAccel(gForceY, &maxGY);

  Serial.println("gForceX = ");
  Serial.print(gForceX);
  Serial.println("gForceY = ");
  Serial.print(gForceY);

}

//Thanks to Matthew Wiard for helping me make a function of this using floats
void maxAccel(float Accel, float* maxG){
  if (Accel > *maxG) *maxG = Accel;
}




/*
void printData() {
  
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}
 */