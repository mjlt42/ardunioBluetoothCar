
#include "Arduino.h"

//SD read and write
#include "SDcard.h"
#include <SD.h>

//#define MOSI 11
//#define MISO 12
//#define SCLK 13



void setupSD() {
  Serial.begin(9600);
  Serial.println("Initializing Card");
  //CS pin is an output
  pinMode(CS_PIN, OUTPUT);
  //Card will draw power from pin 8, so set it high
  pinMode(POW_PIN, OUTPUT);
  digitalWrite(POW_PIN, HIGH);
  if (!SD.begin(CS_PIN)) {
    Serial.println("Card Failure");
    return;
  }
  Serial.println("Card Ready");

  //Read the configuration information (speed.txt)
  /*
  File commandFile = SD.open("speed.txt");
  if (commandFile) {
    Serial.println("Reading Command File");
    while (commandFile.available()) {
      refresh_rate = commandFile.parseInt();
    }
    Serial.print("Refresh Rate = ");
    Serial.print(refresh_rate);
    Serial.println("ms");
    commandFile.close();  //Close the file when finished
  } else {
    Serial.println("Could not read command file.");
    return;
  }*/
}

void runSD(float data1, float data2) {
  long timeStamp = millis();

  //Open a file and write to it.
  File dataFile = SD.open("Glog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(timeStamp);
    dataFile.print(",");
    dataFile.println(data1);
    dataFile.print(",");
    dataFile.println(data2);
    dataFile.close();  //Data isn't actually written until we close the connection!
    Serial.println("SUCCESS: LOG FILE SAVED");

    //Print same thing to the screen for debugging
    Serial.println("SAVED DATA: ");
    Serial.print(timeStamp);
    Serial.print(": ");
    Serial.println(data1);
    Serial.print(", ");
    Serial.println(data2);
  } else {
    Serial.println("ERROR: COULD NOT OPEN LOG FILE");
  }
}