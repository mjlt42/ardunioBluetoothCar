//PPPP

#include "Arduino.h"
#include "CrashDetect.h"

//Ultra sonic sensor
float pingTime; //Time for sound to travel to and back from object
float speedOfSound= 331 + 0.61*25;
unsigned long lastTriggerTime = 0;     //stores the last time the trigger pulse was sent
bool triggerSent = false;              //boolen to check if trigger pulse was sent or not
unsigned long echoStartTime = 0;       //time since echo pulse was detected
bool echoReceived = false;             //boolen to check if echo pulse was detected or not

/*
Author: Matthew Lopez Tarsky (12/11/24) (Assisted with ChatGPT)
Function uses polling to check distance using ultra sonic sensor

*/
float checkDistance(int trigPin, int echoPin, int timeDelay) {
  float distance; //Distance measured
  unsigned long currentTime = micros();
  //Send trigger pulse if not sent already
  if (!triggerSent) {
    digitalWrite(trigPin, LOW);                  
    if (currentTime - lastTriggerTime > 2000) {  //waits 2000 micro seconds for trigger pin to settle 
      digitalWrite(trigPin, HIGH);               
      lastTriggerTime = currentTime;
      triggerSent = true;
    }
  } else {
    if (currentTime - lastTriggerTime > 15) { //waits 15 micro seconds for trigger pin to settle
      digitalWrite(trigPin, LOW);
    }
  }

  if (triggerSent && !echoReceived) {
    if (digitalRead(echoPin) == HIGH) {
      echoStartTime = micros();
    } else if (digitalRead(echoPin) == LOW && echoStartTime > 0) {
      pingTime = micros() - echoStartTime; //calculate the ping when echo ends.
      echoReceived = true;
    }
  }


  if (echoReceived) {
    pingTime = pingTime / 1000000.0;                   //convert into seconds
    distance = (speedOfSound * pingTime) / 2.0;  //distance in meters
    distance *= 100; //convert to cms                         
    triggerSent = false;
    echoReceived = false;
    echoStartTime = 0;
    lastTriggerTime = currentTime + timeDelay;  //delay/wait added
  }
  return distance;
}