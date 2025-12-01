#include "Arduino.h"

const int pinTrig = 3;
const int pinEcho = 6;
const int delayTime = 10000; //Delay time in micro seconds

float checkDistance(int trigPin, int echoPin, int timeDelay);