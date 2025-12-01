#include "Arduino.h"

void setupMPU();
void recordAccelRegisters();
void processAccelData();
void maxAccel(float Accel, float* maxG);
