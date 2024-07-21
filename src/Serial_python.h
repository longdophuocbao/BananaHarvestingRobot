#include <Arduino.h>

#ifndef SERIAL_H
#define SERIAL_H

extern float Position_X;
extern float Position_Y;
extern float Position_Z;
bool ReceiveSerial();
extern int EnableRUN;

#endif