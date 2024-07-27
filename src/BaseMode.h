#include <Arduino.h>
#include <Motor.h>
#include "Kinematic.h"
//#include "Kinematic.h"
#include <IBusBM.h>



#ifndef BASEMODE_H
#define BASEMODE_H

extern double theta11;
extern double theta22;

void InitIBUS();
void Move_basemode_auto_tranjectory();
#endif