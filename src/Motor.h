#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"
#include "constants.h"

#ifndef MOTOR_H
#define MOTOR_H

extern int32_t Offset_1;
extern int32_t Offset_2;

extern int8_t Reverse_1;
extern int8_t Reverse_2;

extern bool knifeActivated;

void InitMotor();
unsigned long GetEconder(uint16_t _Motor);
void GetAngle(double *_theta1, double *_theta2);
int32_t TransferAngle2Pulse(uint16_t Motor, float angle_Rad);
double TransferPulse2Angle(uint16_t Motor, int32_t pulse);
void Move(uint16_t Motor, uint16_t speed, int32_t pulse);
void Move_Rad(uint16_t _Motor, uint16_t _Speed, double _Angle);
void Move_Speed(uint16_t _Motor, int32_t _Speed);
void MoveZ(int pulse);

void Set_PID_RAM(uint16_t _Motor, uint8_t _Kp_Pos, uint8_t _Ki_Pos, uint8_t _Kp_Spe, uint8_t _Ki_Spe, uint8_t _Kp_Tor, uint8_t _Ki_Tor);

void GotoHome_basket();
void GotoHome_Z();
void GotoHOME_1();
void GotoHOME_2();
#endif