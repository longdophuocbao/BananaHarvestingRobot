#include "Motor.h"
// #include <Serial_CAN_Module.h>
// #include <SoftwareSerial.h>
#include <AccelStepper.h>

const int SPI_CS_PIN = 5;
MCP_CAN mcp2515(SPI_CS_PIN);

int32_t Offset_1 = 0;
int32_t Offset_2 = 0;

int8_t Reverse_1 = 1;
int8_t Reverse_2 = -1;

int Pulse_Z = 22;
int Dir_Z = 21;
int ena_Z = 13;

int Step = 4;
int Dir = 32;
int ena = 13;

AccelStepper Stepper_Z(1, Pulse_Z, Dir_Z, ena_Z);
AccelStepper Stepper_bastket(1, Step, Dir, ena);

#define CAN_MAX_DLEN 8
unsigned char rxBuf[8];
typedef struct _vCAN_t
{
    unsigned long can_id;
    unsigned char can_dlc = 8;
    unsigned char data[CAN_MAX_DLEN] __attribute__((aligned(8)));
} vCAN_t;

vCAN_t canSent;
vCAN_t canRecei;

// float Limit_Rad_1_0 = -1.4;
// float Limit_Rad_1_1 = 1.4;
// float Limit_Rad_2_0 = -1.5;
// float Limit_Rad_2_1 = 2.86;

/// @brief
void InitMotor()
{
    mcp2515.begin(MCP_NORMAL, CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setMode(MCP_NORMAL);
    canRecei.can_id = 0x141;
    canRecei.can_dlc = 8;
    canRecei.data[0] = 0x92;
    canRecei.data[1] = 0x0;
    canRecei.data[2] = 0x0;
    canRecei.data[3] = 0x0;
    canRecei.data[4] = 0x0;
    canRecei.data[5] = 0x0;
    canRecei.data[6] = 0x0;
    canRecei.data[7] = 0x0;

    Stepper_Z.setMaxSpeed(2000);
    Stepper_Z.setAcceleration(1000);

    Stepper_bastket.setMaxSpeed(2000);
    Stepper_bastket.setAcceleration(1000);
}

/// @brief
/// @param _Motor
/// @return
unsigned long GetEconder(uint16_t _Motor)
{
    canRecei.can_id = _Motor;
    canRecei.data[0] = 0x92;
    unsigned long rawValue = 0;
    mcp2515.sendMsgBuf(canRecei.can_id, 0, 8, canRecei.data);
    while (1)
    {
        mcp2515.readMsgBuf(&canSent.can_id, &canSent.can_dlc, canRecei.data);

        if (canRecei.data[0] == 0x92)
        {
            uint32_t a = canRecei.data[4];
            uint32_t b = canRecei.data[3];
            uint32_t c = canRecei.data[2];
            uint32_t d = canRecei.data[1];
            rawValue = (a << 24) | (b << 16) | (c << 8) | d;
            break;
        }
    }

    // for (size_t i = 0; i < 5; i++)
    // {
    //     if (mcp2515.readMessage(&canRecei) == MCP2515::ERROR_OK)
    //     {
    //         if (canRecei.data[0] == 0x92)
    //         {
    //             uint32_t a = canRecei.data[4];
    //             uint32_t b = canRecei.data[3];
    //             uint32_t c = canRecei.data[2];
    //             uint32_t d = canRecei.data[1];
    //             rawValue = (a << 24) | (b << 16) | (c << 8) | d;
    //             break;
    //         }
    //     }
    // }
    return rawValue;
}

void MoveZ(int pulse)
{
    Stepper_Z.moveTo(-(pulse * 40));
    Stepper_Z.runToPosition();
}

void GetAngle(double *_theta1, double *_theta2)
{
    int32_t pulse_1 = (int32_t)(GetEconder(Motor_1) - 0x100000000);
    int32_t pulse_2 = (int32_t)(GetEconder(Motor_2) - 0x100000000);
    // Serial.print("pulse_1:");
    // Serial.println(pulse_1);
    // Serial.print("pulse_2:");
    // Serial.println(pulse_2);
    // uint32_t pulse_1 = GetEconder(Motor_1);
    // uint32_t pulse_2 = GetEconder(Motor_2);
    *_theta1 = TransferPulse2Angle(Motor_1, pulse_1);
    *_theta2 = TransferPulse2Angle(Motor_2, pulse_2);
}

/// @brief
/// @param Motor
/// @param angle_Rad
/// @return
int32_t TransferAngle2Pulse(uint16_t _Motor, float _angle_Rad = 0)
{
    int32_t pulse;
    if (_Motor == Motor_1)
    {
        pulse = ((int32_t)(_angle_Rad * 550039.4833) + Offset_1 + (int32_t)967088) * Reverse_1;
        return pulse;
    }
    else
    {
        pulse = ((int32_t)(_angle_Rad * 504202.8597) + abs(Offset_2) - (int32_t)1403758) * Reverse_2;
        return pulse;
    }
}

double TransferPulse2Angle(uint16_t Motor, int32_t pulse)
{
    if (Motor == Motor_1)
    {
        return (double)((pulse / Reverse_1) - Offset_1 - (int32_t)967088) / 550039.4833;
    }
    else
    {
        return (double)((pulse / Reverse_2) - abs(Offset_2) + (int32_t)1403758) / 504202.8597;
    }
}

/// @brief
/// @param Motor
/// @param speed
/// @param pulse
void Move(uint16_t Motor, uint16_t speed, int32_t pulse)
{
    // uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);
    canSent.can_id = Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0xA4;
    canSent.data[1] = 0x0;
    canSent.data[2] = (uint8_t)(speed);
    canSent.data[3] = (uint8_t)(speed >> 8);
    canSent.data[4] = (uint8_t)pulse;
    canSent.data[5] = (uint8_t)(pulse >> 8);
    canSent.data[6] = (uint8_t)(pulse >> 16);
    canSent.data[7] = (uint8_t)(pulse >> 24);
    mcp2515.sendMsgBuf(canSent.can_id, 0, 8, canSent.data);
}

/// @brief
/// @param _Motor
/// @param _Speed
/// @param _Angle
void Move_Rad(uint16_t _Motor, uint16_t _Speed, double _Angle)
{
    if (_Motor == Motor_1)
    {
        if (_Angle < (double)Limit_Rad_1_0)
        {
            Serial.println("Over under angle Motor1!");
            _Angle = Limit_Rad_1_0;
        }
        else if (_Angle > (double)Limit_Rad_1_1)
        {
            Serial.println("Over higher angle Motor1!");
            _Angle = Limit_Rad_1_1;
        }
    }
    else
    {
        if (_Angle < (double)Limit_Rad_2_0)
        {
            Serial.println("Over under angle Motor2!");
            _Angle = Limit_Rad_2_0;
        }
        else if (_Angle > (double)Limit_Rad_2_1)
        {
            Serial.println("Over higher angle Motor2!");
            _Angle = Limit_Rad_2_1;
        }
    }
    int32_t pulse = TransferAngle2Pulse(_Motor, _Angle);

    // Serial.print(_Angle);
    // Serial.print("\t");
    // Serial.println(pulse);
    canSent.can_id = _Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0xA4;
    canSent.data[1] = 0x0;
    canSent.data[2] = _Speed;
    canSent.data[3] = _Speed >> 8;
    canSent.data[4] = pulse;
    canSent.data[5] = pulse >> 8;
    canSent.data[6] = pulse >> 16;
    canSent.data[7] = pulse >> 24;
    mcp2515.sendMsgBuf(canSent.can_id, 0, canSent.can_dlc, canSent.data);
}
void Set_PID_RAM(uint16_t _Motor, uint8_t _Kp_Pos, uint8_t _Ki_Pos, uint8_t _Kp_Spe, uint8_t _Ki_Spe, uint8_t _Kp_Tor, uint8_t _Ki_Tor)
{
    Serial.println("Set PID");
    canSent.can_id = _Motor;
    canSent.can_dlc = 8;
    canSent.data[0] = 0x31;
    canSent.data[1] = 0x0;
    canSent.data[2] = _Kp_Pos;
    canSent.data[3] = _Ki_Pos;
    canSent.data[4] = _Kp_Spe;
    canSent.data[5] = _Ki_Spe;
    canSent.data[6] = _Kp_Tor;
    canSent.data[7] = _Ki_Tor;
    mcp2515.sendMsgBuf(canSent.can_id, 0, canSent.can_dlc, canSent.data);

    canRecei.can_id = _Motor;
    canRecei.data[0] = 0x31;
    // mcp2515.sendMsgBuf(canSent.can_id,0,canSent.can_dlc,canSent.data);
    while (1)
    {
        mcp2515.readMsgBuf(&canSent.can_id, &canSent.can_dlc, canRecei.data);
        if (canRecei.data[0] == 0x31)
        {
            if ((canRecei.data[2] == _Kp_Pos) && (canRecei.data[3] == _Ki_Pos) && (canRecei.data[4] == _Kp_Spe) && (canRecei.data[5] == _Ki_Spe) && (canRecei.data[6] == _Kp_Tor) && (canRecei.data[7] == _Ki_Tor))
            {
                // Serial.print(_Motor,HEX);
                Serial.println(" Successfully - Set PID");
                break;
            }
        }

        delay(10);
    }
}

void GotoHome_basket()
{
    Serial.println("Go to home basket!");
    Stepper_bastket.setMaxSpeed(1000);
    // Stepper_bastket.setAcceleration(2000);
    // Stepper_bastket.setSpeed(500);
    Stepper_bastket.setCurrentPosition(0); // Set vị trí hiện tại của động cơ là 0
    Stepper_bastket.moveTo(-200);
    Stepper_bastket.runToPosition();

    int temp = 0;
    while (analogRead(Limit_open) < 500)
    {
        Serial.println("cc");
        Stepper_bastket.moveTo(temp);
        Stepper_bastket.runToPosition();
        temp = temp + 100;
    }
    Stepper_bastket.setCurrentPosition(0);
    Stepper_bastket.setMaxSpeed(speed_z);
    Stepper_bastket.setAcceleration(accel_z);
    Stepper_bastket.moveTo(-4000);
    Stepper_bastket.runToPosition();
}

void GotoHome_Z()
{
    Serial.println("Go to home Z_axis!");
    Stepper_Z.setMaxSpeed(4000);
    Stepper_Z.setAcceleration(2000);
    Stepper_Z.setSpeed(1000);
    Stepper_Z.setCurrentPosition(0); // Set vị trí hiện tại của động cơ là 0
    Stepper_Z.moveTo(-1000);
    Stepper_Z.runToPosition();
    delay(500);
    int temp = 0;
    while (analogRead(Limit_dw) < 500)
    {
        Stepper_Z.moveTo(temp);
        Stepper_Z.runToPosition();
        temp = temp + 50;
    }
    Stepper_Z.setCurrentPosition(0);
    Stepper_Z.setMaxSpeed(speed_z);
    Stepper_Z.setAcceleration(accel_z);
    // Stepper_Z.moveTo(-4000);
    // Stepper_Z.runToPosition();
}
/// @brief
void GotoHOME_1()
{
    // Serial.println("RUNNING_gotoHOME_Motor_1");
    //  Di chuyển về home với tốc độ cao ban đầu
    const uint16_t initialSpeed = 1200;
    const uint16_t initialStep = 1000;
    const uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);
    int32_t temp = 0;
    while (1)
    {
        if (digitalRead(Limit_1) == 0)
        {
            break;
        }
        Move(Motor_1, initialSpeed, temp);
        temp -= initialStep;
        delayMicroseconds(initialDelay);
    }
    // Serial.println("1_gotoHOME_Motor_1");
    delay(100);

    temp = temp + 50000;
    Move(Motor_1, initialSpeed, temp);
    delay(550);

    // Tìm vị trí home chính xác với tốc độ và bước rất nhỏ
    const uint16_t finalSpeed = initialSpeed / 10;
    const uint16_t finalStep = initialStep / 20;
    const uint16_t finalDelay = (uint16_t)((((float)finalStep) / ((float)finalSpeed)) * 10000.0);
    while (1)
    {
        if (digitalRead(Limit_1) == 0)
        {
            break;
        }
        Move(Motor_1, finalSpeed, temp);
        temp = temp - finalStep;
        delayMicroseconds(finalDelay);
    }
    // Serial.println("Finish_gotoHOME_Motor_1");
    Offset_1 = (int32_t)(GetEconder(Motor_1) - 0x100000000);
    delay(100);
}

void GotoHOME_2()
{
    // Serial.println("RUNNING_gotoHOME_Motor_2");
    //  Di chuyển về home với tốc độ cao ban đầu
    const uint16_t initialSpeed = 1200;
    const uint16_t initialStep = 1000;
    const uint16_t initialDelay = (uint16_t)((((float)initialStep) / ((float)initialSpeed)) * 10000.0);

    int32_t temp = 0;

    while (1)
    {
        if (digitalRead(Limit_2) == 0)
        {
            break;
        }
        Move(Motor_2, initialSpeed, temp);
        temp = temp - initialStep;
        delayMicroseconds(initialDelay);
    }
    // Serial.println("1_gotoHOME_Motor_2");
    delay(100);

    temp = temp + 50000;
    Move(Motor_2, initialSpeed, temp);
    delay(550);

    // Tìm vị trí home chính xác với tốc độ và bước rất nhỏ
    const uint16_t finalSpeed = initialSpeed / 10;
    const uint16_t finalStep = initialStep / 20;
    const uint16_t finalDelay = (uint16_t)((((float)finalStep) / ((float)finalSpeed)) * 10000.0);

    while (1)
    {
        if (digitalRead(Limit_2) == 0)
        {
            break;
        }
        Move(Motor_2, finalSpeed, temp);
        temp = temp - finalStep;
        delayMicroseconds(finalDelay);
    }
    // Serial.println("Finish_gotoHOME_Motor_2");
    Offset_2 = (int32_t)(GetEconder(Motor_2) - 0x100000000);
    delay(100);
}