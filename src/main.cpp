#include <Arduino.h>
#include <math.h>

#include "Serial_python.h"
#include "Motor.h"
#include "InverseMatrix.h"
#include "Constants.h"
#include "Kinematic.h"
#include "BaseMode.h"
#include "esp_task_wdt.h"

#include "soc/rtc_wdt.h"
// Motor step
// dual core

double theta1 = 0;
double theta2 = 0;

uint16_t theta1_dot;
uint16_t theta2_dot;

const double L1 = L1_length;
const double L2 = L2_length;

double t = 0;
unsigned int currentStep = 0;
double t0 = 0;
double tf = 0;

double qd_X = 0;
double vd_X = 0;
float ad_X = 0;

double qd_Y = 0;
double vd_Y = 0;
float ad_Y = 0;

float qd_Z = 0;
float vd_Z = 0;
float ad_Z = 0;

double T[6] = {0, 2, 6, 9, 25, 30};

// double V[6][3] = {{10, -10, 0},
//                   {0, -100, 0},
//                   {0, -100, 0},
//                   {-50, 30, 0},
//                   {0, 0, 0},
//                   {0, 0, 0}};

double V[6][3] = {{10, -10, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {-50, 30, 0},
                  {0, 0, 0},
                  {0, 0, 0}};
// Acceleration
float AC[6][3] = {{0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0},
                  {0, 0, 0}};

float start[3] = {PosInitisl_X, PosInitisl_Y, PosInitisl_Z};  // 0
float temp_taget_Bana[3] = {Position_X, Position_Y + 100, 0}; // 1
float taget_Bana[3] = {Position_X, Position_Y - 150, 0};      // 2
// float taget_Bana[3] = {PosInitisl_X, PosInitisl_Y, 0};     // 2
float tranfer[3] = {PosInitisl_X, PosInitisl_Y, 0}; // 3

float temp_taget_Car[3] = {PosInitisl_X, PosInitisl_Y, PosInitisl_Z}; // 4

float taget_Car[3] = {650, 0, 0}; // 5

float q[6][3] = {{start[0], start[1], start[2]},
                 {temp_taget_Bana[0], temp_taget_Bana[1], temp_taget_Bana[2]},
                 {taget_Bana[0], taget_Bana[1], taget_Bana[2]},
                 {tranfer[0], tranfer[1], tranfer[2]},
                 {temp_taget_Car[0], temp_taget_Car[1], temp_taget_Car[2]},
                 {taget_Car[0], taget_Car[1], taget_Car[2]}};

void CalculatLine()
{
  double tar_x = 0;
  double tar_y = 0;
  double tar_t1 = 0;
  double tar_t2 = 0;
  Inverse_Kinemaic(tar_x, tar_y, &tar_t1, &tar_t1);
  double angle_line = tar_t2 - tar_t1 - PI / 2;
  double tar_x0 = Position_X - Distance * sin(angle_line);
  double tar_y0 = Position_Y + Distance * cos(angle_line);

  double tar_x1 = Position_X + Distance * sin(angle_line);
  double tar_y1 = Position_Y - Distance * cos(angle_line);
}

void updataPosition()
{
  if (Position_X < MinX)
    Position_X = MinX;
  if (Position_X > MaxX)
    Position_X = MaxX;
  if (Position_Y < MinY)
    Position_Y = MinY;
  if (Position_Y > MaxY)
    Position_Y = MaxY;
  if (Position_Z < MinZ)
    Position_Z = MinZ;
  if (Position_Z > MaxZ)
    Position_Z = MaxZ;

  double tar_the1 = 0;
  double tar_the2 = 0;

  Inverse_Kinemaic(Position_X, Position_Y, &tar_the1, &tar_the2);
  double angle_line = tar_the2 - tar_the1 - (PI / 2);
  Serial.print(" angle: ");
  Serial.println(angle_line);
  q[1][0] = Position_X - Distance * sin(angle_line);
  q[1][1] = Position_Y + Distance * cos(angle_line);
  q[2][0] = Position_X + (Distance - 80) * sin(angle_line);
  q[2][1] = Position_Y - (Distance - 80) * cos(angle_line);

  // q[1][0] = Position_X ;
  // q[1][1] = Position_Y ;

  Serial.println("Update position from camera!");
  Serial.print(q[1][0]);
  Serial.print("\t");
  Serial.println(q[1][1]);
  Serial.print(q[2][0]);
  Serial.print("\t");
  Serial.println(q[2][1]);

  // MoveZ(Position_Z);
  // delay(500);
}

double q0 = 0;
double q1 = 0;
double v0 = 0;
double v1 = 0;
double ac0 = 0;
double ac1 = 0;

double b[6][1] = {{0}, {0}, {0}, {1}, {1}, {1}};
double M_inv[6][6] = {{0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0}};
double a[6][1];

double period_t = (float)period_time;

uint32_t periodTime = (uint32_t)(period_t * 1000000.0);

uint16_t index_Value = 0;
enum RunMODE
{
  Automation,
  Manual
};
RunMODE runMode = Automation;

bool isRunning = false;
bool initi = true;
bool Calculate = false;
bool Calculate_done = false;
uint16_t count_index = 0;

// Hàm tính toán vận tốc góc từ ma trận Jacobi và ma trận vận tốc dài
void calculateAngularVelocity(float J[3][3], float x_dot[3], float theta_dot[3])
{
  // Tính toán vận tốc góc bằng cách nhân ma trận giả nghịch đảo với ma trận vận tốc dài
  for (int i = 0; i < 3; i++)
  {
    theta_dot[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      theta_dot[i] += J[i][j] * x_dot[j]; // Nhân ma trận và cộng dồn
    }
  }
}

void GoHome()
{
  Serial.println("Start_gotoHOME");
  // GotoHome_Z();
  //   GotoHome_basket();
  GotoHOME_1();
  Serial.print("Offset_1: ");
  Serial.println(Offset_1);

  GotoHOME_2();
  Serial.print("   Offset_2: ");
  Serial.println(Offset_2);

  double theta_1_temp = 0;
  double theta_2_temp = 0;
  Inverse_Kinemaic((double)PosInitisl_X, (double)PosInitisl_Y, &theta_1_temp, &theta_2_temp);
  Move_Rad(Motor_1, 1000, theta_1_temp);
  Move_Rad(Motor_2, 1000, theta_2_temp);
  Serial.println("Finish_setHome");
}

StructTrajectory Trajectory(1, 0, 0);
IBusBM IBus;

void setup()
{
  InitIBUS();
  setCpuFrequencyMhz(240);

  pinMode(Limit_1, INPUT);
  pinMode(Limit_2, INPUT);
  pinMode(Start_Button, INPUT);
  pinMode(Knift, OUTPUT);
  // pinMode(Limit_dw, INPUT);
  // pinMode(Limit_open, INPUT);

  Serial.begin(115200);

  InitMotor();
  // InitIbus();
  digitalWrite(Knift, LOW);
  // Stepper
  Serial.println("SETup_ Finish");

  // double the1=0;
  // double the2=0;
  // Position_X= 500;
  // Position_Y =100;
  // Inverse_Kinemaic(Position_X, Position_Y, &the1, &the2);
  // Serial.println(the1,4);
  // Serial.println(the2,4);

  // double angle_line = the2 - the1 - (PI/ 2);
  // Serial.print(" angle: ");
  // Serial.println(angle_line);
  // q[1][0] = Position_X - Distance * sin(angle_line);
  // q[1][1] = Position_Y + Distance * cos(angle_line);
  // q[2][0] = Position_X + Distance * sin(angle_line);
  // q[2][1] = Position_Y - Distance * cos(angle_line);

  // q[1][0] = Position_X ;
  // q[1][1] = Position_Y ;
  // q[2][0] = Position_X ;
  // q[2][1] = Position_Y - 100;

  // Serial.println("Update position from camera!");
  // Serial.print(q[1][0]);
  // Serial.print("\t");
  // Serial.println(q[1][1]);
  // Serial.print(q[2][0]);
  // Serial.print("\t");
  // Serial.println(q[2][1]);

  // esp_task_wdt_delete(NULL);
  // rtc_wdt_protect_off();
  // rtc_wdt_disable();
  rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  rtc_wdt_set_time(RTC_WDT_STAGE0, 500000);
}

void updateMatixM()
{
  t0 = T[currentStep];
  tf = T[currentStep + 1];

  double M[6][6] = {{1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5)},
                    {0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4)},
                    {0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3)},
                    {1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5)},
                    {0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4)},
                    {0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3)}};

  inverseMatrix(M, M_inv);
}
void SetInit()
{
  currentStep = 0;
  t = 0;
  // Calculate = true;
  index_Value = 0;
  count_index = 0;
  knifeActivated = false;
  Trajectory.setTotal_Index((uint16_t)(T[3] / period_t) + 1); // T[3]
  Serial.print("Completed set Total Index!");
  Serial.println(T[2] / period_t);
  // delay(1000);
  updateMatixM();
}

void loop()
{
  // for (int i = 0; i < 10; i++)
  // {
  //   Serial.print("\t");
  //   Serial.print(i);
  //   Serial.print(": ");
  //   Serial.print(IBus.readChannel(i));
  // }
  // Serial.println();
  // delay(5000);
  // if (initi == true)
  // {
  //   initi = false;
  //   GoHome();
  // }
  // delay(10000);

  if (IBus.readChannel(9) > 1500)
  {
    if (initi == true)
    {
      // Serial.println("begin");
      //  Set_PID_RAM(Motor_1, Kp_Pos_1, Ki_Pos_1, Kp_Spe_1, Ki_Spe_1, Kp_Tor_1, Ki_Tor_1);
      //  Set_PID_RAM(Motor_2, Kp_Pos_2, Ki_Pos_2, Kp_Spe_2, Ki_Spe_2, Kp_Tor_2, Ki_Tor_2);
      delay(3000);
      initi = false;
      GoHome();
      SetInit();
    }

    switch (runMode)
    {
    case Manual:
      delay(10);
      Move_basemode_auto_tranjectory();
      break;

    case Automation:
      if (ReceiveSerial())
      {
        if ((EnableRUN == 1) && (isRunning == false))
        {
          // digitalWrite(Led_Status_SERIAL, HIGH);
          EnableRUN = 0;
          updataPosition();
          Calculate = true;
          isRunning = true;
          //  Serial.println("Start_Cut");
          SetInit();
          delay(10);
        }
        while (Calculate == true)
        {
          Serial.println("Calculate");
          if (t >= T[currentStep + 1])
          {
            currentStep++;
            updateMatixM();
          }

          if (index_Value >= (Trajectory.getTotal_Index()))
          {
            Calculate_done = true;
            Calculate = false;
            Serial.println("FINISHED");
            Serial.println(Trajectory.getTotal_Index());
            break;
          }

          // Trajectory plan for X
          q0 = q[currentStep][0];
          q1 = q[currentStep + 1][0];
          v0 = V[currentStep][0];
          v1 = V[currentStep + 1][0];
          ac0 = AC[currentStep][0];
          ac1 = AC[currentStep + 1][0];

          b[0][0] = q0;
          b[1][0] = v0;
          b[2][0] = ac0;
          b[3][0] = q1;
          b[4][0] = v1;
          b[5][0] = ac1;
          for (int i = 0; i < 6; i++)
          {
            for (int j = 0; j < 1; j++)
            {
              a[i][j] = 0;
              for (int k = 0; k < 6; k++)
              {
                a[i][j] += M_inv[i][k] * b[k][j];
              }
            }
          }

          qd_X = (a[0][0]) + (a[1][0]) * t + (a[2][0]) * pow(t, 2) + (a[3][0]) * pow(t, 3) + (a[4][0]) * pow(t, 4) + (a[5][0]) * pow(t, 5);
          vd_X = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);
          // ad_X = 2 * a[2][0] * 1 + 6 * a[3][0] * t + 12 * a[4][0] * pow(t, 2) + 20 * a[5][0] * pow(t, 3);

          // Trajectory plan for Y
          q0 = q[currentStep][1];
          q1 = q[currentStep + 1][1];
          v0 = V[currentStep][1];
          v1 = V[currentStep + 1][1];
          ac0 = AC[currentStep][1];
          ac1 = AC[currentStep + 1][1];
          b[0][0] = q0;
          b[1][0] = v0;
          b[2][0] = ac0;
          b[3][0] = q1;
          b[4][0] = v1;
          b[5][0] = ac1;

          for (int i = 0; i < 6; i++)
          {
            for (int j = 0; j < 1; j++)
            {
              a[i][j] = 0;
              for (int k = 0; k < 6; k++)
              {
                a[i][j] += M_inv[i][k] * b[k][j];
              }
            }
          }

          qd_Y = a[0][0] * 1 + a[1][0] * t + a[2][0] * pow(t, 2) + a[3][0] * pow(t, 3) + a[4][0] * pow(t, 4) + a[5][0] * pow(t, 5);
          vd_Y = a[1][0] * 1 + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);
          // ad_Y = 2 * a[2][0] * 1 + 6 * a[3][0] * t + 12 * a[4][0] * pow(t, 2) + 20 * a[5][0] * pow(t, 3);

          // Inverse Kinematic
          Inverse_Kinemaic(qd_X, qd_Y, &theta1, &theta2);
          Trajectory.setTheta(0, index_Value, theta1);
          Trajectory.setTheta(1, index_Value, theta2);

          theta1_dot = abs(((cos(theta1 + theta2) / (L1 * sin(theta2))) * vd_X + (sin(theta1 + theta2) / (L1 * sin(theta2))) * vd_Y) * 5500.39);
          theta2_dot = abs(((-(L2 * cos(theta1 + theta2) + L1 * cos(theta1)) / (L1 * L2 * sin(theta2))) * vd_X + (-(L2 * sin(theta1 + theta2) + L1 * sin(theta1)) / (L1 * L2 * sin(theta2))) * vd_Y) * 5042.03);
          // Serial.print(index_Value);Serial.print(",");
          // Serial.print(t);Serial.print(",");
          // Serial.print(qd_X);Serial.print(",");
          // Serial.print(qd_Y);Serial.print(",");
          // Serial.print(theta1,6);Serial.print(",");
          // Serial.print(theta2,6);Serial.print(",");
          // Serial.print(vd_X,6);Serial.print(",");
          // Serial.print(vd_Y,6);Serial.print(",");

          // Serial.print(theta1_dot);Serial.print(",");
          // Serial.print(theta2_dot);Serial.print("\n");
          // delay(2000);

          Trajectory.setSpeed(0, index_Value, theta1_dot);
          Trajectory.setSpeed(1, index_Value, theta2_dot);

          t += period_t;
          index_Value++;
        }

        // Run according trajectory plan caculated
        if (Calculate_done == true)
        {
          // Serial.println("ccccc");
          // if (count_index < (Trajectory.getTotal_Index()))
          for (uint16_t count_index_2 = 0; count_index_2 < Trajectory.getTotal_Index(); count_index_2++)
          {
            Move_Rad(Motor_1, abs(Trajectory.getSpeed(0, count_index_2)), Trajectory.getTheta(0, count_index_2));
            delayMicroseconds(1500);
            Move_Rad(Motor_2, abs(Trajectory.getSpeed(1, count_index_2)), Trajectory.getTheta(1, count_index_2));
            delayMicroseconds(1500);
            delayMicroseconds((uint32_t)(period_t * 1000000.0) - 3000.0);
            // count_index++;

            // Serial.print(count_index); Serial.print(",");
            // Serial.print(Trajectory.getTheta(0,count_index),4); Serial.print(",");
            // Serial.print(Trajectory.getTheta(1,count_index),4); Serial.print(",");
            // Serial.print(Trajectory.getSpeed(0,count_index)); Serial.print(",");
            // Serial.print(Trajectory.getSpeed(1,count_index)); Serial.print("\n");
            // count_index++;

            // if (count_index == (uint16_t)(T[1] / period_t))
            // {
            //   delay(10000);
            // }

            if ((count_index_2 > (uint16_t)(T[1] / period_t)) && (count_index_2 < (uint16_t)(T[2] / period_t)) && (knifeActivated == false))
            {
              knifeActivated = true;
              digitalWrite(Knift, HIGH);
            }
            else if ((count_index_2 > (uint16_t)(T[2] / period_t)) && (knifeActivated == true))
            {
              knifeActivated = false;
              digitalWrite(Knift, LOW);
            }
          }
          isRunning = false;
          Serial.println("RUNNING FINISH!");
          delay(500);
        }
        // break;
      }
      else
      {
        Serial.println("Waiting data!");
        delay(10);
      }
    }
  }
  else
  {
    if (IBus.readChannel(7) > 1500)
    {
      Serial.println("Automation");
      runMode = Automation;
    }
    else
    {
      Serial.println("Manual");
      runMode = Manual;
      theta11 = -1.1377;
      theta22 = 2.7274;
      knifeActivated = false;
      // double cur_theta1 = 0;
      // double cur_theta2 = 0;

      // GetAngle(&cur_theta1, &cur_theta2);
    }
    delay(500);
    initi = true;
  }
}

// END FILE