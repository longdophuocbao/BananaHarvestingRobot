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

// Motor angles and speeds
double theta1 = 0;       // Angle of motor 1 (radians)
double theta2 = 0;       // Angle of motor 2 (radians)
uint16_t theta1_dot = 0; // Speed of motor 1
uint16_t theta2_dot = 0; // Speed of motor 2

// Robot arm constants
const double L1 = L1_length; // Length of first arm segment
const double L2 = L2_length; // Length of second arm segment

// Timing variables
double t = 0;                 // Current time
unsigned int currentStep = 0; // Current trajectory step
double t0 = 0;                // Start time of current step
double tf = 0;                // End time of current step

// Position, velocity, and acceleration
double qd_X = 0, vd_X = 0;          // X position and velocity
float ad_X = 0;                     // X acceleration
double qd_Y = 0, vd_Y = 0;          // Y position and velocity
float ad_Y = 0;                     // Y acceleration
float qd_Z = 0, vd_Z = 0, ad_Z = 0; // Z position, velocity, acceleration

// Trajectory timing points
double T[6] = {0, 2, 6, 9, 25, 30};

// Velocity profiles for each step
double V[6][3] = {
    {10, -10, 0}, // Step 1
    {0, 0, 0},    // Step 2
    {0, 0, 0},    // Step 3
    {-50, 30, 0}, // Step 4
    {0, 0, 0},    // Step 5
    {0, 0, 0}     // Step 6
};

// Acceleration profiles (all zero in this case)
float AC[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Target positions for trajectory
float start[3] = {PosInitisl_X, PosInitisl_Y, PosInitisl_Z};           // Initial position
float temp_target_banana[3] = {Position_X, Position_Y + 100, 0};       // Temporary banana target
float target_banana[3] = {Position_X, Position_Y - 150, 0};            // Final banana target
float transfer[3] = {PosInitisl_X, PosInitisl_Y, 0};                   // Transfer position
float temp_target_car[3] = {PosInitisl_X, PosInitisl_Y, PosInitisl_Z}; // Temporary car target
float target_car[3] = {650, 0, 0};                                     // Final car target

// Position array for all steps
float q[6][3] = {
    {start[0], start[1], start[2]},
    {temp_target_banana[0], temp_target_banana[1], temp_target_banana[2]},
    {target_banana[0], target_banana[1], target_banana[2]},
    {transfer[0], transfer[1], transfer[2]},
    {temp_target_car[0], temp_target_car[1], temp_target_car[2]},
    {target_car[0], target_car[1], target_car[2]}};

// Update position based on camera input
void updatePosition()
{
  // Constrain position within limits
  Position_X = constrain(Position_X, MinX, MaxX);
  Position_Y = constrain(Position_Y, MinY, MaxY);
  Position_Z = constrain(Position_Z, MinZ, MaxZ);

  // Calculate inverse kinematics
  double tar_theta1, tar_theta2;
  Inverse_Kinemaic(Position_X, Position_Y, &tar_theta1, &tar_theta2);

  // Calculate orientation angle
  double angle_line = tar_theta2 - tar_theta1 - (PI / 2);
  Serial.print("Angle: ");
  Serial.println(angle_line);

  // Update target positions
  q[1][0] = Position_X - Distance * sin(angle_line);
  q[1][1] = Position_Y + Distance * cos(angle_line);
  q[2][0] = Position_X + (Distance - 80) * sin(angle_line);
  q[2][1] = Position_Y - (Distance - 80) * cos(angle_line);

  // Log updated positions
  Serial.println("Updated position from camera:");
  Serial.print("q[1]: ");
  Serial.print(q[1][0]);
  Serial.print(", ");
  Serial.println(q[1][1]);
  Serial.print("q[2]: ");
  Serial.print(q[2][0]);
  Serial.print(", ");
  Serial.println(q[2][1]);
}

// Trajectory calculation variables
double q0 = 0, q1 = 0;                                  // Start and end positions
double v0 = 0, v1 = 0;                                  // Start and end velocities
double ac0 = 0, ac1 = 0;                                // Start and end accelerations
double b[6][1] = {{0}, {0}, {0}, {1}, {1}, {1}};        // Boundary conditions
double M_inv[6][6] = {0};                               // Inverse matrix for trajectory
double a[6][1];                                         // Polynomial coefficients
double period_t = (float)period_time;                   // Sampling period
uint32_t periodTime = (uint32_t)(period_t * 1000000.0); // Period in microseconds
uint16_t index_Value = 0;                               // Current trajectory index

// Operating modes
enum RunMode
{
  AUTOMATION,
  MANUAL
};
RunMode runMode = AUTOMATION;

// State flags
bool isRunning = false;
bool initialized = true;
bool calculate = false;
bool calculateDone = false;
uint16_t countIndex = 0;

// Calculate angular velocities using Jacobian
void calculateAngularVelocity(float J[3][3], float x_dot[3], float theta_dot[3])
{
  for (int i = 0; i < 3; i++)
  {
    theta_dot[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      theta_dot[i] += J[i][j] * x_dot[j];
    }
  }
}

// Move to home position
void goHome()
{
  Serial.println("Moving to home position...");
  GotoHOME_1();
  Serial.print("Offset_1: ");
  Serial.println(Offset_1);
  GotoHOME_2();
  Serial.print("Offset_2: ");
  Serial.println(Offset_2);

  // Set initial position
  double theta1_temp, theta2_temp;
  Inverse_Kinemaic((double)PosInitisl_X, (double)PosInitisl_Y, &theta1_temp, &theta2_temp);
  Move_Rad(Motor_1, 1000, theta1_temp);
  Move_Rad(Motor_2, 1000, theta2_temp);
  Serial.println("Home position reached");
}

// Trajectory and IBus objects
StructTrajectory trajectory(1, 0, 0);
IBusBM IBus;

// Setup function
void setup()
{
  InitIBUS();
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz

  // Initialize pins
  pinMode(Limit_1, INPUT);
  pinMode(Limit_2, INPUT);
  pinMode(Start_Button, INPUT);
  pinMode(Knift, OUTPUT);
  digitalWrite(Knift, LOW);

  // Initialize serial and motors
  Serial.begin(115200);
  InitMotor();

  // Configure watchdog timer
  rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
  rtc_wdt_set_time(RTC_WDT_STAGE0, 500000);

  Serial.println("Setup completed");
}

// Update trajectory matrix
void updateMatrixM()
{
  t0 = T[currentStep];
  tf = T[currentStep + 1];

  double M[6][6] = {
      {1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5)},
      {0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4)},
      {0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3)},
      {1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5)},
      {0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4)},
      {0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3)}};

  inverseMatrix(M, M_inv);
}

// Initialize trajectory
void setInit()
{
  currentStep = 0;
  t = 0;
  index_Value = 0;
  countIndex = 0;
  knifeActivated = false;
  trajectory.setTotal_Index((uint16_t)(T[3] / period_t) + 1);
  Serial.print("Set total index: ");
  Serial.println(T[2] / period_t);
  updateMatrixM();
}

// Main loop
void loop()
{
  if (IBus.readChannel(9) > 1500)
  {
    // Initialize on first run
    if (initialized)
    {
      delay(3000);
      initialized = false;
      goHome();
      setInit();
    }

    switch (runMode)
    {
    case MANUAL:
      delay(10);
      Move_basemode_auto_tranjectory();
      break;

    case AUTOMATION:
      if (ReceiveSerial())
      {
        if (EnableRUN == 1 && !isRunning)
        {
          EnableRUN = 0;
          updatePosition();
          calculate = true;
          isRunning = true;
          setInit();
          delay(10);
        }

        // Calculate trajectory
        while (calculate)
        {
          Serial.println("Calculating trajectory...");
          if (t >= T[currentStep + 1])
          {
            currentStep++;
            updateMatrixM();
          }

          if (index_Value >= trajectory.getTotal_Index())
          {
            calculateDone = true;
            calculate = false;
            Serial.print("Trajectory calculation finished. Total indices: ");
            Serial.println(trajectory.getTotal_Index());
            break;
          }

          // X trajectory
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
            a[i][0] = 0;
            for (int k = 0; k < 6; k++)
            {
              a[i][0] += M_inv[i][k] * b[k][0];
            }
          }

          qd_X = a[0][0] + a[1][0] * t + a[2][0] * pow(t, 2) + a[3][0] * pow(t, 3) + a[4][0] * pow(t, 4) + a[5][0] * pow(t, 5);
          vd_X = a[1][0] + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);

          // Y trajectory
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
            a[i][0] = 0;
            for (int k = 0; k < 6; k++)
            {
              a[i][0] += M_inv[i][k] * b[k][0];
            }
          }

          qd_Y = a[0][0] + a[1][0] * t + a[2][0] * pow(t, 2) + a[3][0] * pow(t, 3) + a[4][0] * pow(t, 4) + a[5][0] * pow(t, 5);
          vd_Y = a[1][0] + 2 * a[2][0] * t + 3 * a[3][0] * pow(t, 2) + 4 * a[4][0] * pow(t, 3) + 5 * a[5][0] * pow(t, 4);

          // Inverse kinematics and velocity calculations
          Inverse_Kinemaic(qd_X, qd_Y, &theta1, &theta2);
          trajectory.setTheta(0, index_Value, theta1);
          trajectory.setTheta(1, index_Value, theta2);

          theta1_dot = abs(((cos(theta1 + theta2) / (L1 * sin(theta2))) * vd_X + (sin(theta1 + theta2) / (L1 * sin(theta2))) * vd_Y) * 5500.39);
          theta2_dot = abs(((-(L2 * cos(theta1 + theta2) + L1 * cos(theta1)) / (L1 * L2 * sin(theta2))) * vd_X + (-(L2 * sin(theta1 + theta2) + L1 * sin(theta1)) / (L1 * L2 * sin(theta2))) * vd_Y) * 5042.03);

          trajectory.setSpeed(0, index_Value, theta1_dot);
          trajectory.setSpeed(1, index_Value, theta2_dot);

          t += period_t;
          index_Value++;
        }

        // Execute trajectory
        if (calculateDone)
        {
          for (uint16_t count_index_2 = 0; count_index_2 < trajectory.getTotal_Index(); count_index_2++)
          {
            Move_Rad(Motor_1, abs(trajectory.getSpeed(0, count_index_2)), trajectory.getTheta(0, count_index_2));
            delayMicroseconds(1500);
            Move_Rad(Motor_2, abs(trajectory.getSpeed(1, count_index_2)), trajectory.getTheta(1, count_index_2));
            delayMicroseconds(1500);
            delayMicroseconds((uint32_t)(period_t * 1000000.0) - 3000.0);

            // Control knife
            if (count_index_2 > (uint16_t)(T[1] / period_t) && count_index_2 < (uint16_t)(T[2] / period_t) && !knifeActivated)
            {
              knifeActivated = true;
              digitalWrite(Knift, HIGH);
            }
            else if (count_index_2 > (uint16_t)(T[2] / period_t) && knifeActivated)
            {
              knifeActivated = false;
              digitalWrite(Knift, LOW);
            }
          }
          isRunning = false;
          Serial.println("Trajectory execution completed!");
          delay(500);
        }
      }
      else
      {
        Serial.println("Waiting for serial data...");
        delay(10);
      }
      break;
    }
  }
  else
  {
    // Switch between modes
    runMode = IBus.readChannel(7) > 1500 ? AUTOMATION : MANUAL;
    Serial.println(runMode == AUTOMATION ? "Automation mode" : "Manual mode");
    if (runMode == MANUAL)
    {
      theta11 = -1.1377;
      theta22 = 2.7274;
      knifeActivated = false;
    }
    delay(500);
    initialized = true;
  }
}