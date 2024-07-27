#include "BaseMode.h"

// StructTrajectory Trajectory(1, 0, 0);
extern IBusBM IBus; // IBus object
void InitIBUS()
{
    IBus.begin(Serial2, 2);
}

double theta11 = -1.1377;
double theta22 = 2.7274;
StructTrajectory Trajectory_Base(1, 0, 0);
bool knifeActivated = false;
void Move_basemode_auto_tranjectory()
{  
    if ((IBus.readChannel(8) > 1500) && (knifeActivated == false))
    {
        knifeActivated = true;
        digitalWrite(Knift, HIGH);
    }
    else if ((IBus.readChannel(8) < 1500) && (knifeActivated == true))
    {
        knifeActivated = false;
        digitalWrite(Knift, LOW);
    }
    // Serial.println("Move_basemode_auto_tranjectory");
    int val_X = map(IBus.readChannel(1), 1000, 2000, -20, 20);
    int val_Y = map(IBus.readChannel(3), 1000, 2000, 20, -20);
    Serial.print("val_X:");
    Serial.println(val_X);
    Serial.print("val_Y:");
    Serial.println(val_Y);
    // Serial.println("val_X");
    if ((-2 > val_X) || (2 < val_X) || (-2 > val_Y) || (2 < val_Y))
    {
        // double theta11;
        // double theta22;
        // GetAngle(&theta1, &theta2);
        double X_cur;
        double Y_cur;

        Forward_Kinemaic(theta11, theta22, &X_cur, &Y_cur);

        double X_target = X_cur + val_X;
        double Y_target = Y_cur + val_Y;
        double theta1_target;
        double theta2_target;

        Inverse_Kinemaic(X_target, Y_target, &theta1_target, &theta2_target);

        uint16_t distance_move = sqrt(pow(abs(X_target - X_cur), 2) + pow(abs(Y_target - Y_cur), 2));
        Trajectory_Base.setTotal_Index(distance_move*2);
        double time_move = ((double)distance_move) * 1 / 100;

        for (int index_i = 0; index_i < Trajectory_Base.getTotal_Index() + 1; index_i++)
        {
            Trajectory_Base.setTheta(0, index_i, theta11 + ((theta1_target - theta11) / Trajectory_Base.getTotal_Index()) * index_i);
            Trajectory_Base.setTheta(1, index_i, theta22 + ((theta2_target - theta22) / Trajectory_Base.getTotal_Index()) * index_i);

            Trajectory_Base.setSpeed(0, index_i, abs(((theta1_target - theta11) / (time_move)) * 5500.39));
            Trajectory_Base.setSpeed(1, index_i, abs(((theta2_target - theta22) / (time_move)) * 5042.03));
        }
        for (int index_i = 0; index_i < Trajectory_Base.getTotal_Index() + 1; index_i++)
        {
            Move_Rad(Motor_1, (uint16_t)abs(Trajectory_Base.getSpeed(0, index_i)), Trajectory_Base.getTheta(0, index_i));
            Move_Rad(Motor_2, (uint16_t)abs(Trajectory_Base.getSpeed(1, index_i)), Trajectory_Base.getTheta(1, index_i));
            delayMicroseconds((uint32_t)(time_move / Trajectory_Base.getTotal_Index() * 1000000.00));
        }
        // for (int index_i = 0; index_i < Trajectory_Base.getTotal_Index() + 1; index_i++)
        // {
        //     Serial.print(index_i);
        //     Serial.print(",");
        //     Serial.print(Trajectory_Base.getTheta(0, index_i), 6);
        //     Serial.print(",");
        //     Serial.print(Trajectory_Base.getTheta(1, index_i), 6);
        //     Serial.print(",");
        //     Serial.print(Trajectory_Base.getSpeed(0, index_i));
        //     Serial.print(",");
        //     Serial.println(Trajectory_Base.getSpeed(1, index_i));
        // }
        theta11 = Trajectory_Base.getTheta(0, Trajectory_Base.getTotal_Index());
        theta22 = Trajectory_Base.getTheta(1, Trajectory_Base.getTotal_Index());
        // delay(10000);
        // Trajectory_Base.deallocateMemory();
    }
}


