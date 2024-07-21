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
    if ((IBus.readChannel(8) > 1000) && (knifeActivated == false))
    {
        knifeActivated = true;
        digitalWrite(Knift, HIGH);
    }
    else if ((IBus.readChannel(8) < 1000) && (knifeActivated == true))
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
    if ((-5 > val_X) || (5 < val_X) || (-5 > val_Y) || (5 < val_Y))
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
        // Serial.print("distance_move");
        // Serial.println(distance_move / 4);
        Trajectory_Base.setTotal_Index(distance_move);
        double time_move = ((double)distance_move) * 1 / 100;
        // Serial.print("time_move");
        // Serial.println(time_move, 3);

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

int compareNumbers(float previous, float current)
{
    if (current > previous)
    {
        return -1; // Số sau lớn hơn số trước
    }
    else if (current < previous)
    {
        return 1; // Số sau nhỏ hơn số trước
    }
    else
    {
        return 0; // Hai số bằng nhau
    }
}

// void Move_basemode_auto_tranjectory()
// {
//     // Serial.println("Move_basemode ");
//     int32_t Encoder_1 = GetEconder(Motor_1);
//     int32_t Encoder_2 = GetEconder(Motor_2);
//     float theta1_old = TransferPulse2Angle(Motor_1, Encoder_1 );
//     float theta2_old = TransferPulse2Angle(Motor_2, Encoder_2 );

//     float X_old =0;
//     float Y_old =0;

//     int distance_X_move = 0;
//     int distance_Y_move = 0;
//     //ReceiveDATA_Ibus(&distance_X_move,&distance_Y_move);

//     float X_Target = X_old + distance_X_move;
//     float Y_Target = Y_old + distance_Y_move;

//     Forward_Kinemaic(theta1_old,theta2_old,&X_old,&Y_old);

//     int index_trajectory = abs(round(distance(X_old,Y_old, X_Target, Y_Target))) * 2 ;

//     float X_temp=0;
//     float Y_temp=0;

//     float theta1=0;
//     float theta2=0;

//     for (int Value_Index = 0; Value_Index < index_trajectory + 1; Value_Index++)
//     {
//         X_temp= X_old + Value_Index*0.5*compareNumbers(X_Target,X_old) ;
//         Y_temp= Y_old + Value_Index*0.5*compareNumbers(X_Target,X_old) ;
//         Inverse_Kinemaic(X_temp,Y_temp,&theta1,&theta2);

//         Trajectory.theta[0][Value_Index] = theta1;
//         Trajectory.theta[1][Value_Index] = theta2;

//     }
//     for (int Index = 0; Index < index_trajectory + 1; Index++)
//     {
//         Move_Rad(Motor_1,1000,Trajectory.theta[0][Index]);
//         Move_Rad(Motor_2,1000,Trajectory.theta[1][Index]);
//     }
//     Trajectory.deallocateMemory();

// }
