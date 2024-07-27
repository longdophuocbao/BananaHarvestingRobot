/** @file constants.h
    @brief A Documented file.
    
    Here declare all constant variables
    For schematic please refer to:
    For additional pin functionality refer to:

*/
#ifndef CONSTANTS_H
#define CONSTANTS_H

#define can_tx 9    // tx of serial can module connect to D2
#define can_rx 50    // rx of serial can module connect to D3

#define Limit_1 26   // pin limit switch Motor 1
#define Limit_2 27   // pin limit switch Motor 2
#define Start_Button 34  // butti 
#define Limit_open A5
#define Limit_dw A3
#define Knift 33

#define Motor_1 0x141   //ID motor 1
#define Motor_2 0x142   //ID motor 2

#define Kp_Pos_1 80   
#define Ki_Pos_1 80   

#define Kp_Spe_1 40   
#define Ki_Spe_1 30 

#define Kp_Tor_1 40   
#define Ki_Tor_1 40 

#define Kp_Pos_2 80   
#define Ki_Pos_2 80   

#define Kp_Spe_2 40   
#define Ki_Spe_2 30 

#define Kp_Tor_2 40   
#define Ki_Tor_2 40 

#define period_time 0.02//0.03

#define Limit_Rad_1_0 -1.8
#define Limit_Rad_1_1 1.3
#define Limit_Rad_2_0 -1.3 
#define Limit_Rad_2_1 2.79

#define MinX 150
#define MaxX 850
#define MinY -550
#define MaxY 550
#define MinZ 0
#define MaxZ 400

#define L1_length 380
#define L2_length 500
#define Distance 200


#define COUNTS_PER_REV 1296000 
#define GEAR_RATIO (48.0 / 18.0) 

#define speed_z 2000
#define accel_z 1000


#define PosInitisl_X 150
#define PosInitisl_Y 140
#define PosInitisl_Z 0



#endif