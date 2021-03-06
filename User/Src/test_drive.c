
/*********************************************************** 
 *@file test_drive.c
 *@date 2017-10-06
 *@author Beck Pang
 *@architect Davide Asnaghi
 *
 
 The function provides a quick way to drive the motor from 
 the Remote Control input, using the following variables

 RX_X1	LEFT X axis 	Joystic
 RX_Y1	LEFT Y axis 	Joystic
 RX_X2 	RIGHT X axis	Joystic

 The following are standard library definitions for RC

 RC_CH_VALUE_MAX	Max value from Joystic
 RC_CH_VALUE_Min	Min value from Joystic
************************************************************/
#include "test_drive.h"
#include "test_DBUS.h"
#include "chassis_motors.h"

int32_t front_right;
int32_t back_right;
int32_t front_left;
int32_t back_left;
int16_t ltr_front_right;
int16_t ltr_back_right;
int16_t ltr_front_left;
int16_t ltr_back_left;

int32_t up_down;
int16_t up_down_position;
int16_t up_down_velocity;

void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1, int multiple)
{
    //Remote Control Commands, Mapped to match min and max RPM
    int16_t drive  = (int16_t)map(RX_X2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    int16_t strafe = (int16_t)map(RX_Y1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    int16_t rotate = (int16_t)map(RX_X1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    // For later coordinate with Gimbal
    int rotate_feedback = 0;

    front_right = (-1*drive + strafe + rotate) + rotate_feedback;   // CAN1 ID: 0x201
    back_right = (drive + strafe + rotate) + rotate_feedback;       // CAN1 ID: 0x202
    front_left = (drive - strafe + rotate) + rotate_feedback;       // CAN1 ID: 0x203
    back_left = (-1*drive - strafe + rotate) + rotate_feedback;     // CAN1 ID: 0x204
	
    //Update using CAN bus chassis function (provided by Alex Wong)
		ltr_front_right = PID_Control((float)CM1Encoder.velocity_from_ESC,0.1*multiple*front_right,&pid_front_right,0);
	  ltr_back_right = PID_Control((float)CM2Encoder.velocity_from_ESC,0.1*multiple*back_right,&pid_back_right,1);
    ltr_front_left = PID_Control((float)CM3Encoder.velocity_from_ESC,0.1*multiple*front_left,&pid_motors,2);
		ltr_back_left = PID_Control((float)CM4Encoder.velocity_from_ESC,0.1*multiple*back_left,&pid_motors,3);
		
    Chassis1_Set_Speed(ltr_front_right/4, ltr_back_right/4, ltr_front_left/4, ltr_back_left/4);
}

void drive_pneumatic(int RX_Y2)
{
    //Remote Control Commands, Mapped to match min and max RPM
		int16_t updown = (int16_t)map(RX_Y2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    // For later coordinate with Gimbal
	  up_down = updown; //CAN2 ID: 0X201
	//------------------------------------------------------------------------------------------------------------
		//up_down_position = PID_pos_Control(0.0001*(float)CM5Encoder.ecd_angle,0.01*up_down,&pid_position);
	//------------------------------------------------------------------------------------------------------------
		//up_down_velocity = PID_Control((float)CM5Encoder.velocity_from_ESC,up_down_position,&pid_velocity,5);
		//up_down_velocity = PID_Control((float)CM5Encoder.velocity_from_ESC,0.1*up_down,&pid_velocity,4);
    //Update using CAN bus chassis function (provided by Alex Wong)
		
		Chassis2_Set_Speed(0.1*0.5*up_down+800);
}
