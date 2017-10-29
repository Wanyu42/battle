/**
  *@file test_can.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */

#include "can.h"
#include "arm_math.h"
#include "chassis_motors.h"
#include <stdlib.h>
#include "test_DBUS.h"
  
uint8_t can1_rx_data[8];
uint8_t can2_rx_data[8];

// Chassis Motor Encoder
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // CAN1 Address 201
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // CAN1 Address 202
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // CAN1 Address 203
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // CAN1 Address 204
volatile Encoder CM5Encoder = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // CAN2 Address 201

uint32_t can_chassis_count[4] = {0, 0, 0, 0};

PID_Regulator_t pid_motors = {15.0,1.0,0.0,0.1,7000.0,500.0,1};
PID_Regulator_t pid_velocity = {3.8,0.0,0.0,0.1,4000.0,10.0,1};
PID_Regulator_t pid_position = {40.0,0.1,0.0,0.1,9000.0,500.0,-1};

/*
 * can filter must be initialized before use
 */
void Can2_Filter_Init(CAN_HandleTypeDef* hcan)
{
    CAN_FilterConfTypeDef canfilter;

    //create memory to save the message, if not will raise error
    static CanTxMsgTypeDef  Tx2Message;
    static CanRxMsgTypeDef  Rx2Message;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    //filtrate any ID you want here
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    canfilter.FilterActivation = ENABLE;
    canfilter.BankNumber = 14;

    //use different filter for can1&can2
    
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;

    HAL_CAN_ConfigFilter(hcan, &canfilter);

}

void Can1_Filter_Init(CAN_HandleTypeDef* hcan)
{
    CAN_FilterConfTypeDef canfilter;

    //create memory to save the message, if not will raise error
    static CanTxMsgTypeDef  Tx1Message;
    static CanRxMsgTypeDef  Rx1Message;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    //filtrate any ID you want here
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    canfilter.FilterActivation = ENABLE;
    canfilter.BankNumber = 14;

    //use different filter for can1&can2
    
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;

    HAL_CAN_ConfigFilter(hcan, &canfilter);

}
/*
 * Overload the interrupt function beneath
 * it will be auto callback when can receive msg completely
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan == &hcan1) {       // CAN bus 1 is for drive motors
        if  (hcan->pRxMsg->IDE == CAN_ID_STD) {  // Check correct CAN Identifier Type

            switch (hcan->pRxMsg->StdId) {

                case 0x201:     // Motor ID = 1
                    encoderProcess(&CM1Encoder, hcan->pRxMsg);
                    break;
                case 0x202:     // Motor ID = 2
                    encoderProcess(&CM2Encoder, hcan->pRxMsg);
                    break;
                case 0x203:     // Motor ID = 3
                    encoderProcess(&CM3Encoder, hcan->pRxMsg);
                    break;
                case 0x204:     // Motor ID = 4
                    encoderProcess(&CM4Encoder, hcan->pRxMsg);
                    break;
            }

        }
				__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
    }
		if(hcan == &hcan2){
				if (hcan->pRxMsg->IDE == CAN_ID_STD){
						if (hcan->pRxMsg->StdId == 0x201){
								encoderProcess(&CM5Encoder, hcan->pRxMsg);
						}
				}
				__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
		}
  
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}

/*
 *  Input   can message
 *  Process the can bus message
 * */
void encoderProcess(volatile Encoder* ecd, CanRxMsgTypeDef* msg)
{
    // Initialize the angle
    ecd->position_raw_value_last = ecd->position_raw_value;

    // Bitwise OR operation to extract the 16-bit information
    ecd->velocity_from_ESC  = (msg->Data[2] << 8) | msg->Data[3];
    ecd->position_raw_value = (msg->Data[0] << 8) | msg->Data[1];
		ecd->position_diff = ecd->position_raw_value - ecd->position_raw_value_last;
		if(ecd->position_diff >7500){ecd->round_count++;}
		else if(ecd->position_diff < -7500){ecd->round_count--;}
		
		ecd->position_ecd_value = ecd->position_raw_value + ecd->round_count * ENCODER_MAX;
		ecd->position_ecd_bias = (int32_t)map(0, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
		ecd->ecd_angle = ecd->round_count*RADIAN_CIRCLE + (float)abs(ecd->position_raw_value - ecd->position_ecd_bias)*RADIAN_CIRCLE/ENCODER_MAX;
}

/*
 * PID Closed loop speed controller for chassis motors
 * Input
 */
void Chassis1_Set_Speed(int16_t RPM1, int16_t RPM2, int16_t RPM3, int16_t RPM4)
{

    uint8_t can1_message_chassis[8] = {0,0,0,0,0,0,0,0};

    can1_message_chassis[0] = RPM1 >> 8;
    can1_message_chassis[1] = RPM1;
    can1_message_chassis[2] = RPM2 >> 8;
    can1_message_chassis[3] = RPM2;
    can1_message_chassis[4] = RPM3 >> 8;
    can1_message_chassis[5] = RPM3;
    can1_message_chassis[6] = RPM4 >> 8;
    can1_message_chassis[7] = RPM4;

    CAN_Send_Msg(&hcan1, can1_message_chassis, 0x200);
}

void Chassis2_Set_Speed(int16_t RPM1)
{

    uint8_t can2_message_chassis[8] = {0,0,(uint8_t)5000>>8,(uint8_t)5000,(uint8_t)5000>>8,(uint8_t)5000,(uint8_t)5000>>8,(uint8_t)5000};

    can2_message_chassis[0] = RPM1 >> 8;
    can2_message_chassis[1] = RPM1;

    CAN_Send_Msg(&hcan2, can2_message_chassis, 0x200);
}
/*
 * CAN Send Message function for DJI components only
 */
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id)
{
    uint8_t index = 0;
    for(index = 0; index < 8; index++) {hcan->pTxMsg->Data[index] = msg[index];}

    hcan->pTxMsg->StdId = id;
    hcan->pTxMsg->IDE = CAN_ID_STD;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 0x08;

    HAL_CAN_Transmit(hcan, 10);
}
//PID controller
int16_t PID_Control(float measured,float target,PID_Regulator_t * pid, int8_t address){
		static float error_v[2]={0.0,0.0};
		static float output = 0;
		static float inte[6] = {0,0,0,0,0,0};
		
		if(fabs(measured) < pid->GAP){measured = 0.0;}
		
		error_v[0] = error_v[1];
		error_v[1] = target - measured;
		inte[address] += error_v[1];
		if(inte[address] > pid->inte_Max){inte[address] = pid->inte_Max;}
		if(inte[address] < -pid->inte_Max){inte[address] = -pid->inte_Max;}		
		
		output = error_v[1] * pid->kp + inte[address] * pid->ki + (error_v[1]-error_v[0])*pid->kd;
		output = output * pid->sign;
		
		if(output > pid->ESC_Max){output = pid->ESC_Max;}
		if(output < -pid->ESC_Max){output = -pid->ESC_Max;}
		
		return (int16_t)output;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
