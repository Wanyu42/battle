/** 
  *@file test_DBUS.h 
  *@date 2017-09-28 
  *@author ZK 
  *@brief  
  */

#include "test_DBUS.h"
#include "usart.h"

uint8_t USART_RX_BUF[18];
volatile RC_Ctl_t RC_Ctl;
extern DMA_HandleTypeDef hdma_usart1_rx;

//Initialization 
void RC_Init()
{
    RC_Ctl.rc.channel0 = RC_CH_VALUE_OFFSET;
    RC_Ctl.rc.channel1 = RC_CH_VALUE_OFFSET;
    RC_Ctl.rc.channel2 = RC_CH_VALUE_OFFSET;
    RC_Ctl.rc.channel3 = RC_CH_VALUE_OFFSET;
    // MX_USART1_UART_Init();
    HAL_UART_Init(&huart1);//Configue all needed for DUBS
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // Enable IDLE interrupt function
    HAL_UART_Receive_DMA(&huart1, USART_RX_BUF, 18);    //receive now
}

//Interrupt function 
void decryptDBUS()
{
    RC_Ctl.rc.channel0 = ((USART_RX_BUF[0]) | (USART_RX_BUF[1] << 8)) & 0x07FF;
    RC_Ctl.rc.channel1 = ((USART_RX_BUF[1] >> 3) | (USART_RX_BUF[2] << 5)) & 0x07FF;
    RC_Ctl.rc.channel2 =
            ((USART_RX_BUF[2] >> 6) | (USART_RX_BUF[3] << 2) | ((uint32_t) USART_RX_BUF[4] << 10)) & 0x07FF;
    RC_Ctl.rc.channel3 = ((USART_RX_BUF[4] >> 1) | (USART_RX_BUF[5] << 7)) & 0x07FF;
    RC_Ctl.rc.s1 = ((USART_RX_BUF[5] >> 4) & 0x000C) >> 2;                         //!< Switch left
    RC_Ctl.rc.s2 = ((USART_RX_BUF[5] >> 4) & 0x0003);
} 
 
