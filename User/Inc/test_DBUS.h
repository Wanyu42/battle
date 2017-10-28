#ifndef __TEST_DEBUS_H_
#define __TEST_DEBUS_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
/* ----------------------- RC Channel Definition---------------------------- */ 
#define RC_CH_VALUE_MIN              ((uint16_t)364 ) 
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024) 
#define RC_CH_VALUE_MAX              ((uint16_t)1684)
#define DBUS_BUFFER_SIZE             ((uint8_t)18)
typedef union{		
		struct{
			uint16_t channel0;
			uint16_t channel1;
			uint16_t channel2;
			uint16_t channel3;
			uint8_t  s1;
			uint8_t  s2;
		}rc;
}RC_Ctl_t;
extern uint8_t USART_RX_BUF[18];
extern volatile RC_Ctl_t RC_Ctl;
void RC_Init(void);
void decryptDBUS(void);
#endif /* __TEST_DEBUS_H_ */
