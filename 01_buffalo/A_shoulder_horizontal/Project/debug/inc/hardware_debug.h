

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_DEBUG_H_
#define __HARDWARE_DEBUG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

/** @addtogroup APP
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


#define IO_TEST_GPIO_PIN   		GPIO_Pin_6 
#define IO_TEST_GPIO_PORT  		GPIOA
#define IO_TEST_GPIO_CLK   		RCC_AHB1Periph_GPIOA


#define No_Trigger
//#define External_Independent_Trigger 
//#define External_Synchronous_Trigger
//#define Software_Synchronous_Trigger

#define DAC_Trigger_1              DAC_Trigger_T6_TRGO
#define DAC_Trigger_2              DAC_Trigger_T8_TRGO

/* Exported functions --------------------------------------------------------*/
void debug_init(void);
void IO_switch(uint8_t status);
 
void Dac1_Set_Vol(int16_t vol);	//设置通道1输出电压
void Dac2_Set_Vol(int16_t vol);	//设置通道1输出电压

#endif /* __RIOM_UI_H */

/**
  * @}
  */ 

/**
  * @}
  */
   
/******************* (C) COPYRIGHT 2012 PRMI *****END OF FILE****/
