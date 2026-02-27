/**
	***********************************
	* 文件名:	CANOpen_can.h
	* 作者:		stone
	* 版本:		V0.1
	* 日期:		2018-3-29
	* 描述:		CANOPEN协议底层总线接口
	***********************************
	*/
#ifndef __canfestival_can_H_
#define __canfestival_can_H_

/* 功能:	can总线配置
	 参数:	无
	 返回值:无
 */
 


#include "stm32f4xx.h"
#include <stdio.h>

extern CAN_HandleTypeDef Can_Handle;

//CAN1
//#define CANx                       CAN2
#define CAN1_CLK_ENABLE()          __CAN1_CLK_ENABLE()
#define CAN1_RX_IRQ				         CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler		     CAN1_RX0_IRQHandler

#define CAN1_RX_PIN                 GPIO_PIN_11
#define CAN1_TX_PIN                 GPIO_PIN_12
#define CAN1_TX_GPIO_PORT           GPIOA
#define CAN1_RX_GPIO_PORT           GPIOA
#define CAN1_TX_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define CAN1_RX_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define CAN1_AF_PORT                GPIO_AF9_CAN1 


//#define CANx                       CAN2
#define CAN2_CLK_ENABLE()           __CAN2_CLK_ENABLE()
#define CAN2_RX_IRQ				          CAN2_RX0_IRQn
#define CAN2_RX_IRQHandler		      CAN2_RX0_IRQHandler

#define CAN2_RX_PIN                 GPIO_PIN_5
#define CAN2_TX_PIN                 GPIO_PIN_6
#define CAN2_TX_GPIO_PORT           GPIOB
#define CAN2_RX_GPIO_PORT           GPIOB
#define CAN2_TX_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define CAN2_RX_GPIO_CLK_ENABLE()   __GPIOB_CLK_ENABLE()
#define CAN2_AF_PORT                GPIO_AF9_CAN2 


void CANOpen_can_config(void);
static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(void);
void CAN_Config(void);
void CAN_SetMsg(void);
void Init_RxMes(void);
#endif