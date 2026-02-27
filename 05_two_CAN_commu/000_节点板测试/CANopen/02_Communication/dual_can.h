#ifndef __DUAL_CAN_H
#define __DUAL_CAN_H

#include "sys.h"
#include "main.h"
#include "data.h"
// CAN bus defines for cortex-M4 STM32F407

//can1
//#define CANx                       CAN1
#define CAN1_CLK                    RCC_APB1Periph_CAN1
#define CAN1_RX_PIN                 GPIO_Pin_11
#define CAN1_TX_PIN                 GPIO_Pin_12
#define CAN1_GPIO_PORT              GPIOA
#define CAN1_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define CAN1_RX0_IRQn               CAN1_RX0_IRQn

//#define GPIO_AF_CAN1               GPIO_AF_CAN1
#define CAN1_RX_SOURCE              GPIO_PinSource11
#define CAN1_TX_SOURCE              GPIO_PinSource12

//can2
//#define CANx                       CAN1
#define CAN2_CLK                    RCC_APB1Periph_CAN2
#define CAN2_RX_PIN                 GPIO_Pin_5
#define CAN2_TX_PIN                 GPIO_Pin_6
#define CAN2_GPIO_PORT              GPIOB
#define CAN2_GPIO_CLK               RCC_AHB1Periph_GPIOB
#define CAN2_RX0_IRQn               CAN2_RX0_IRQn

//#define GPIO_AF_CAN_2               GPIO_AF_CAN2
#define CAN2_RX_SOURCE              GPIO_PinSource5
#define CAN2_TX_SOURCE              GPIO_PinSource6


unsigned char CAN1_Init(CO_Data * d, uint32_t bitrate);
unsigned char CAN2_Init(CO_Data * d, uint32_t bitrate);
u8 can_init(void);


#endif 

