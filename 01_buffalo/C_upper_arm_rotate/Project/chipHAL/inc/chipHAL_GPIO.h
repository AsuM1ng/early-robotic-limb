/**
 * COPYRIGHT NOTICE
 * Copyright (C) 2018, Buffalo Robot Tech.co, Ltd
 * All rights reserved.
 *
 * @file 	chipHAL_GPIO.h
 * @brief   This file contains all the functions prototypes for the GPIO firmware
 *          library. Some of stm32 lib are used there.
 *
 * @author  Chen Huachuan
 * @email   Chenhc@buffalo-robot.com
 * @date    Dec.04, 2017
 * @version Software v1.0.0.0
 */
#ifndef _GPIO_H__
#define _GPIO_H__

#include "stm32f4xx_gpio.h"	
#include "Board_type_config.h"


/**
* CANID GPIO interface
*/
#if(TEST_BOARD)  /*DEBUG 时使用的是下肢的节点板，CAN ID针脚定义不同*/
    #define CANID0_GPIO_PIN    GPIO_Pin_4
    #define CANID0_GPIO_PORT   GPIOB

    #define CANID1_GPIO_PIN    GPIO_Pin_2
    #define CANID1_GPIO_PORT   GPIOD

    #define CANID2_GPIO_PIN    GPIO_Pin_11
    #define CANID2_GPIO_PORT   GPIOC

    #define CANID3_GPIO_PIN    GPIO_Pin_15
    #define CANID3_GPIO_PORT   GPIOA

    /**
    * CANID GPIO clock
    */
    #define CANID0_GPIO_CLK    RCC_AHB1Periph_GPIOB
    #define CANID1_GPIO_CLK    RCC_AHB1Periph_GPIOD
    #define CANID2_GPIO_CLK    RCC_AHB1Periph_GPIOC
    #define CANID3_GPIO_CLK    RCC_AHB1Periph_GPIOA
#else

    #define CANID0_GPIO_PIN    GPIO_Pin_9
    #define CANID0_GPIO_PORT   GPIOC

    #define CANID1_GPIO_PIN    GPIO_Pin_8
    #define CANID1_GPIO_PORT   GPIOA

    #define CANID2_GPIO_PIN    GPIO_Pin_9
    #define CANID2_GPIO_PORT   GPIOA

    #define CANID3_GPIO_PIN    GPIO_Pin_10
    #define CANID3_GPIO_PORT   GPIOA
    
    /**
    * CANID GPIO clock
    */
    #define CANID0_GPIO_CLK    RCC_AHB1Periph_GPIOC
    #define CANID1_GPIO_CLK    RCC_AHB1Periph_GPIOA
    #define CANID2_GPIO_CLK    RCC_AHB1Periph_GPIOA
    #define CANID3_GPIO_CLK    RCC_AHB1Periph_GPIOA

#endif



/**
* LED GPIO interface
*/

#if (TEST_BOARD)
    #define LED_GPIO_PORT       GPIOC
    #define RCC_LED_GPIO_CLOCK  RCC_AHB1Periph_GPIOC
    
    #define LED_G GPIO_Pin_6
    #define LED_B GPIO_Pin_7
    #define LED_R GPIO_Pin_8
    #define LED_W GPIO_Pin_9
#else

    #define LED_GPIO_PORT       GPIOC
    #define RCC_LED_GPIO_CLOCK  RCC_AHB1Periph_GPIOC
    
    #define LED_G               GPIO_Pin_10
    #define LED_B               GPIO_Pin_11
    #define LED_R               GPIO_Pin_12

#endif



/**
* USART GPIO interface
*/
///< None

/**
* AbsEncoder GPIO interface
*/
#if (('A' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE))
	#define ABSOLUTE_ENCODER_CLOCK_GPIO_PORT   GPIOB
	#define ABSOLUTE_ENCODER_CLOCK_GPIO_PIN    GPIO_Pin_8

	#define ABSOLUTE_ENCODER_DATA_GPIO_PORT    GPIOB
	#define ABSOLUTE_ENCODER_DATA_GPIO_PIN     GPIO_Pin_9
#endif



#define TEST_GPIO_PORT                          GPIOB
#define TEST_GPIO_CLOCK                         RCC_AHB1Periph_GPIOB
#define TEST_GPIO_PINTX                         GPIO_Pin_10
#define TEST_GPIO_PINRX                         GPIO_Pin_11

/**
* function interface
*/
void GPIO_CanIDInit(void);
void GPIO_LedInit(void);
void GPIO_UsartInit(void);
void TestGpio_USART_Init(void);

#endif

