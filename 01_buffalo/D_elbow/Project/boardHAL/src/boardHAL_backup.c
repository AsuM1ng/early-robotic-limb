/*****************************************************************
 * COPYRIGHT NOTICE
 * Copyright (C) 2018, Buffalo Robot Tech.co, Ltd
 * All rights reserved.
 *
 * @file 	BHAL.c
 * @brief   This file contains all the functions prototypes for the GPIO firmware
 *          library. Some of stm32 lib are used there.
 *
 * @author  chenhuachuan
 * @email   Chenhc@buffalo-robot.com
 * @date    07.25, 2018
 * @version Software v1.0.0.0
******************************************************************/

#include "boardHAL.h"

/***************** BHAL *******************/
//BHAL_Typedef 			gBHAL;		//Board Hardware Abstract Layer module.

/**********************************************************************
*
*    init relevant abstrct functions
*
*    void ISR_Init(void);     //Initilize Interupt
*    void BHAL_Init(void);    //inInitilize Board Hardware Abstract Layer
*
***********************************************************************/

/**********************************************************************
*                 Methods of enable/disable all the interupts
* method1：
* core_cmFunc.h
*	__disable_irq();
*	__enable_irq();
*
* method2:
* core_cm4.h
*	NVIC_EnableIRQ(IRQn);
*	NVIC_DisableIRQ(IRQn);
*
**********************************************************************
*	__WFI();    //make the chip sleep, any interupt will wake up the chip
**********************************************************************/

/**********************************************************************
*
*       					Interupt Initialization
*
**********************************************************************/
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}

//disable all the interupt(not including fault or NMI interupt)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}

//enable all the interupt
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}

//setup the top address of the stack
//addr:Top address of the stack
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}


void ISR_Init(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);				//3 bits of pre_priority, 1 bit of sub_priority

	/* ADC的中断：程序中使用EOC/JEOC中断，即ADC转换完成中断 */
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;									
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_PRIORITY;		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									
	NVIC_Init(&NVIC_InitStructure);
	
	/* incenc的Z相中断 */
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_CHZ_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ENCODER_CHZ_PRE;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = ENCODER_CHZ_SUB;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	
	NVIC_Init(&NVIC_InitStructure);	
	
	/* servo interupt */
	NVIC_InitStructure.NVIC_IRQChannel = SERVO_TIM_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SERVO_PRE_PRIORITY;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SERVO_SUB_PRIORITY;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*******************************
*
*       Initilize the Board HAL
*
********************************/
int poweron_delay = 8000;
extern uint8_t clock_electrical_level;
void BHAL_Init(void)
{	

	__disable_irq();		//< Disable the all IRQs before initializing.
	
	/* Initialize Node-ID switch */
	GPIO_NodeIDSwitchInit();
	
	
	/* Initialize LED */
	GPIO_LED_Init();
	
	/* Initialize Buzzer */
	
	/* USART configuration */
	GPIO_USART_Init();
	CHAL_USART_Init(115200);
		
	/* PWM Initialization*/
	GPIO_PWM_Init();
	TIM_PWM_Init();
	TIM_CtrlPWMOutputs(TIM8, ENABLE);						///< Enable TIM8 MOE.
	
	while (poweron_delay--);								///< Wait for PWM stable.
	
	/**
	* Phase current Initialization
	*/
	GPIO_PhaseCurrent_Init();
	ADC_PhaseCurrent_Init();
		
	/**
	* Servo timebase Initialization
	*/
	TIM_Servo_Init();
	
    
    /* Absolute Encoder Initialization */
    GPIO_AbsEncoder_Init();
    GPIO_SetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
    clock_electrical_level = 1;
	
//    TIM_Delay_start();
//    EXTIX_Init();
	
	/* ISR Initialization */
	ISR_Init();
	
	__enable_irq();					///< After all peripherals have been initialized, IRQs are enabled.
}

/**********************************************************************
*
*    hardware reset
*
***********************************************************************/
void BHAL_Reset(void)
{
	//__set_FAULTMASK(1);
	//NVIC_SystemReset();
}

/****************************************************************
*
*    RS232 abstraction function
*
****************************************************************/
void BHAL_Rs232Tx( uint8_t  * source , uint8_t num)
{
   //USART1_Tx( source , num);
}
uint8_t BHAL_Rs232TxLeft(void)
{
   return 0;// USART1_TxLeft();
}
uint8_t BHAL_Rs232RxNum(void)
{
   return 0;//USART1_RxNum();
}
uint8_t BHAL_Rs232Rx(uint8_t * destination ,uint8_t num)
{
   return 0;//USART1_Rx( destination , num);
}

/**************************************************************
*
*               time relavant abstraction function
*
**************************************************************/


/*********************************************
*	BHAL_DelayUs(uint32_t us);
*   作用：精确的微秒级别延时，阻塞式
*	输入参数：需要延时的微秒数
*   返回值：无
*********************************************/
void BHAL_DelayUs(uint32_t us)
{
uint8_t i;
	while(us--)
	{
		i=33;//实测每微秒相当于36个循环
		while(i--);
	}
}

/*********************************************
*	BHAL_SysTimerInit(uint32_t *timer)
*   作用：初始化一个系统ms级别定时器，定时器本质是一个32位的变量
*	输入参数：定时器指针，
*   返回值：无
*********************************************/ 
void BHAL_SysTimerCreater(uint32_t *pTimer)
{
	 *pTimer=0;
}
  //该函数并不是精确的ms级定时 
/*********************************************
*	BHAL_SysTimerMs(uint32_t *pTimer  ,uint16_t howMuchTime)
*   作用： 查询系统定时器离上次触发是否已经间隔定时时间， 
           第一次使用的时候，定时器不会被触发，
*	输入参数：pTimer定时器指针， howMuchTime 定时时间，单位为ms
*   返回值：0：未到定时时间，1：到定时时间
*********************************************/ 
uint16_t BHAL_SysTimerMs(uint32_t *pTimer  ,uint16_t howMuchTime)
{
	return 0;
}

/********************
*
*    CRC相关函数
*
********************/
void BHAL_ResetCrc(void)
{
	  CRC->CR=1;
}

uint32_t BHAL_GetCrc(void)
{
	return CRC->DR;
}

void BHAL_InputCrc(uint32_t val)
{
	CRC->DR=val;
}


/**************************
	绿灯亮控制函数，内部使用
**************************/
void BHAL_LedBlueLight(void)
{
}

void BHAL_LedBlueDark(void)
{
}

void BHAL_LedRedLight(void)
{
}

void BHAL_LedRedDark(void)
{
}

/********************
* BHAL power switch
*********************/
void BHAL_PowerOn(void)
{	 
}
void BHAL_PowerOff(void)
{	 
}


