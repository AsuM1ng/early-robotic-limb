
/********************************************************************************
*@brife 此文件主要是配合串口示波器使用，用于实时显示相关参数的曲线，只能输出不能输入
		1.一般讲曲线打印的相关的任务放在while（1）函数中
		
*******************************************************************************/

#include "chipHAL_USART.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

#define	CHAL_USART                  USART3
#define	CHAL_USART_CLK              RCC_APB1Periph_USART3
#define	CHAL_USART_IRQn             USART3_IRQn
#define	CHAL_USART_IRQHandler       USART3_IRQHandler

#define CHAL_USART_GPIO_POART       GPIOB
#define CHAL_USART_GPIO             RCC_AHB1Periph_GPIOB
#define CHAL_USART_TX_PIN           GPIO_PinSource10
#define CHAL_USART_RX_PIN           GPIO_PinSource11


//////////////////////////////////////////////////////////////////
#if 1
#pragma import(__use_no_semihosting)                        
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
void _sys_exit(int x) 
{
	x = x; 
} 

int fputc(int ch, FILE *f)
{
	while((USART3->SR&0X40)==0);
	USART3->DR = (u8) ch;      
	return ch;
}
#endif
 

uint8_t USART_RX_BUF[USART_REC_LEN];

uint16_t USART_RX_STA = 0;

void chipHAL_UsartInit(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
#if EN_USART3_RX
    NVIC_InitTypeDef NVIC_InitStructure;
    #endif

	RCC_APB1PeriphClockCmd(CHAL_USART_CLK,ENABLE);
    RCC_AHB1PeriphClockCmd(CHAL_USART_GPIO,ENABLE);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);
    USART_ClearFlag(USART3, USART_FLAG_TC);

#if EN_USART3_RX
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    #endif
}


#if EN_USART3_RX

void USART3_IRQHandler(void)
{
	uint8_t Res;
#if SYSTEM_SUPPORT_OS
	OSIntEnter();
    #endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		Res =USART_ReceiveData(USART3);
		
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(Res!=0x0a)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;
			}
			else
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS
	OSIntExit();  											 
#endif
}
#endif	

 


