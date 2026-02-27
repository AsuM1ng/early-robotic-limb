
#include "chipHAL_CAN.h"
#include "stm32f4xx_can.h"
#include "Board_type_config.h"

#define BLDC_CAN1_CLK                    RCC_APB1Periph_CAN1
#define BLDC_CAN1_RX_PIN                 GPIO_Pin_11
#define BLDC_CAN1_TX_PIN                 GPIO_Pin_12
#define BLDC_CAN1_GPIO_PORT              GPIOA
#define BLDC_CAN1_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define BLDC_CAN1_AF_PORT                GPIO_AF_CAN1
#define BLDC_CAN1_RX_SOURCE              GPIO_PinSource11
#define BLDC_CAN1_TX_SOURCE              GPIO_PinSource12 


int CAN1_Config(uint32_t bitrate)
{
	GPIO_InitTypeDef  		GPIO_InitStructure;
	CAN_InitTypeDef 		CAN_InitStructure;
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef 		NVIC_InitStructure;

	CAN_DeInit(NODE_CAN1);

	/* GPIO clock enable */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHB1PeriphClockCmd(BLDC_CAN1_GPIO_CLK, ENABLE);
	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(BLDC_CAN1_CLK, ENABLE);

	GPIO_PinAFConfig(BLDC_CAN1_GPIO_PORT,BLDC_CAN1_RX_SOURCE,BLDC_CAN1_AF_PORT);
	GPIO_PinAFConfig(BLDC_CAN1_GPIO_PORT,BLDC_CAN1_TX_SOURCE,BLDC_CAN1_AF_PORT);

	/* Configure CAN pin: RX */
	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = BLDC_CAN1_TX_PIN | BLDC_CAN1_RX_PIN ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(BLDC_CAN1_GPIO_PORT, &GPIO_InitStructure);

	/* CAN register init */
    //	CAN_DeInit(CAN1);/* 通过 RCC_APB1RSTR 寄存器来复位 CAN1 */
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;/* 优先级由请求顺序决定 */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	/*CAN的时钟是APB1，APB1=42M ，CAN波特率=RCC_APNB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler*/
    switch(bitrate)
	{
		case 1000000://1Mbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 3; 				
		break;			
		case 500000://500Kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 6; 				
		break;
		case 250000://250Kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 12; 				
		break;
		default:;
    }
	CAN_Init(NODE_CAN1, &CAN_InitStructure);
    
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;															   	
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(NODE_CAN1,CAN_IT_FMP0, ENABLE);
    
	/* Configure one bit for preemption priority */
    //	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	/* Enable CAN1 RX0 interrupt IRQ channel */
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);		
	return 0;
}

#define BLDC_CAN2_GPIO_PORT              GPIOB
#define BLDC_CAN2_AF_PORT                GPIO_AF_CAN2
#define BLDC_CAN2_CLK                    RCC_APB1Periph_CAN2
#define BLDC_CAN2_GPIO_CLK               RCC_AHB1Periph_GPIOB

#if ('A' == JOINTBOARD_TYPE) || ('C' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE)

#define BLDC_CAN2_RX_PIN                 GPIO_Pin_5
#define BLDC_CAN2_TX_PIN                 GPIO_Pin_6

#define BLDC_CAN2_RX_SOURCE              GPIO_PinSource5
#define BLDC_CAN2_TX_SOURCE              GPIO_PinSource6

#elif ('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE)

#define BLDC_CAN2_RX_PIN                 GPIO_Pin_12
#define BLDC_CAN2_TX_PIN                 GPIO_Pin_13

#define BLDC_CAN2_RX_SOURCE              GPIO_PinSource12
#define BLDC_CAN2_TX_SOURCE              GPIO_PinSource13

#endif


int CAN2_Config(uint32_t bitrate)
{
	GPIO_InitTypeDef  		GPIO_InitStructure;
	CAN_InitTypeDef 		CAN_InitStructure;
	CAN_FilterInitTypeDef 	CAN_FilterInitStructure;
	NVIC_InitTypeDef 		NVIC_InitStructure;

	CAN_DeInit(NODE_CAN2);

	/* GPIO clock enable */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHB1PeriphClockCmd(BLDC_CAN2_GPIO_CLK, ENABLE);
    /* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(BLDC_CAN2_CLK, ENABLE);

	GPIO_PinAFConfig(BLDC_CAN2_GPIO_PORT, BLDC_CAN2_RX_SOURCE, BLDC_CAN2_AF_PORT);
	GPIO_PinAFConfig(BLDC_CAN2_GPIO_PORT, BLDC_CAN2_TX_SOURCE, BLDC_CAN2_AF_PORT);

	/* Configure CAN pin: RX */
	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BLDC_CAN2_TX_PIN | BLDC_CAN2_RX_PIN ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(BLDC_CAN2_GPIO_PORT, &GPIO_InitStructure);


	/* CAN register init */
    //	CAN_DeInit(CAN2);/* 通过 RCC_APB1RSTR 寄存器来复位 CAN2 */
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;/* 优先级由请求顺序决定 */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/*CAN的时钟是APB1，APB1=42M ，CAN波特率=RCC_APNB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler*/
    switch(bitrate)
	{
		case 1000000://1Mbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 3; 				
		break;			
		case 500000://500Kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 6; 				
		break;
		case 250000://250Kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
			CAN_InitStructure.CAN_Prescaler = 12; 				
		break;
		default:;
    }
	CAN_Init(NODE_CAN2, &CAN_InitStructure);
    
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;															   	
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(NODE_CAN2,CAN_IT_FMP0, ENABLE);
	/* Configure one bit for preemption priority */
    //	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	/* Enable CAN2 RX0 interrupt IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	
	NVIC_Init(&NVIC_InitStructure);	
	return 0;
}
