/**
  ******************************************************************************
  * @file    canfestival.c
  * @author  Linp
  * @version V1.0
  * @date    2024.9.27
  * @brief   can驱动（正常工作模式）
  ******************************************************************************
**/

#include "canfestival_can.h"
#include "applicfg.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"

CAN_HandleTypeDef Can1_Handle;		//CAN1句柄
CAN_HandleTypeDef Can2_Handle;		//CAN2句柄
CanTxMsgTypeDef Tx1Message;		//CAN1发送缓冲区
CanTxMsgTypeDef Tx2Message;		//CAN2发送缓冲区
CanRxMsgTypeDef Rx1Message;		//CAN1接收缓冲区
CanRxMsgTypeDef Rx2Message;		//CAN2接收缓冲区

extern __IO uint32_t flag ;		 //用于标志是否接收到数据，在中断函数中赋值
/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能引脚时钟 */
    CAN1_TX_GPIO_CLK_ENABLE();
    CAN1_RX_GPIO_CLK_ENABLE();	
	  CAN2_TX_GPIO_CLK_ENABLE();
    CAN2_RX_GPIO_CLK_ENABLE();	

    /* 配置CAN1发送引脚 */
    GPIO_InitStructure.Pin = CAN1_TX_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Alternate =  GPIO_AF9_CAN1;
    HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
	
	  //配置CAN2发送引脚
	  GPIO_InitStructure.Pin = CAN2_TX_PIN;
	  HAL_GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
	  

    /* 配置CAN1接收引脚 */
    GPIO_InitStructure.Pin = CAN1_RX_PIN ;
    HAL_GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
		
		/* 配置CAN2接收引脚 */
    GPIO_InitStructure.Pin = CAN2_RX_PIN ;
    HAL_GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
}



/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
	/* 配置抢占优先级的分组 */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);
	/*中断设置，抢占优先级0，子优先级为0*/
	HAL_NVIC_SetPriority(CAN1_RX_IRQ | CAN2_RX_IRQ, 0 ,0);
	HAL_NVIC_EnableIRQ(CAN1_RX_IRQ | CAN2_RX_IRQ);
}



/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
	
	/************************CAN1通信参数设置**********************************/
	/* 使能CAN时钟 */
    __CAN1_CLK_ENABLE();
    __CAN2_CLK_ENABLE();
    Can1_Handle.Instance = CAN1;
    Can1_Handle.pTxMsg = &Tx1Message;
    Can1_Handle.pRxMsg = &Rx1Message;
	/* CAN单元初始化 */
	Can1_Handle.Init.TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	Can1_Handle.Init.ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
	Can1_Handle.Init.AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	Can1_Handle.Init.NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	Can1_Handle.Init.RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	Can1_Handle.Init.TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	Can1_Handle.Init.Mode = CAN_MODE_NORMAL;    //正常工作模式
	Can1_Handle.Init.SJW=CAN_SJW_1TQ;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
	 
	/* ss=1 bs1=3 bs2=3 位时间宽度为(1+3+3) 波特率即为时钟周期tq*(1+3+3)  */
	Can1_Handle.Init.BS1=CAN_BS1_3TQ;		   //BTR-TS1 时间段1 占用了3个时间单元
	Can1_Handle.Init.BS2=CAN_BS2_3TQ;		   //BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	Can1_Handle.Init.Prescaler =6;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+3+3)/6=1 Mbps
	HAL_CAN_Init(&Can1_Handle);
	
	
		/************************CAN2通信参数设置**********************************/
		/* 使能CAN时钟 */
    __CAN1_CLK_ENABLE();
    __CAN2_CLK_ENABLE();
    Can2_Handle.Instance = CAN2;
    Can2_Handle.pTxMsg = &Tx2Message;
    Can2_Handle.pRxMsg = &Rx2Message;
	/* CAN单元初始化 */
	Can2_Handle.Init.TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	Can2_Handle.Init.ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
	Can2_Handle.Init.AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	Can2_Handle.Init.NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	Can2_Handle.Init.RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	Can2_Handle.Init.TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	Can2_Handle.Init.Mode = CAN_MODE_NORMAL;    //正常工作模式
	Can2_Handle.Init.SJW=CAN_SJW_1TQ;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
	 
	/* ss=1 bs1=3 bs2=3 位时间宽度为(1+3+3) 波特率即为时钟周期tq*(1+3+3)  */
	Can2_Handle.Init.BS1=CAN_BS1_3TQ;		   //BTR-TS1 时间段1 占用了3个时间单元
	Can2_Handle.Init.BS2=CAN_BS2_3TQ;		   //BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	Can2_Handle.Init.Prescaler =6;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+3+3)/6=1 Mbps
	HAL_CAN_Init(&Can2_Handle);
	
		
		
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
	CAN_FilterConfTypeDef  CAN_FilterInitStructure;

		/************************CAN1滤波器设置**********************************/
	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.FilterNumber=0;						//筛选器组0
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	//工作在掩码模式
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.FilterIdHigh= 00000;		//要筛选的ID高位 
	CAN_FilterInitStructure.FilterIdLow= 0x0000; //要筛选的ID低位 
	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	//筛选器被关联到FIFO0
	CAN_FilterInitStructure.FilterActivation=ENABLE;			//使能筛选器
	HAL_CAN_ConfigFilter(&Can1_Handle,&CAN_FilterInitStructure);
	
	/************************CAN2滤波器设置**********************************/
	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.FilterNumber=14;						//筛选器组0
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	//工作在掩码模式
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.FilterIdHigh= 0x0000;		//要筛选的ID高位 
	CAN_FilterInitStructure.FilterIdLow= 0x0000; //要筛选的ID低位 
	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	//筛选器被关联到FIFO0
	CAN_FilterInitStructure.FilterActivation=ENABLE;			//使能筛选器
	HAL_CAN_ConfigFilter(&Can2_Handle,&CAN_FilterInitStructure);
}


/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();
  Init_RxMes(); 
  HAL_CAN_Receive_IT(&Can1_Handle, CAN_FIFO0); 	
	HAL_CAN_Receive_IT(&Can2_Handle, CAN_FIFO0);
}


/**
  * @brief  初始化 Rx Message数据结构体
  * @param  RxMessage: 指向要初始化的数据结构体
  * @retval None
  */
void Init_RxMes(void)
{
  uint8_t ubCounter = 0;

  /*把CAN1接收结构体清零*/
  Can1_Handle.pRxMsg->StdId = 0x00;
  Can1_Handle.pRxMsg->ExtId = 0x00;
  Can1_Handle.pRxMsg->IDE = CAN_ID_STD;
  Can1_Handle.pRxMsg->DLC = 0;
  Can1_Handle.pRxMsg->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    Can1_Handle.pRxMsg->Data[ubCounter] = 0x00;
  }
	
	/*把CAN2接收结构体清零*/
  Can2_Handle.pRxMsg->StdId = 0x00;
  Can2_Handle.pRxMsg->ExtId = 0x00;
  Can2_Handle.pRxMsg->IDE = CAN_ID_STD;
  Can2_Handle.pRxMsg->DLC = 0;
  Can2_Handle.pRxMsg->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    Can2_Handle.pRxMsg->Data[ubCounter] = 0x00;
  }
}


/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */	 
void CAN_SetMsg(void)
{	  
  uint8_t ubCounter = 0;
  Can1_Handle.pTxMsg->StdId=0x00;						 
  Can1_Handle.pTxMsg->ExtId=0x1314;					 //使用的扩展ID
  Can1_Handle.pTxMsg->IDE=CAN_ID_EXT;				  //扩展模式
  Can1_Handle.pTxMsg->RTR=CAN_RTR_DATA;				 //发送的是数据
  Can1_Handle.pTxMsg->DLC=8;							 //数据长度为8字节
	
  /*设置要发送的数据0-7*/
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    Can1_Handle.pTxMsg->Data[ubCounter] = ubCounter;
  }
	

  Can2_Handle.pTxMsg->StdId=0x00;						 
  Can2_Handle.pTxMsg->ExtId=0x1314;					 //使用的扩展ID
  Can2_Handle.pTxMsg->IDE=CAN_ID_EXT;				  //扩展模式
  Can2_Handle.pTxMsg->RTR=CAN_RTR_DATA;				 //发送的是数据
  Can2_Handle.pTxMsg->DLC=8;							 //数据长度为8字节
	
  /*设置要发送的数据0-7*/
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    Can1_Handle.pTxMsg->Data[ubCounter] = ubCounter;
  }
}

/**
  * @brief  CAN接收完成中断(非阻塞) 
  * @param  hcan: CAN句柄指针
  * @retval 无
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	/* 比较ID是否为0x1314 */ 
	if((hcan->pRxMsg->ExtId==0x1314) && (hcan->pRxMsg->IDE==CAN_ID_EXT) && (hcan->pRxMsg->DLC==8) )
	{
		flag = 1; //接收成功  
	}
	else
	{
		flag = 0; //接收失败
	}
	/* 准备中断接收 */
	HAL_CAN_Receive_IT(&Can_Handle, CAN_FIFO0);
}
/**
  * @brief  CAN错误回调函数
  * @param  hcan: CAN句柄指针
  * @retval 无
  */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	printf("\r\nCAN出错\r\n");
}




unsigned char canSend(CAN_HandleTypeDef *hcan, Message *m)
{
	int i, res;
	CanTxMsgTypeDef TxMessage = {0};
	TxMessage.StdId = m->cob_id;
	TxMessage.IDE = CAN_ID_STD;
	if(m->rtr)
  		TxMessage.RTR = CAN_RTR_REMOTE;
	else
  		TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = m->len;
	for(i=0 ; i<m->len ; i++)
		TxMessage.Data[i] = m->data[i]; 
		if(hcan->Instance == CAN1)
			res = HAL_CAN_Transmit(&Can1_Handle, 15);
		else
			res = HAL_CAN_Transmit(&Can2_Handle, 15);
	if(res == CAN_TXSTATUS_NOMAILBOX)
		return 0; 	// error
    return 1;		// succesful
}


void CAN1_RX0_IRQHandler(void)
{
	Message Rx_Message;
	uint16_t res;
	/* ???? */
	res = HAL_CAN_Receive(&Can1_Handle, CAN_FIFO0, 15);

	/* ??canopen??? */
	if(res == HAL_OK)
	{
		Rx_Message.cob_id = Can1_Handle.pRxMsg->StdId;						/* ??????ID */
		Rx_Message.rtr = (Can1_Handle.pRxMsg->RTR == CAN_RTR_DATA) ? 0 : 1;	/* ????? */
		Rx_Message.len = Can1_Handle.pRxMsg->DLC;							/* ????? */
		memcpy(Rx_Message.data, Can1_Handle.pRxMsg->Data, Can1_Handle.pRxMsg->DLC);		/* ?? */
	}

	/* canopen????????? */
	canDispatch(&masterObjdict_Data, &Rx_Message);
}










/**************************END OF FILE************************************/
