/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

uc ucCanRxBuff[64];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

  //
CAN_HandleTypeDef	CAN1_Handler;   //CAN1句柄
CAN_TxHeaderTypeDef	CAN1_TxHeader;      //发送
CAN_RxHeaderTypeDef	CAN1_RxHeader;      //接收

CAN_HandleTypeDef	CAN2_Handler;   //CAN2句柄
CAN_TxHeaderTypeDef	CAN2_TxHeader;      //发送
CAN_RxHeaderTypeDef	CAN2_RxHeader;      //接收

CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1TQ~CAN_SJW_4TQ
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1TQ~CAN_BS2_8TQ;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1TQ~CAN_BS1_16TQ
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+tbs2+1)*brp); 其中tbs1和tbs2我们只用关注标识符上标志的序号，例如CAN_BS2_1TQ，我们就认为tbs2=1来计算即可。
//mode:CAN_MODE_NORMAL,普通模式;CAN_MODE_LOOPBACK,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_MODE_LOOPBACK);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 

u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode)
{
	CAN_InitTypeDef		CAN1_InitConf;
    
    CAN1_Handler.Instance=CAN1;
	
	CAN1_Handler.Init = CAN1_InitConf;
	
    CAN1_Handler.Init.Prescaler=brp;				//分频系数(Fdiv)为brp+1
    CAN1_Handler.Init.Mode=mode;					//模式设置 
    CAN1_Handler.Init.SyncJumpWidth=tsjw;			//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ
    CAN1_Handler.Init.TimeSeg1=tbs1;				//tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ
    CAN1_Handler.Init.TimeSeg2=tbs2;				//tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ
    CAN1_Handler.Init.TimeTriggeredMode=DISABLE;	//非时间触发通信模式 
    CAN1_Handler.Init.AutoBusOff=DISABLE;			//软件自动离线管理
    CAN1_Handler.Init.AutoWakeUp=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN1_Handler.Init.AutoRetransmission=ENABLE;	//禁止报文自动传送 
    CAN1_Handler.Init.ReceiveFifoLocked=DISABLE;	//报文不锁定,新的覆盖旧的 
    CAN1_Handler.Init.TransmitFifoPriority=DISABLE;	//优先级由报文标识符决定 
	
    if(HAL_CAN_Init(&CAN1_Handler)!=HAL_OK)			//初始化
		return 1;
    return 0;
}

void CAN1_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 15;

  if (HAL_CAN_ConfigFilter(&CAN1_Handler, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    while(1)
	  {
	  }
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CAN1_Handler) != HAL_OK)
  {
    /* Start Error */
    while(1)
	  {
	  }
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&CAN1_Handler, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    while(1)
	  {
	  }
  }

  /*##-5- Configure Transmission process #####################################*/
  CAN1_TxHeader.StdId = 0x321;
  CAN1_TxHeader.ExtId = 0x01;
  CAN1_TxHeader.RTR = CAN_RTR_DATA;
  CAN1_TxHeader.IDE = CAN_ID_STD;
  CAN1_TxHeader.DLC = 2;
  CAN1_TxHeader.TransmitGlobalTime = DISABLE;
}

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(uint32_t ExtId, u8* msg,u8 len)
{	
    u8 i=0;
    u32 j = 0;
	u32 TxMailbox;
	u8 message[8];
    CAN1_TxHeader.StdId=0X13;        //标准标识符
    CAN1_TxHeader.ExtId=ExtId;        //扩展标识符(29位)
    CAN1_TxHeader.IDE=CAN_ID_EXT;    //使用标准帧
    CAN1_TxHeader.RTR=CAN_RTR_DATA;  //数据帧
    CAN1_TxHeader.DLC=len;                
    for(i=0;i<len;i++)
    {
		message[i]=msg[i];
	}
    if(HAL_CAN_AddTxMessage(&CAN1_Handler, &CAN1_TxHeader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
	while((HAL_CAN_GetTxMailboxesFreeLevel(&CAN1_Handler) != 3) && (++j < 0xfff)) 
    {
        
    }
    if(j >= 0xfff)
    {
        return 1;
    }
    return 0;
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{
 	u32 i;
	u8	RxData[8];

	if(HAL_CAN_GetRxFifoFillLevel(&CAN1_Handler, CAN_RX_FIFO0) != 1)
	{
		return 0xF1;
	}

	if(HAL_CAN_GetRxMessage(&CAN1_Handler, CAN_RX_FIFO0, &CAN1_RxHeader, RxData) != HAL_OK)
	{
		return 0xF2;
	}
    for(i=0;i<CAN1_RxHeader.DLC;i++)
    buf[i]=RxData[i];
	return CAN1_RxHeader.DLC;
}


CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1TQ~CAN_SJW_4TQ
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1TQ~CAN_BS2_8TQ;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1TQ~CAN_BS1_16TQ
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+tbs2+1)*brp); 其中tbs1和tbs2我们只用关注标识符上标志的序号，例如CAN_BS2_1TQ，我们就认为tbs2=1来计算即可。
//mode:CAN_MODE_NORMAL,普通模式;CAN_MODE_LOOPBACK,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_MODE_LOOPBACK);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 

u8 CAN2_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode)
{
	CAN_InitTypeDef		CAN2_InitConf;
    
    CAN2_Handler.Instance=CAN2;
	
	CAN2_Handler.Init = CAN2_InitConf;
	
    CAN2_Handler.Init.Prescaler=brp;				//分频系数(Fdiv)为brp+1
    CAN2_Handler.Init.Mode=mode;					//模式设置 
    CAN2_Handler.Init.SyncJumpWidth=tsjw;			//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1TQ~CAN_SJW_4TQ
    CAN2_Handler.Init.TimeSeg1=tbs1;				//tbs1范围CAN_BS1_1TQ~CAN_BS1_16TQ
    CAN2_Handler.Init.TimeSeg2=tbs2;				//tbs2范围CAN_BS2_1TQ~CAN_BS2_8TQ
    CAN2_Handler.Init.TimeTriggeredMode=DISABLE;	//非时间触发通信模式 
    CAN2_Handler.Init.AutoBusOff=DISABLE;			//软件自动离线管理
    CAN2_Handler.Init.AutoWakeUp=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN2_Handler.Init.AutoRetransmission=ENABLE;	//禁止报文自动传送 
    CAN2_Handler.Init.ReceiveFifoLocked=DISABLE;	//报文不锁定,新的覆盖旧的 
    CAN2_Handler.Init.TransmitFifoPriority=DISABLE;	//优先级由报文标识符决定 
	
    if(HAL_CAN_Init(&CAN2_Handler)!=HAL_OK)			//初始化
		return 1;
    return 0;
}

//CAN底层驱动，引脚配置，时钟配置，中断配置
//此函数会被HAL_CAN_Init()调用
//hcan:CAN句柄
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    if(hcan->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();                //使能CAN1时钟
        __HAL_RCC_CAN2_CLK_ENABLE();                //使能CAN1时钟
        __HAL_RCC_GPIOB_CLK_ENABLE();			    //开启GPIOA时钟
        
        GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;   //PA11,12
        GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
        GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
        GPIO_Initure.Speed=GPIO_SPEED_FAST;         //快速
        GPIO_Initure.Alternate=GPIO_AF9_CAN1;       //复用为CAN1++
        HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //初始化
    }
    
    else if(hcan->Instance == CAN2)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();                //使能CAN1时钟
        __HAL_RCC_CAN2_CLK_ENABLE();                //使能CAN1时钟
        __HAL_RCC_GPIOB_CLK_ENABLE();			    //开启GPIOA时钟
        
        GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13;   //PA11,12
        GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
        GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
        GPIO_Initure.Speed=GPIO_SPEED_FAST;         //快速
        GPIO_Initure.Alternate=GPIO_AF9_CAN2;       //复用为CAN1++
        HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //初始化
    }
}

void CAN2_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 15;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 15;

  if (HAL_CAN_ConfigFilter(&CAN2_Handler, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    while(1)
	  {
	  }
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(&CAN2_Handler) != HAL_OK)
  {
    /* Start Error */
    while(1)
	  {
	  }
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&CAN2_Handler, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    while(1)
	  {
	  }
  }

  /*##-5- Configure Transmission process #####################################*/
  CAN2_TxHeader.StdId = 0x321;
  CAN2_TxHeader.ExtId = 0x01;
  CAN2_TxHeader.RTR = CAN_RTR_DATA;
  CAN2_TxHeader.IDE = CAN_ID_STD;
  CAN2_TxHeader.DLC = 2;
  CAN2_TxHeader.TransmitGlobalTime = DISABLE;
}

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN2_Send_Msg(uint32_t ExtId, u8* msg,u8 len)
{	
    u8 i=0;
	u32 TxMailbox;
    u32 j = 0;
	u8 message[8];
    CAN2_TxHeader.StdId=0X13;        //标准标识符
    CAN2_TxHeader.ExtId=ExtId;        //扩展标识符(29位)
    CAN2_TxHeader.IDE=CAN_ID_EXT;    //使用标准帧
    CAN2_TxHeader.RTR=CAN_RTR_DATA;  //数据帧
    CAN2_TxHeader.DLC=len;                
    for(i=0;i<len;i++)
    {
		message[i]=msg[i];
	}
    if(HAL_CAN_AddTxMessage(&CAN2_Handler, &CAN2_TxHeader, message, &TxMailbox) != HAL_OK)//发送
	{
		return 1;
	}
    while((HAL_CAN_GetTxMailboxesFreeLevel(&CAN2_Handler) != 3) && (++j < 0xfff)) 
    {
        
    }
    if(j >= 0xfff)
    {
        return 1;
    }
    return 0;
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN2_Receive_Msg(u8 *buf)
{
 	u32 i;
	u8	RxData[8];

	if(HAL_CAN_GetRxFifoFillLevel(&CAN2_Handler, CAN_RX_FIFO0) != 1)
	{
		return 0xF1;
	}

	if(HAL_CAN_GetRxMessage(&CAN2_Handler, CAN_RX_FIFO0, &CAN2_RxHeader, RxData) != HAL_OK)
	{
		return 0xF2;
	}
    for(i=0;i<CAN2_RxHeader.DLC;i++)
    buf[i]=RxData[i];
	return CAN2_RxHeader.DLC;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

STM32双路CAN的应用有以下几点需要注意：
1，滤波器的初始化问题
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 15;C
