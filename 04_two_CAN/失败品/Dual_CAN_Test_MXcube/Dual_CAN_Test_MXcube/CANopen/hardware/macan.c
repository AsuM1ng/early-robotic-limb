//mycan.c
#include "mycan.h"
#include "can_1.h"
#include "canfestival_master.h"
 
extern TIM_HandleTypeDef htim1;

uint8_t ubKeyNumber = 0x0;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;	

uint8_t canfestival_can_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{ 
  CAN_FilterTypeDef  sFilterConfig;

  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  
  /* Configure Transmission process */
	
	return 0;
}

/**
  * @brief  Rx Fifo 0 message pending callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
//  unsigned int i = 0;
	Message RxMSG ;
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxMSG.data);
  RxMSG.cob_id = (uint16_t)(RxHeader.StdId);
  if( RxHeader.RTR == CAN_RTR_REMOTE )
  {
     RxMSG.rtr = 1;    
  }
  else
  {
      RxMSG.rtr = 0; 
  }  
	RxMSG.len = RxHeader.DLC;
	canDispatch(&Master_Data, &(RxMSG)); 
	
}

uint8_t canSend(CAN_PORT notused, Message *message)
{
	CanTxMsg TxMessage;
	uint32_t TxMailbox = 0;

	/* ×é×°CANÊý¾Ý°ü */
	TxHeader.DLC = message->len;							
	memcpy(TxMessage.Data, message->data, message->len);	
	TxHeader.IDE = CAN_ID_STD;								
	TxHeader.StdId = message->cob_id;				
	TxHeader.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;	

	while(( HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxMessage.Data,&TxMailbox)!=HAL_OK))
	{
			return 0;
	}
	return 1;
}

void setTimer(TIMEVAL value)
{
	TIM1->ARR = TIM1->CNT + value;
}

TIMEVAL getElapsedTime(void)
{
	return TIM1->CNT;
}


void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	TimeDispatch();
  /* USER CODE END TIM1_UP_IRQn 1 */
}


