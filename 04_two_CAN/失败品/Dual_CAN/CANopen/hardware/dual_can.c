// Includes for the Canfestival driver
#include "dual_can.h"


static CO_Data *co_data_1 = NULL;
static CO_Data *co_data_2 = NULL;


//Initialize the CAN hardware 
unsigned char CAN1_Init(CO_Data * d1, uint32_t bitrate)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  
  /* save the canfestival handle */  
  co_data_1 = d1;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF7 */
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, GPIO_AF_CAN1);
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, GPIO_AF_CAN1); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN | CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	
    
  /* CAN Baudrate (CAN clocked at 42 MHz)  42e6 / ( 6 * (1+BS1+BS2))  */
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  if(bitrate == 1000000){
  	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  }
  else {	//除去1M频率。剩下都配置为500K
  	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  	CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
  }

  CAN_InitStructure.CAN_Prescaler = 6;
  CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  return 1;
}

// The driver send a CAN message passed from the CANopen stack


unsigned char CAN2_Init(CO_Data * d2, uint32_t bitrate)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  
  /* save the canfestival handle */  
  co_data_2 = d2;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
//	RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(CAN2_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF7 */
  GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_RX_SOURCE, GPIO_AF_CAN2);
  GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_TX_SOURCE, GPIO_AF_CAN2);
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN | CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN2_GPIO_PORT, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
	RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);	
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate (CAN clocked at 42 MHz)  42e6 / ( 6 * (1+BS1+BS2))  */
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
  if(bitrate == 1000000){
  	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  }
  else {	//除去1M频率。剩下都配置为500K
  	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  	CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
  }

  CAN_InitStructure.CAN_Prescaler = 6;
  CAN_Init(CAN2, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 15;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

  return 1;
}





unsigned char canSend(CAN_PORT notused, Message *m)
{
	int i, res;
	CanTxMsg TxMessage = {0};
	TxMessage.StdId = m->cob_id;
	TxMessage.IDE = CAN_ID_STD;
	if(m->rtr)
  		TxMessage.RTR = CAN_RTR_REMOTE;
	else
  		TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = m->len;
	for(i=0 ; i<m->len ; i++)
		TxMessage.Data[i] = m->data[i]; 
    res = CAN_Transmit(CAN1, &TxMessage);
	if(res == CAN_TxStatus_NoMailBox)
		return 0; 	// error
    return 1;		// succesful
}

//The driver pass a received CAN message to the stack
/*
unsigned char canReceive(Message *m)
{
}
*/
unsigned char canChangeBaudRate_driver( CAN_HANDLE fd, char* baud)
{
	return 0;
}

/**
  * @brief  This function handles CAN1 RX0 interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	int i;
	int position;
	unsigned char actual_position[8]={0x23,0x7A,0x60,0x00};
	CanRxMsg RxMessage = {0};
	Message rxm = {0};
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	// Drop extended frames
	if(RxMessage.IDE == CAN_ID_EXT) //不处理扩展帧
		return;
	rxm.cob_id = RxMessage.StdId;
  	if(RxMessage.RTR == CAN_RTR_REMOTE)//远程帧
		rxm.rtr = 1;
	rxm.len = RxMessage.DLC;
	for(i=0 ; i<rxm.len ; i++)
  		 rxm.data[i] = RxMessage.Data[i];
		
		/*****这里是自己的数据接收部分*******/
//	if( rxm.cob_id==0x581)	
//	{
//		if(rxm.data[0] == 0x43 && rxm.data[1] == 0x64 && rxm.data[2] == 0x60)
//		{
//			
//			for(i = 0; i < 4; i++)
//				actual_position[i+4] = rxm.
//			
//		}
//	else
		canDispatch(co_data_1, &rxm);//CANopen自身的处理函数，因为快速SDO不需要反馈，所以在上边处理后就不需要调用这步了
	
}



void CAN2_RX0_IRQHandler(void)
{
	int i;
	int position;
	unsigned char actual_position[8]={0x23,0x7A,0x60,0x00};
	CanRxMsg RxMessage = {0};
	Message rxm = {0};
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
	// Drop extended frames
	if(RxMessage.IDE == CAN_ID_EXT) //不处理扩展帧
		return;
	rxm.cob_id = RxMessage.StdId;
  	if(RxMessage.RTR == CAN_RTR_REMOTE)//远程帧
		rxm.rtr = 1;
	rxm.len = RxMessage.DLC;
	for(i=0 ; i<rxm.len ; i++)
  		 rxm.data[i] = RxMessage.Data[i];
		
		/*****这里是自己的数据接收部分*******/
//	if( rxm.cob_id==0x581)	
//	{
//		if(rxm.data[0] == 0x43 && rxm.data[1] == 0x64 && rxm.data[2] == 0x60)
//		{
//			
//			for(i = 0; i < 4; i++)
//				actual_position[i+4] = rxm.
//			
//		}
//	else
		canDispatch(co_data_2, &rxm);//CANopen自身的处理函数，因为快速SDO不需要反馈，所以在上边处理后就不需要调用这步了
	
}

