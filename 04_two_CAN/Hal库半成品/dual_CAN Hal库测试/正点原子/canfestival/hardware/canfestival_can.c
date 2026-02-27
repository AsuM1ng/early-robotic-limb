#include "stm32f4xx.h"
#include "canfestival_can.h"
#include "canfestival.h"
#include "stm32f4xx_hal_gpio.h"
#include "can.h"



/* CANOPEN字典 */
extern CO_Data masterObjdict_Data;

/* 功能:	GPIO配置
	 参数:	无
	 返回值:无
 */
static void gpio_config(void)
{
	//CAN1的GPIO初始化
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOA_CLK_ENABLE();//使能PORTA时钟

	GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_Initure.Mode = GPIO_MODE_AF_PP;       //复用功能
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;       //推挽输出
  GPIO_Initure.Speed = GPIO_SPEED_HIGH;    //100MHz  GPIO_Speed_100MHz
  GPIO_Initure.Pull = GPIO_PULLUP;         //上拉
  GPIO_Init(GPIOD, &GPIO_Initure);               //初始化
   GPIO_InitTypeDef GPIO_InitStructure;

   /* 使能引脚时钟 */
   CAN_TX_GPIO_CLK_ENABLE();
   CAN_RX_GPIO_CLK_ENABLE();

   /* 配置CAN发送引脚 */
   GPIO_InitStructure.Pin = CAN1_TX_GPIO_PIN;
   GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
   GPIO_InitStructure.Pull  = GPIO_PULLUP;
   GPIO_InitStructure.Alternate =  GPIO_AF9_CAN1;
   HAL_GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);

   /* 配置CAN接收引脚 */
   GPIO_InitStructure.Pin = CAN1_RX_PIN ;
   HAL_GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
}

/* 功能:	中断嵌套控制器配置
	 参数:	无
	 返回值:无
 */
static void nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 设置CAN接收中断，先占优先级2，从站优先级2 */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/* 初始化中断 */
	NVIC_Init(&NVIC_InitStructure);
}

/* 功能:	CAN总线过滤器配置
	 参数:	无
	 返回值:无
 */

HAL_StatusTypeDef CAN1_Filter_Init(CAN_HandleTypeDef *h_can) 
{
  CAN_FilterTypeDef sFilterConfig;
 
  sFilterConfig.FilterBank = 2;   //chenal 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //标识符屏蔽位模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  //过滤器位宽为单个32位
  sFilterConfig.FilterIdHigh = 0xB000;  //标识符寄存器   
  sFilterConfig.FilterIdLow = 0xC400;   //标识符寄存器   
  //MASK bit 0 means don't care,bit 0 means match 
  sFilterConfig.FilterMaskIdHigh = 0xFE1F;   //屏蔽寄存器  //只存在于标识符屏蔽位模式中，在标识符列表模式中为标识符寄存器 
  sFilterConfig.FilterMaskIdLow = 0xFE1F;    //屏蔽寄存器                                 
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //FIFO0的中断和FIFO1的中断是不一样的，这里是把接收到的报文放入到FIFO0中
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;   //enable filter
  sFilterConfig.SlaveStartFilterBank = 0;    //为从属can选择开始的过滤库，对于单个CAN实例，这个参数没有意义
  if (HAL_CAN_ConfigFilter(h_can, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
    return HAL_ERROR;
  }
  //regist RX_IT
  if (HAL_CAN_ActivateNotification(h_can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //注册CAN_IT_RX_FIFO0_MSG_PENDING 对应的回调函数原型
  {
    Error_Handler();
    return HAL_ERROR;
  }
 return HAL_OK;
}






static void can_filter_config(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	/* 配置过滤器0组，配置成标准标识符且低7位都为0时接受 */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;						/* 设置过滤器组0 */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		/* 屏蔽模式 */
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	/* 32位模式 */
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;					/* 在CANOpen中标准标识符的低7位表示节点ID */
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;					/* 在CANOpen中只用标准标识符，数据帧/远程帧都有 */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;						/* 主节点ID为0 */
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;						/* 标准帧 */
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;	/* 过滤器关联到FIFO0 */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;					/* 使能过滤器 */
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/* 功能:	can总线配置
	 参数:	无
	 返回值:无
 */
void CANOpen_can_config(void)
{
	CAN_InitTypeDef CAN_InitStructure;

	/* 配置IO */
	gpio_config();

	/* 中断嵌套控制器配置 */
	nvic_config();

	/* 配置CAN总线时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN1默认参数 */
	CAN_DeInit(CAN1);

	/* 将结构体填入默认参数 */
	CAN_StructInit(&CAN_InitStructure);

	/* 关闭时间触发模式 */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	/* 关闭自动离线管理 */
	CAN_InitStructure.CAN_ABOM = ENABLE;
	/* 关闭自动唤醒 */
	CAN_InitStructure.CAN_AWUM = ENABLE;
	/* 自动重传 */
	CAN_InitStructure.CAN_NART = DISABLE;
	/* 禁止FIFO溢出时覆盖原报文 */
	CAN_InitStructure.CAN_RFLM = DISABLE;
	/* 关闭优先级取决于ID */
	CAN_InitStructure.CAN_TXFP = DISABLE;
	/* 正常模式 */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* 设置波特率:36MHz/9/(2+1+1)=1mbps */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 9;

	/* 初始化CAN总线 */
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN总线过滤器配置 */
	can_filter_config();

	/* 接收挂起中断 */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* can总线接收中断回调函数 */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg message;
	Message Rx_Message;

	/* 接收消息 */
	CAN_Receive(CAN1, CAN_FIFO0, &message);

	/* 组装canopen数据包 */
	Rx_Message.cob_id = message.StdId;						/* 功能码和节点ID */
	Rx_Message.rtr = (message.RTR == CAN_RTR_DATA) ? 0 : 1;	/* 标识符类型 */
	Rx_Message.len = message.DLC;							/* 数据包长度 */
	memcpy(Rx_Message.data, message.Data, message.DLC);		/* 数据 */

	/* canopen数据包分配处理函数 */
	canDispatch(&masterObjdict_Data, &Rx_Message);
}

/* 功能:	CAN发送数据函数
	 参数:	notused can总线端口
			message canopen数据包
	返回值:	0 成功
			1 失败
 */
uint8_t canSend(CAN_PORT notused, Message *message)
{
	uint32_t i = 0xFFFFFF;
	CanTxMsg TxMessage;
	uint8_t TransmitMailbox = 0;

	/* 组装CAN数据包 */
	TxMessage.DLC = message->len;							/* 数据长度 */
	memcpy(TxMessage.Data, message->data, message->len);	/* 数据 */
	TxMessage.IDE = CAN_ID_STD;								/* 标准ID */
	TxMessage.StdId = message->cob_id;						/* 标识符 */
	TxMessage.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;	/* 数据帧 */

	/* 发送数据包 */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	/* 等待发送成功 */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && --i);

	/* 成功0 超时1 */
	return (i != 0) ? 0 : 1;
}