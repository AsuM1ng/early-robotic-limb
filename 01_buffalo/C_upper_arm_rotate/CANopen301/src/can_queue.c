#include "can_queue.h"
#include "stdint.h"
#include "can.h"

//#define MAX_MAIL_NUM  3
//static uint8_t CAN_msg_num[MAX_MAIL_NUM];   // Tx mailbox flag

_CANxTxQueue CAN1TxQueue;		// define the CAN1 TX Queue
_CANxTxQueue CAN2TxQueue;		// define the CAN2 TX Queue


/**
  * @brief  
  * @param  _CANxTxQueue
  * @retval None
  */  
void ClearCanTxQueue(_CANxTxQueue* Queue)
{
    int i;
    
    for(i = 0; i < MAX_CAN_SIZE; i++)
    {
        memset(&Queue->data, 0, sizeof(CanTxMsg)); 
    }

    Queue->front = 0; 
    Queue->rear = 0;
}


uint8_t isEmptyCanTxQueue(_CANxTxQueue* Queue)
{
    if(Queue->front == Queue->rear)          
    {
        return 1;
    }    
    else
    {
        return 0;    
    }
}


uint8_t isFullCanTxQueue(_CANxTxQueue* Queue)
{
    if( Queue->front == (Queue->rear + 1) % MAX_CAN_SIZE)
    {
        return 1;
    }    
    else
    {
        return 0;    
    }
}


uint8_t insertCanTxQueue(_CANxTxQueue* Queue, CanTxMsg* element)
{
    if(!isFullCanTxQueue(Queue)) 
    {
        memcpy(&Queue->data[Queue->rear], element, sizeof(CanTxMsg));
        Queue->rear = (Queue->rear + 1) % MAX_CAN_SIZE;
        return 0;
    }
    else 
    {
        return 0xFF;
    }
}


uint8_t popCanTxQueue(_CANxTxQueue* Queue, CanTxMsg* element)
{
	if(isEmptyCanTxQueue(Queue))
		return 0xFF;
	
	*element = Queue->data[Queue->front];
	Queue->front = (Queue->front + 1) % MAX_CAN_SIZE;
	
	return 0;	
}


uint8_t canSendOrAndQueue(Message *m, uint8_t process_method)
{
	uint32_t  i;
	CanTxMsg TxMessage;
	CanTxMsg *ptx_msg=&TxMessage;
	
	ptx_msg->StdId = m->cob_id;
	
	if(m->rtr)
	{	 
		ptx_msg->RTR = CAN_RTR_REMOTE;
	}
	else
	{
		 ptx_msg->RTR = CAN_RTR_DATA;
	}
		
	ptx_msg->IDE = CAN_ID_STD;
	ptx_msg->DLC = m->len;
	
	CAN_ITConfig(CAN2, CAN_IT_TME, DISABLE);   	  	  // turn off TME interupt, to lock the canTxQueue	
	// copy message
	for(i = 0; i < m->len; i++)
	{
		 ptx_msg->Data[i] = m->data[i];
	}	
	
	// transmit or add message into queue
	if(TRY_SEND_FIRST == process_method)
	{
		if( CAN_Transmit(CAN2, ptx_msg)==CAN_NO_MB ) 
		{
			if(!isFullCanTxQueue(&CAN2TxQueue))
			{
				insertCanTxQueue(&CAN2TxQueue, ptx_msg);
				CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);   // turn on TME interupt
				return 0x01;		/* add queue success */
			}
			else
			{
				return 0xff;	/* failed */				
			}

		}
		else
		{
			if(!isEmptyCanTxQueue(&CAN2TxQueue))			
				CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);   // turn on TME interupt
			return 0x00;		/* send success */
		}	
	}
	else if(ADD_TO_QUEUE_DIRECTLY == process_method)
	{
		if(!isFullCanTxQueue(&CAN2TxQueue))
		{
			insertCanTxQueue(&CAN2TxQueue, ptx_msg);
			CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);   // turn on TME interupt
			return 0x01;		/* add queue success */
		}
		else
		{			
			return 0xff;	/* failed */
		}
	}
}


//uint8_t canSend(CO_Data* d, Message *m)
//{
//	uint32_t  i;
//    
//	CanTxMsg TxMessage;
//	CanTxMsg *ptx_msg=&TxMessage;
//	ptx_msg->StdId = m->cob_id;
//    
//    d->send_num=1;

//		if(m->rtr)
//		{	 
//			ptx_msg->RTR = CAN_RTR_REMOTE;
//		}
//		else
//		{
//			 ptx_msg->RTR = CAN_RTR_DATA;
//		}
//		
//		ptx_msg->IDE = CAN_ID_STD;
//		ptx_msg->DLC = m->len;
//		
//		for(i = 0; i < m->len; i++)
//		{
//			 ptx_msg->Data[i] = m->data[i];
//		}
//		
//    if( CAN_Transmit( d->canHandle, ptx_msg )==CAN_NO_MB ) /* 如果没有空闲的发送邮箱可以使用，则返回0xff,发送成功则返回0x00 */
//    {
//		if(!isCanTxQueueFull())
//		{
//			insertCanTxQueue();
//			CAN_ITConfig(d->canHandle, CAN_IT_TME, ENABLE);   // 打开发送中断				
//		}
//		else
//			return 0xff;/* 失败 */	
//		
////		if(d->can_port_num == 1)
////		{
////		    slave_send_faild++;
////		}
////		
////		if(d->can_port_num == 2)
////		{
////		    master_send_faild++;
////		}

//    }
//    else
//    {
//        d->send_num=0;
//        return 0x00;/* 成功 */
//    }
//}


//void SetHeadCanQueue(u16 head)
//{
//    if(CANQueue.front != CANQueue.rear)
//    {
//        CANQueue.front = head;
//    }   
//}