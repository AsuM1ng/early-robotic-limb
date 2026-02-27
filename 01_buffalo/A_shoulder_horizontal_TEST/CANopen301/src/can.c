#include "states.h"
#include "TestSlave.h"
#include "CANopenMaster.h"
#include "can.h"
#include "canfestival.h"
#include "chipHAL_CAN.h"
#include "can_queue.h"

uint8_t initCan(int can_port_num, uint32_t bitrate)
{
	if(1 == can_port_num)
	{
        CAN1_Config(bitrate);
	}
	else if(2 == can_port_num)
	{
        CAN2_Config(bitrate);
	}

	return 0;
}


// process the CAN1 Received msg
//uint32_t count_receive=0;
//int cobID_error = 0;
void CAN1_RX0_IRQHandler(void)
{
    CAN_ITConfig(NODE_CAN1,CAN_IT_FMP0, DISABLE);//防止中断服务函数还没执行完又有数据传过来了
//    count_receive++;
//    if(count_receive == 100)
//    {
//        count_receive = 0;
//    }
	CanRxMsg RxMessage;
	Message m;
	int i;
	CAN_Receive(NODE_CAN1, CAN_FIFO0, &RxMessage);
	m.cob_id=RxMessage.StdId;
//    if(RxMessage.StdId == 0x682)
//    {
//        cobID_error++;
//    }
    
	if(RxMessage.RTR == CAN_RTR_REMOTE)
		 m.rtr=1;
	else if(RxMessage.RTR == CAN_RTR_DATA)
		 m.rtr=0;
		 m.len=RxMessage.DLC;
	for(i = 0; i < RxMessage.DLC; i++)
		 m.data[i]=RxMessage.Data[i];
	canDispatch(&TestSlave_Data, &m);

    CAN_ITConfig(NODE_CAN1,CAN_IT_FMP0, ENABLE); 
}


// process the CAN2 Received msg
//uint32_t test_CAN2_rcv_count=0;
void CAN2_RX0_IRQHandler(void)
{
    CAN_ITConfig(NODE_CAN2,CAN_IT_FMP0, DISABLE);//防止中断服务函数还没执行完又有数据传过来了

//    test_CAN2_rcv_count++;
//    if(test_CAN2_rcv_count == 1000)
//    {
//        test_CAN2_rcv_count = 0;
//    }	
	
	CanRxMsg RxMessage;
	Message m;
	int i;
	CAN_Receive(NODE_CAN2, CAN_FIFO0, &RxMessage);
	m.cob_id=RxMessage.StdId;

	if(RxMessage.RTR == CAN_RTR_REMOTE)
		 m.rtr=1;
	else if(RxMessage.RTR == CAN_RTR_DATA)
	{
		 m.rtr=0;
		 m.len=RxMessage.DLC;
	}
		if( m.cob_id>>7==0xB)	//快速SDO回应ID为0x580+对方id, 右移7位即为0xB 
	{
		int16_t test=0;
		switch(m.cob_id)
		{
			case 0x581:
				for(i = 0; i < 4; i++)
				test=m.data[4]|m.data[5]<<8;
//				printf("test=%d,\r\n",test);	//其实在中断不应该使用打印函数的，这里是方便调试而已
				break;
			default:
				break;
		}
	
	}
	else
		canDispatch(&CANopenMaster_Data, &m);
	
	for(i = 0; i < RxMessage.DLC; i++)
		 m.data[i]=RxMessage.Data[i];
	canDispatch(&CANopenMaster_Data, &m);

	
	
    CAN_ITConfig(NODE_CAN2,CAN_IT_FMP0, ENABLE); 
}


/***************************************************************************************/
//uint32_t  send_num = 0;
//int master_send_faild = 0;
//int slave_send_faild = 0;
uint8_t canSend(CO_Data* d, Message *m)
{
	uint32_t  i;
	uint8_t process_method = TRY_SEND_FIRST;
	
	CanTxMsg TxMessage;
	CanTxMsg *ptx_msg=&TxMessage;
	_CANxTxQueue *ptx_queue;
	
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
	
	if(1 == d->can_port_num)
		ptx_queue = &CAN1TxQueue;
	else if(2 == d->can_port_num)
		ptx_queue = &CAN2TxQueue;

	CAN_ITConfig(d->canHandle, CAN_IT_TME, DISABLE);   	  	  // turn off TME interupt, to lock the canTxQueue	
	// copy message
	for(i = 0; i < m->len; i++)
	{
		 ptx_msg->Data[i] = m->data[i];
	}
		
	// transmit or add message into queue
	if(TRY_SEND_FIRST == process_method)
	{
		if( CAN_Transmit(d->canHandle, ptx_msg)==CAN_NO_MB ) 
		{
			if(!isFullCanTxQueue(ptx_queue))
			{
				insertCanTxQueue(ptx_queue, ptx_msg);
				CAN_ITConfig(d->canHandle, CAN_IT_TME, ENABLE);   // turn on TME interupt
				return 0x01;		/* add queue success */
			}
			else
			{
				return 0xff;	/* failed */
			}
		}
		else
		{
			if(!isEmptyCanTxQueue(ptx_queue))
				CAN_ITConfig(d->canHandle, CAN_IT_TME, ENABLE);   // turn on TME interupt
			
			return 0x00;		/* send success */
		}	
	}
}


int error_CAN1_transmit = 0;
//
void CAN1_TX_IRQHandler(void)
{
	CanTxMsg TxMessage;
	CanTxMsg *ptx_msg=&TxMessage;
	
	if(SET == CAN_GetITStatus(CAN1, CAN_IT_TME))
	{		
		if(!isEmptyCanTxQueue(&CAN1TxQueue))
		{
			popCanTxQueue(&CAN1TxQueue, ptx_msg);
			if( CAN_Transmit(CAN1, ptx_msg)==CAN_NO_MB ) 
			{
				++error_CAN1_transmit;
			}
		}
		else
		{
			CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE); 
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
	}	
}


int error_CAN2_transmit = 0;
//
void CAN2_TX_IRQHandler(void)
{
	CanTxMsg TxMessage;
	CanTxMsg *ptx_msg=&TxMessage;
	
	if(SET == CAN_GetITStatus(NODE_CAN2, CAN_IT_TME))
	{		
		if(!isEmptyCanTxQueue(&CAN2TxQueue))
		{
			popCanTxQueue(&CAN2TxQueue, ptx_msg);
			if( CAN_Transmit(NODE_CAN2, ptx_msg)==CAN_NO_MB ) 
			{
				++error_CAN2_transmit;
			}
		}
		else
		{
			CAN_ITConfig(NODE_CAN2, CAN_IT_TME, DISABLE); 
		}
		CAN_ClearITPendingBit(NODE_CAN2, CAN_IT_TME);
	}	
}


//int can_count_success=0;
//int can_count_failed=0;
//unsigned char canSend_alarm(CO_Data* d, UNS32 id)
//{
//			uint32_t        i;
//			Message m;
//			m.cob_id = 0x01;
//			m.data[0] = 0x00;
//			m.data[1] = 0x01;
//			m.data[2] = 0x02;
//			m.data[3] = 0x03;
//			m.data[4] = 0x04;
//			m.data[5] = 0x05;
//			m.data[6] = 0x06;
//			m.data[7] = 0x07;
//			m.rtr = 0;
//			m.len = 8;

//			CanTxMsg TxMessage;
//			CanTxMsg *ptx_msg=&TxMessage;
//			ptx_msg->StdId = m.cob_id;

//			if(m.rtr)
//				 ptx_msg->RTR = CAN_RTR_REMOTE;
//			else
//				 ptx_msg->RTR = CAN_RTR_DATA;

//				 ptx_msg->IDE = CAN_ID_STD;
//				 ptx_msg->DLC = m.len;
//			for(i = 0; i < m.len; i++)
//				 ptx_msg->Data[i] = m.data[i];
//		if( CAN_Transmit( d->canHandle, ptx_msg )==CAN_NO_MB ) /* Flaxin 如果没有空闲的发送邮箱可以使用，则返回0xff,发送成功则返回0x00 */
//		{
//			can_count_failed++;
//			if(can_count_failed==10)
//				can_count_failed=0;
//			return 0xff;/* 失败 */
//		}
//		else
//		{
//			can_count_success++;
//			if(can_count_success==10)
//				can_count_success=0;
//			return 0x00;/* 成功 */
//		}	
//}
