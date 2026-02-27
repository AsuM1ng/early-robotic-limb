#include "can_data_send.h"
#include "can_queue.h"
#include "canfestival.h"
#include "TestSlave.h"
#include "global.h"
#include "Board_type_config.h"

volatile uint8_t g_send_position_flag = 0;
volatile uint8_t g_send_heartbeat_flag = 0;
volatile uint8_t g_stop_heartbeat_flag = 0;

void syncSend(void)
{

	Message can_frame;
	can_frame.cob_id = 0x80;
	can_frame.rtr = 0;
	can_frame.len = 0;	
	canSendOrAndQueue(&can_frame, TRY_SEND_FIRST);
}


void cspTargetPositionSend(volatile int target_position)
{
	Message can_frame;
	can_frame.cob_id = 0x180 + SELF_NODE_ID;
	can_frame.rtr = 0;
	can_frame.len = 4;
    can_frame.data[0] = (uint8_t)(target_position & 0xFF);
    can_frame.data[1] = (uint8_t)((target_position >> 8) & 0xFF);
    can_frame.data[2] = (uint8_t)((target_position >> 16) & 0xFF);	
    can_frame.data[3] = (uint8_t)((target_position >> 24) & 0xFF);
	canSendOrAndQueue(&can_frame, ADD_TO_QUEUE_DIRECTLY);
}
	

void positionFeedbackSend(uint8_t node_id, const int abs_encoder_angle, const int motor_encoder_angle)
{
	Message can_frame;
	can_frame.cob_id = 0x180 + node_id;
	can_frame.rtr = 0;
	can_frame.len = 8;
    can_frame.data[0] = (uint8_t)(abs_encoder_angle & 0xFF);
    can_frame.data[1] = (uint8_t)((abs_encoder_angle >> 8) & 0xFF);
    can_frame.data[2] = (uint8_t)((abs_encoder_angle >> 16) & 0xFF);	
    can_frame.data[3] = (uint8_t)((abs_encoder_angle >> 24) & 0xFF);	
    can_frame.data[4] = (uint8_t)(motor_encoder_angle & 0xFF);
    can_frame.data[5] = (uint8_t)((motor_encoder_angle >> 8) & 0xFF);
    can_frame.data[6] = (uint8_t)((motor_encoder_angle >> 16) & 0xFF);	
    can_frame.data[7] = (uint8_t)((motor_encoder_angle >> 24) & 0xFF);	
	canSend(&TestSlave_Data, &can_frame);
}


void motorStatuswordSend(uint8_t node_id, const uint16_t m_statusword)
{
	Message can_frame;
	can_frame.cob_id = 0x280 + node_id;
	can_frame.rtr = 0;
	can_frame.len = 2;
    can_frame.data[0] = (uint8_t)(m_statusword & 0xFF);
    can_frame.data[1] = (uint8_t)((m_statusword >> 8) & 0xFF);
	canSend(&TestSlave_Data, &can_frame);
}


void heartbeatSend(uint8_t node_id)
{
	Message can_frame;
	can_frame.cob_id = 0x700 + node_id;
	can_frame.rtr = 0;
	can_frame.len = 1;	
	can_frame.data[0] = 0x05;
	canSend(&TestSlave_Data, &can_frame);	
}
