#ifndef __CSP_TARGET_POSITION_SEND_H__
#define __CSP_TARGET_POSITION_SEND_H__

#include "stdint.h"

extern volatile uint8_t g_send_position_flag;
extern volatile uint8_t g_send_heartbeat_flag;
extern volatile uint8_t g_stop_heartbeat_flag;

void syncSend(void);
void cspTargetPositionSend(volatile int target_position);

void canFrameSendCtrl(void);
#endif


