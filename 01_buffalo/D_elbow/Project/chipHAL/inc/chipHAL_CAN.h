#ifndef CHIP_HAL_CAN_H__
#define CHIP_HAL_CAN_H__

#include "stdint.h"
#include "Board_type_config.h"

#define NODE_CAN1      CAN1
#define NODE_CAN2      CAN2


int CAN1_Config(uint32_t bitrate);
int CAN2_Config(uint32_t bitrate);	

#endif
