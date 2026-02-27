#ifndef CAN_QUEUE_H_
#define CAN_QUEUE_H_

#include "stm32f4xx_can.h"
#include "can.h"

#define MAX_CAN_SIZE 64
#define TRY_SEND_FIRST 1
#define ADD_TO_QUEUE_DIRECTLY 2

typedef struct CanTxQueue
{
    CanTxMsg data[MAX_CAN_SIZE];
    uint16_t front;
    uint16_t rear;
}_CANxTxQueue;

extern _CANxTxQueue CAN1TxQueue;
extern _CANxTxQueue CAN2TxQueue;

void ClearCanTxQueue(_CANxTxQueue* Queue);
uint8_t isEmptyCanTxQueue(_CANxTxQueue* Queue);
uint8_t isFullCanTxQueue(_CANxTxQueue* Queue);
uint8_t insertCanTxQueue(_CANxTxQueue* Queue, CanTxMsg* element);
uint8_t popCanTxQueue(_CANxTxQueue* Queue, CanTxMsg* element);
uint8_t canSendOrAndQueue(Message *m, uint8_t process_method);

#endif