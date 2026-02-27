#ifndef _MOTION_QUEUE_H_
#define _MOTION_QUEUE_H_


#include "stdint.h"
#define MOTION_QUEUE_LEN 100


typedef struct _MOTION_FLOAT
{
	int position;
	uint32_t max_speed;
	uint32_t acc;
	uint32_t dec;
    unsigned short int  controlword;
}Motion,*PMotion;

typedef struct
{
	int front;
	int rear;
	int number;
	Motion motion[MOTION_QUEUE_LEN];
}MotionQueue;

int  is_motion_queue_empty(MotionQueue* p_queue);
void init_motion_queue(MotionQueue* p_queue);
int  insert_motion_to_queue(MotionQueue* p_queue, Motion* p_motion);  
int  get_motion_queue_front(MotionQueue* p_queue, Motion* p_motion);
int  get_motion_queue_rear(MotionQueue* p_queue, Motion* p_motion);
int  delete_motion_queue_front(MotionQueue* p_queue);

#endif /* _MOTION_QUEUE_H_ */
