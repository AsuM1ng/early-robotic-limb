/**
******************************************************************************
*@file    motiobn_queue.c 
*@author  zoucb
*@brief   motion队列操作模块，主要用于电机motion的接收和发送
******************************************************************************
**/


#include "motion_queue.h"
#include "stdio.h"
#include "string.h"

/**
 * @brief 判断队列是否为空
 * @retval  
 * -0 不为空
 * -1 空
 */
int is_motion_queue_empty(MotionQueue* p_queue)
{
	return (p_queue->front == p_queue->rear);
}
/**
 * @brief 初始化队列
 */
void init_motion_queue(MotionQueue* p_queue)
{
	p_queue->number = 0;
	p_queue->front = 0;
	p_queue->rear = 0;
}

/**
 * @brief 把元素插入队列尾部
 * @retval  
 * -0 失败，队列满 
 * -1成功
 */
int  insert_motion_to_queue(MotionQueue* p_queue, Motion* p_motion)
{
	int tmp = 0;
	/* 队列已经满了 */
	if((MOTION_QUEUE_LEN-1) == p_queue->number)
	{	
		return 0;//队列插入失败
	}
	else
	{
		p_queue->number++;
		memcpy(p_queue->motion + p_queue->rear, p_motion, sizeof(Motion));
		//确保原子操作
		tmp = p_queue->rear + 1;
		p_queue->rear = tmp%MOTION_QUEUE_LEN;
		return 1;
	}
}

/**
 * @brief  获取队列的首元素
 * @retval  
 * -0 失败，队列为空  
 * -1成功
 */
int  get_motion_queue_front(MotionQueue* p_queue,Motion* p_motion)
{
	/*如果队列为空*/
	if (is_motion_queue_empty(p_queue) || (p_motion == NULL))
	{
		return 0;	
	}
	else
	{
		memcpy(p_motion, p_queue->motion + p_queue->front, sizeof(Motion));
		return 1;
	}
}

/**
 * @brief  获取队列的尾元素
 * @retval  
 * -0 失败，队列为空  
 * -1成功
 */
int  get_motion_queue_rear(MotionQueue* p_queue,Motion* p_motion)
{
	/*不管队列是否为空都取队尾的数据*/
	memcpy(p_motion, p_queue->motion + p_queue->front + p_queue->rear, sizeof(Motion));
	return 1;
}

/**
 * @brief  删除队列队首元素
 * @retval  
 * -0 失败，队列为空  
 * -1成功
 */
int delete_motion_queue_front(MotionQueue* p_queue)
{
	int tmp = 0;
	if(is_motion_queue_empty(p_queue))
	{
		return 0;
	}
	else
	{
		p_queue->number--;
		tmp = p_queue->front + 1;
		p_queue->front= tmp%MOTION_QUEUE_LEN;		
		return 1;
	}
}

