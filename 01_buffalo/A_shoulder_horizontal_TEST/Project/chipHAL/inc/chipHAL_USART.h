#ifndef __CHAL_USART_H__
#define __CHAL_USART_H__

#include <stdint.h>

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART3_RX            0

extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	

void chipHAL_UsartInit(uint32_t baudrate);
 
#endif
