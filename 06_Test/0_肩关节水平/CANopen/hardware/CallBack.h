#ifndef __TIMER3_H
#define __TIMER3_H

#include "sys.h"
#include "main.h"

void Can1RxCallback(void (*callback)(Message*, uint8_t*));
void Can2RxCallback(void (*callback)(Message*, uint8_t*));



#endif 

