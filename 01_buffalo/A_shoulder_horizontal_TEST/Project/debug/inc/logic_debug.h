

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOGIC_DEBUG_H_
#define __LOGIC_DEBUG_H_


#define LOGIC_DEBUG_ON  0

// K represents key, S represents switch
#define KEY_SWITCH_DEBUG 'S'


/* Includes ------------------------------------------------------------------*/

#include "data.h"

int CANopenSlaveODdataInit(CO_Data* d);
int CANopenMasterODdataInit(CO_Data* d);

#endif 

