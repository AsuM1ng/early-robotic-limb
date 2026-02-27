#include "logic_debug.h"
#include "chipHAL_GPIO.h"
#include "chipHAL_buzzer.h"
#include "boardHAL.h"
#include "TestSlave.h"
#include "CANopenMaster.h"


#ifdef LOGIC_DEBUG_ON


int CANopenSlaveODdataInit(CO_Data* d)
{
    setODentry(d, 0x2F19, 0x00, &clear_fault, &sizeUNS16, OD_ACCESS_RW);
    setODentry(d, 0x2F30, 0x00, &led_status, &sizeUNS16, OD_ACCESS_R);
    setODentry(d, 0x2F31, 0x00, &buzzer_status, &sizeUNS16, OD_ACCESS_R);
    setODentry(d, 0x2F32, 0x00, &ADC_status, &sizeUNS16, OD_ACCESS_R);
    setODentry(d, 0x2F33, 0x00, &jointboard_status, &sizeUNS16, OD_ACCESS_RW);
    setODentry(d, 0x2F34, 0x00, &toggle_switch_status, &sizeUNS16, OD_ACCESS_R);
    setODentry(d, 0x2F24, 0x00, &abs_encoder_raw_data, &sizeUNS32, OD_ACCESS_R);
    setODentry(d, 0x2F25, 0x00, &stop_motion, &sizeUNS16, OD_ACCESS_RW);	

    setODentry(d, 0x3001, 0x00, &motor_current_param[0], &sizeUNS32, OD_ACCESS_RW);		
    setODentry(d, 0x3001, 0x01, &motor_current_param[1], &sizeUNS32, OD_ACCESS_RW);	
	return 0;
}


int CANopenMasterODdataInit(CO_Data* d)
{
    setODentry(d, 0x2F01, 0x00, &empty_obj_2F01, &sizeUNS32, OD_ACCESS_RW);		
	return 0;
}


#endif


