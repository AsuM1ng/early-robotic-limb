#include "Para_monitor.h"

u8 Get_Powersupply(void)
{
	sendSDO(&Master_Data,SDO_CLIENT,0,Read_Powersupply);
	return MD.Power_Supply;
}

//u16 Get_error


