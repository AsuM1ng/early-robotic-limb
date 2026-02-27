#include "main.h"	

int main(void)
{ 
/******  系统初始化  ******/
	task_flag_init();
	sys_init();
	hardware_init();
	waiting_timer_init();

	PAout(8) = 0;
	PDout(2) = 0;
	delay_ms(1000);
	PAout(8) = 1;
	PDout(2) = 1;
	
	while(1)
	{
		key_test();
		
		Action_Process();
	}
}

