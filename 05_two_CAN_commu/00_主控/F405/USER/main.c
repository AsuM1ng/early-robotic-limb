#include "main.h"	

PressFlag pressf;

int main(void)
{ 
	sys_init();

	while(1)
	{
		key_driver_task();
	}
}
