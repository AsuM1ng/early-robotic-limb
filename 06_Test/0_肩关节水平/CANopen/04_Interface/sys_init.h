#ifndef __SYS_INIT_H
#define __SYS_INIT_H

void sys_init(void);
void hardware_init(void);
void task_flag_init(void);
void SystemClock_Config(void);
void Stm32_SoftReset(void);
void waiting_timer_init(void);

#endif

