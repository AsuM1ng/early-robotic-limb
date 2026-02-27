#ifndef __EXTI_H
#define __EXTI_H
#include "sys.h"

#define KEY_RIGHT_INT_GPIO_PORT              GPIOE
#define KEY_RIGHT_INT_GPIO_PIN               SYS_GPIO_PIN4
#define KEY_RIGHT_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE????? */
#define KEY_RIGHT_INT_IRQn                   EXTI4_IRQn
#define KEY_RIGHT_INT_IRQHandler             EXTI4_IRQHandler

#define KEY_DOWN_INT_GPIO_PORT              GPIOE
#define KEY_DOWN_INT_GPIO_PIN               SYS_GPIO_PIN3
#define KEY_DOWN_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE????? */
#define KEY_DOWN_INT_IRQn                   EXTI3_IRQn
#define KEY_DOWN_INT_IRQHandler             EXTI3_IRQHandler

#define KEY_LEFT_INT_GPIO_PORT              GPIOE
#define KEY_LEFT_INT_GPIO_PIN               SYS_GPIO_PIN2
#define KEY_LEFT_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 4; }while(0)   /* PE????? */
#define KEY_LEFT_INT_IRQn                   EXTI2_IRQn
#define KEY_LEFT_INT_IRQHandler             EXTI2_IRQHandler

#define KEY_UP_INT_GPIO_PORT              GPIOA
#define KEY_UP_INT_GPIO_PIN               SYS_GPIO_PIN0
#define KEY_UP_INT_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 0; }while(0)   /* PA????? */
#define KEY_UP_INT_IRQn                   EXTI0_IRQn
#define KEY_UP_INT_IRQHandler             EXTI0_IRQHandler

/******************************************************************************************/


void extix_init(void);  /* ??????? */ 				    
#endif

