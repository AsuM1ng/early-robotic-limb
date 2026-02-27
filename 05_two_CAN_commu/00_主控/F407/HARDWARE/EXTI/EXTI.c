#include "stm32f4xx.h"
#include "delay.h"//ÑÓÊ±º¯ÊýËùÔÚµÄÍ·ÎÄ¼þ
#include "EXTI.h"
#include "KEY.h"
#include "dual_can.h"
#include "main.h"
#include "sys.h"

void KEY0_INT_IRQHandler(void)
{
    delay_ms(20);
    EXTI->PR = KEY_RIGHT_INT_GPIO_PIN;

    if (KEY_RIGHT == 0)
    {
        kf.right_key_flag = 1;
    }
}

void KEY1_INT_IRQHandler(void)
{ 
    delay_ms(20);
    EXTI->PR = KEY_DOWN_INT_GPIO_PIN;

    if (KEY_DOWN == 0)
    {
        LED0_TOGGLE();
    }
}

void KEY2_INT_IRQHandler(void)
{
    delay_ms(20);
    EXTI->PR = KEY_LEFT_INT_GPIO_PIN;

    if (KEY_LEFT == 0)
    {
        LED1_TOGGLE();
    }
}

void WKUP_INT_IRQHandler(void)
{ 
    delay_ms(20);
    EXTI->PR = KEY_UP_INT_GPIO_PIN;

    if (KEY_UP == 1)
    {
        BEEP_TOGGLE();
    }
}

void extix_init(void)
{
    key_init();
    sys_nvic_ex_config(KEY0_INT_GPIO_PORT, KEY0_INT_GPIO_PIN, SYS_GPIO_FTIR);   /* KEY0??????½????????? */
    sys_nvic_ex_config(KEY1_INT_GPIO_PORT, KEY1_INT_GPIO_PIN, SYS_GPIO_FTIR);   /* KEY1??????½????????? */
    sys_nvic_ex_config(KEY2_INT_GPIO_PORT, KEY2_INT_GPIO_PIN, SYS_GPIO_FTIR);   /* KEY2??????½????????? */
    sys_nvic_ex_config(WKUP_INT_GPIO_PORT, WKUP_INT_GPIO_PIN, SYS_GPIO_RTIR);   /* WKUP????????????????? */

    sys_nvic_init( 0, 2, KEY0_INT_IRQn, 2); /* ???0?????????2????2 */
    sys_nvic_init( 1, 2, KEY1_INT_IRQn, 2); /* ???1?????????2????2 */
    sys_nvic_init( 2, 2, KEY2_INT_IRQn, 2); /* ???2?????????2????2 */
    sys_nvic_init( 3, 2, WKUP_INT_IRQn, 2); /* ???3?????????2????2 */

}