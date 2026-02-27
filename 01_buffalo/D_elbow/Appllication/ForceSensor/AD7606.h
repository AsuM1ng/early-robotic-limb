#ifndef AD7606_H
#define AD7606_H

#include <stdint.h>
#include "chipHAL_SPI.h"

#if JOINTBOARD_TYPE == 'F'

#include "stm32f4xx.h"
#include "timer.h"

#ifndef TRUE
    #define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* 每个样本2字节，采集8个通道 */
#define FORCE_CH_NUM			3				/* 采集3通道 */
#define ADDIVISION				16

extern int16_t i16AD7606_Value[FORCE_CH_NUM];
/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */





/*　定义AD7606其它的GPIO */
#define AD_RESET_PIN                     GPIO_Pin_0
#define AD_RESET_GPIO_PORT               GPIOB
#define AD_RESET_GPIO_CLK                RCC_APB2Periph_GPIOB

#define AD_CONVST_PIN                    GPIO_Pin_1
#define AD_CONVST_GPIO_PORT              GPIOB
#define AD_CONVST_GPIO_CLK               RCC_APB2Periph_GPIOB

#define AD_OS0_PIN                     	 GPIO_Pin_2
#define AD_OS0_GPIO_PORT                 GPIOB		
#define AD_OS0_GPIO_CLK                  RCC_APB2Periph_GPIOB

#define AD_OS1_PIN                     	 GPIO_Pin_3
#define AD_OS1_GPIO_PORT               	 GPIOB		
#define AD_OS1_GPIO_CLK                	 RCC_APB2Periph_GPIOB

#define AD_OS2_PIN                     	 GPIO_Pin_4
#define AD_OS2_GPIO_PORT               	 GPIOB		
#define AD_OS2_GPIO_CLK                	 RCC_APB2Periph_GPIOB

#define AD_RANGE_PIN                     GPIO_Pin_5
#define AD_RANGE_GPIO_PORT               GPIOB
#define AD_RANGE_GPIO_CLK                RCC_APB2Periph_GPIOB

//引脚电平设置
//GPIOB->BRR  = 0x08就是把GPIOB port 3降为低电平
//GPIOB->BSRR = 0x08就是把GPIOB port 3升为高电平
#define CS_0()		PORT_CS->BRR = PIN_CS
#define CS_1()		PORT_CS->BSRR = PIN_CS

#define SCK_0()		PORT_SCK->BRR = PIN_SCK
#define SCK_1()		PORT_SCK->BSRR = PIN_SCK

#define MISO_0()	PORT_MISO->BRR = PIN_MISO
#define MISO_1()	PORT_MISO->BSRR = PIN_MISO

#define MOSI_0()	PORT_MOSI->BRR = PIN_MOSI
#define MOSI_1()	PORT_MOSI->BSRR = PIN_MOSI

#define MISO_IS_HIGH()	(GPIO_ReadInputDataBit(PORT_MISO, PIN_MISO) == Bit_SET)


#define AD_RESET_LOW()					AD_RESET_GPIO_PORT->BRR = AD_RESET_PIN
#define AD_RESET_HIGH()					AD_RESET_GPIO_PORT->BSRR = AD_RESET_PIN

#define AD_OS0_0()						AD_OS0_GPIO_PORT->BRR = AD_OS0_PIN
#define AD_OS0_1()						AD_OS0_GPIO_PORT->BSRR = AD_OS0_PIN

#define AD_OS1_0()						AD_OS1_GPIO_PORT->BRR = AD_OS1_PIN
#define AD_OS1_1()						AD_OS1_GPIO_PORT->BSRR = AD_OS1_PIN

#define AD_OS2_0()						AD_OS2_GPIO_PORT->BRR = AD_OS2_PIN
#define AD_OS2_1()						AD_OS2_GPIO_PORT->BSRR = AD_OS2_PIN

#define AD_CONVST_LOW()					AD_CONVST_GPIO_PORT->BRR = AD_CONVST_PIN
#define AD_CONVST_HIGH()				AD_CONVST_GPIO_PORT->BSRR = AD_CONVST_PIN

#define AD_RANGE_5V()					AD_RANGE_GPIO_PORT->BRR = AD_RANGE_PIN
#define AD_RANGE_10V()					AD_RANGE_GPIO_PORT->BSRR = AD_RANGE_PIN

typedef struct
{
	int16_t i16Axis_x;
	int16_t i16Axis_y;
	int16_t i16Axis_z;
	
	int16_t i16ThreeAxisVal[FORCE_CH_NUM];
	
	double Kalman_x_last[FORCE_CH_NUM];
	double Kalman_p_last[FORCE_CH_NUM];
	int16_t i16ThreeAxisFilterVal[FORCE_CH_NUM];
}AD7606_Data_t;


extern AD7606_Data_t Ad7606Para;

void AD7606Config(void);


#endif

#endif

