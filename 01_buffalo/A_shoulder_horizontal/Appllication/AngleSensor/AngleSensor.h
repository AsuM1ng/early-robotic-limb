#ifndef TDKANGLESENSOR_H_INCLUDED
#define TDKANGLESENSOR_H_INCLUDED

#include "stm32f4xx_spi.h"
#include "Board_type_config.h"


#define AVG_TEMP_LENGH      20

#define SPI_CH1         SPI1
#define SPI_CH2         SPI2

/* 定义SPI1 GPIO */
#define PORT_SPI_CH1    GPIOA
#define PIN_SCK_CH1     GPIO_Pin_5
#define PIN_MISO_CH1    GPIO_Pin_6
#define PIN_MOSI_CH1    GPIO_Pin_7

/* 定义读取AD转换芯片的SPI2 GPIO */
#define PORT_SPI_CH2    GPIOB
#define PIN_CS_CH2      GPIO_Pin_12
#define PIN_SCK_CH2     GPIO_Pin_13
#define PIN_MISO_CH2    GPIO_Pin_14
#define PIN_MOSI_CH2    GPIO_Pin_15


#define RCC_DATA_DE_GPIO            RCC_AHB1Periph_GPIOC
#define PORT_DATA_DE                GPIOC
#define DATA_DE_PIN                 GPIO_Pin_6

#define RCC_DATA_RE_GPIO            RCC_AHB1Periph_GPIOB
#define PORT_DATA_RE                GPIOB
#define DATA_RE_PIN                 GPIO_Pin_15


#define RCC_CLOCK_DE_GPIO           RCC_AHB1Periph_GPIOC
#define PORT_CLOCK_DE               GPIOC
#define CLOCK_DE_PIN                GPIO_Pin_8

#define RCC_CLOCK_RE_GPIO           RCC_AHB1Periph_GPIOC
#define PORT_CLOCK_RE               GPIOC
#define CLOCK_RE_PIN                GPIO_Pin_7



/*  chip 1 片选/触发信号GPIO 端口*/
#define CS_SPI_CH1_GPIO_CLK         RCC_AHB1Periph_GPIOA    /*A/D chip 1 片选/触发信号GPIO clock*/
#define PORT_CS_SPI_CH1             GPIOA
#define CS_SPI_CH1_PIN              GPIO_Pin_4

#define ANGLESENSOR_EN              GPIO_SetBits(PORT_CS_SPI_CH1,CS_SPI_CH1_PIN);
#define ANGLESENSOR_DISEN           GPIO_ResetBits(PORT_CS_SPI_CH1,CS_SPI_CH1_PIN);

typedef enum{
    Normal  = 1,
    Inverse = -1
}EncoderIncDir_en;


typedef struct{
    _Bool bHardwareInit;
    _Bool PositalDataReady;
    
    uint32_t u17SinglTrun;
    uint16_t u16MultTrun;
    
    uint32_t u17PreSinglTrun;
    uint16_t u16PreMultTrun;

    int32_t i32AngleOffset; /*绝对位置编码器初始角*/
    int32_t i32AngleBuff[AVG_TEMP_LENGH];
    int32_t i32SensorAngle; /*关节实际角度*/
    int32_t i32PreAngle;    /*前一次测得的角度*/
}Posital_Para_t;


extern Posital_Para_t PositalPara;

extern uint16_t SPI2_InitFinish;

void AbsAngleSensor_Init(void);

int16_t SPI1_ReadWriteByte(uint16_t TxData);
int16_t SPI2_ReadWriteByte(uint16_t TxData);

/*单圈编码器角度读取*/
int32_t GetSingleTurnSensorAngle(int32_t AngleOffset);

/*多圈编码器角度读取*/
int32_t GetPositalActualAngle(int32_t AngleOffset);

int32_t CalcJointUperArmRotateAngle(int32_t angleOffset);


#endif // TDKANGLESENSOR_H_INCLUDED
