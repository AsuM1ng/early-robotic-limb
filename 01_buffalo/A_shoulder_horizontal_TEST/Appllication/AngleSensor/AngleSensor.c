#include "AngleSensor.h"
#include "chipHAL_delay.h"
#include "JointControl.h"
#include "Umath.h"
#include "stdlib.h"

uint16_t SPI2_InitFinish = 0;

Posital_Para_t PositalPara;

void SPI_CH1_Init(void);
#if ('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE)
void TDKAngleSensor_CS_Init();
#endif

void Posital_DataClockDir(void);
void SPI_CH2_Init(void);

    
void AbsAngleSensor_Init(void)
{
#if (('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE))
    TDKAngleSensor_CS_Init();
    SPI_CH1_Init();
#elif (('A' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE))
    Posital_DataClockDir();
    SPI_CH2_Init();
    PositalPara.i32AngleOffset = JOINT_ANGLE_OFFSET;
    PositalPara.bHardwareInit = TRUE;
#endif
}

void TDKAngleSensor_CS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(CS_SPI_CH1_GPIO_CLK, ENABLE);
    
    /*通用设置*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//GPIO_PuPd_UP;
    
    /*初始化通道1选择GPIO*/
    GPIO_InitStructure.GPIO_Pin = CS_SPI_CH1_PIN;
    GPIO_Init(PORT_CS_SPI_CH1, &GPIO_InitStructure);
    GPIO_ResetBits(PORT_CS_SPI_CH1, CS_SPI_CH1_PIN);  //TAD2141片选拉低
}

void SPI_CH1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟

    //GPIOA 4,5,6,7初始化设置
    GPIO_InitStructure.GPIO_Pin = PIN_SCK_CH1|PIN_MISO_CH1|PIN_MOSI_CH1;//PA4~7复用功能输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(PORT_SPI_CH1, &GPIO_InitStructure);//初始化

//    GPIO_PinAFConfig(PORT_SPI_CH1,GPIO_PinSource4,GPIO_AF_SPI1); //PA4复用为 SPI1
    GPIO_PinAFConfig(PORT_SPI_CH1,GPIO_PinSource5,GPIO_AF_SPI1); //PA5复用为 SPI1
    GPIO_PinAFConfig(PORT_SPI_CH1,GPIO_PinSource6,GPIO_AF_SPI1); //PA6复用为 SPI1
    GPIO_PinAFConfig(PORT_SPI_CH1,GPIO_PinSource7,GPIO_AF_SPI1); //PA7复用为 SPI1
    
    //这里只针对SPI口初始化
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收16位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为64
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI_CH1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
    SPI_Cmd(SPI_CH1, ENABLE); //使能SPI外设
    SPI1_ReadWriteByte(0xff);
    SPI1_ReadWriteByte(0xff);
}

void Posital_DataClockDir(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_DATA_DE_GPIO, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_DATA_RE_GPIO, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_CLOCK_DE_GPIO, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_CLOCK_RE_GPIO, ENABLE);
    
    /*通用设置*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    
    /*初始化通道1选择GPIO*/
    GPIO_InitStructure.GPIO_Pin = DATA_DE_PIN;
    GPIO_Init(PORT_DATA_DE, &GPIO_InitStructure);
    GPIO_ResetBits(PORT_DATA_DE, DATA_DE_PIN);  /*数据DE 拉低，数据输出端为高阻*/
    
    GPIO_InitStructure.GPIO_Pin = DATA_RE_PIN;
    GPIO_Init(PORT_DATA_RE, &GPIO_InitStructure);
    GPIO_ResetBits(PORT_DATA_RE, DATA_RE_PIN);  /*数据RE 拉低，使能差分数据输入*/
    
    GPIO_InitStructure.GPIO_Pin = CLOCK_DE_PIN;
    GPIO_Init(PORT_CLOCK_DE, &GPIO_InitStructure);
    GPIO_SetBits(PORT_CLOCK_DE, CLOCK_DE_PIN);  /*时钟DE 拉高，使能差分时钟输出*/
    
    GPIO_InitStructure.GPIO_Pin = CLOCK_RE_PIN;
    GPIO_Init(PORT_CLOCK_RE, &GPIO_InitStructure);
    GPIO_SetBits(PORT_CLOCK_RE, CLOCK_RE_PIN);  /*时钟RE 拉高，差分时钟输入为高阻*/
}

void SPI_CH2_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟

    //GPIOFB13,14,15初始化设置
    GPIO_InitStructure.GPIO_Pin = PIN_SCK_CH2|PIN_MISO_CH2;//PA3~5复用功能输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(PORT_SPI_CH2, &GPIO_InitStructure);//初始化

    GPIO_PinAFConfig(PORT_SPI_CH2,GPIO_PinSource13,GPIO_AF_SPI2); //PB12复用为 SPI2
    GPIO_PinAFConfig(PORT_SPI_CH2,GPIO_PinSource14,GPIO_AF_SPI2); //PA6复用为 SPI2

    //这里只针对SPI口初始化
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI2
    RCC_APB2PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI2

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       //设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//定义波特率预分频的值:波特率预分频值为64
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI_CH2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
    
    SPI_Cmd(SPI_CH2, ENABLE); //使能SPI外设
    
    SPI2_ReadWriteByte(0xFFFF);
}




int16_t SPI1_ReadWriteByte(uint16_t TxData)
{
    while (SPI_I2S_GetFlagStatus(SPI_CH1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空

    SPI_I2S_SendData(SPI_CH1, TxData); //通过外设SPIx发送一个byte  数据
    
    while (SPI_I2S_GetFlagStatus(SPI_CH1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
    
    return SPI_I2S_ReceiveData(SPI_CH1); //返回通过SPIx最近接收的数据	
}

int16_t SPI2_ReadWriteByte(uint16_t TxData)
{
    while (SPI_I2S_GetFlagStatus(SPI_CH2, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空

    SPI_I2S_SendData(SPI_CH2, TxData); //通过外设SPIx发送一个byte  数据

    while (SPI_I2S_GetFlagStatus(SPI_CH2, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  

    return SPI_I2S_ReceiveData(SPI_CH2); //返回通过SPIx最近接收的数据
}



/** \brief  读取TDK单圈编码器原始数据，并转换为角度值
 *  
 * \param   void
 * \return  uint16_t 关节角度值，角度范围(真实角度 X 100)：0 ~ 36000
 *
 */
uint16_t GetSingleTurnAngle(void)
{
    uint16_t u16rawData = 0;
    uint32_t u32angleData = 0;

    uint8_t status_h = 0;
    uint8_t status_l = 0;

    /*SPI CMD_WRITE_REG*/
    ANGLESENSOR_DISEN;
    
    /*TDK传感器操作码*/
    SPI1_ReadWriteByte(0x05);

    /*读取角度原始数据*/
    u16rawData = SPI1_ReadWriteByte(0xFF);
    u16rawData = (u16rawData << 8) + SPI1_ReadWriteByte(0xFF);
    
    /*读取状态和CRC数据*/
    status_h = SPI1_ReadWriteByte(0xFF);
    status_l = SPI1_ReadWriteByte(0xFF);
    
    ANGLESENSOR_EN;
    
    /*最大角度值为65535，65535 * 36000 = 2359260000，小于uint32_t 表示的最大数据，不会溢出*/
    u32angleData = 0xFFFF & ((u16rawData * 36000) >> 16);

    return (uint16_t) u32angleData;
}


/** \brief  关节角度读取函数(单圈绝对值编码器)
 *          读取当前关节角度；
 * \param   int32_t AngleOffset     关节安装角度偏移
 * \return  int32_t 关节角度值，角度范围：0 ~ 360度(0~36000)
 *                  肘关节角度范围 -180 ~0~ +180
 *
 */
int32_t GetSingleTurnSensorAngle(int32_t AngleOffset)
{
    int32_t li32AngleTemp = 0;

    li32AngleTemp = (int32_t)(GetSingleTurnAngle() - AngleOffset);
    
    if(li32AngleTemp < 0)
    {
        li32AngleTemp += 36000;
    }

    li32AngleTemp = li32AngleTemp % 36000;

#if('F' == JOINTBOARD_TYPE)
    /*将实际角度转换为-180度 ~ +180*/
    if(li32AngleTemp >= 18000)
    {
        li32AngleTemp = (li32AngleTemp - 36000);
    }
#endif
    
    return li32AngleTemp;
}




int32_t absValue = 0;
int32_t GetPositalActualAngle(int32_t AngleOffset)
{
    uint16_t RawBuff[3] = {0};
    uint16_t index = 0;
    int32_t i32AngleTemp = 0;
    int32_t i32ActualAngle = 0;
    int32_t i32DAngleDt = 0;
    
    int64_t i64TempData = 0;
    
    static uint16_t count = 0;

    for(index = 0; index < 3; index++)
    {
        while(((SPI2->SR & 0x0002) == RESET));
        SPI2->DR = 0xFFFF;
        
        while(((SPI2->SR & 0x0001) == RESET));
        RawBuff[index] = SPI2->DR;
    }

    /*获取多圈值*/
    PositalPara.u16MultTrun = (((RawBuff[0] & 0x007F) << 9) |\
                            ((RawBuff[1] & 0xFF80) >> 7));
    
    /*获取单圈值*/
    PositalPara.u17SinglTrun = (((RawBuff[1] & 0x007F) << 10) |\
                            ((RawBuff[2] & 0xFFC0) >> 6));     /*更新编码器单圈值*/
    
    i32AngleTemp = ((PositalPara.u16MultTrun << 17 | PositalPara.u17SinglTrun) & 0xFFFFFFFF);

#if ('A' == JOINTBOARD_TYPE)
    i32AngleTemp = i32AngleTemp/(JOINT_GEAR_REDUCTION * 360 / 100);

    /*角度数据原始值 - 偏移角度 = 实际关节角度*/
    i32ActualAngle = (int32_t)(i32AngleTemp - AngleOffset);
    
    i32ActualAngle = 36000 - i32ActualAngle;

    i32ActualAngle = i32ActualAngle % 36000;
    
#elif ('E' == JOINTBOARD_TYPE)
    i32ActualAngle = (((0x03372CF4 - i32AngleTemp) * 100)/2564);
#endif

    /*对关节角度数据求平均值，滑动平均滤波*/
    if(!PositalPara.PositalDataReady)   /*初始化时，记录连续40个数据，并求平均值*/
    {
        if(count < AVG_TEMP_LENGH)      /*400mS*/
        {
            count += 1;
            PositalPara.i32SensorAngle = (Filter_MovingAverage(PositalPara.i32AngleBuff,i32ActualAngle,AVG_TEMP_LENGH));
            PositalPara.i32PreAngle = PositalPara.i32SensorAngle;
        }
        else
        {
            PositalPara.PositalDataReady = TRUE;
        }
        return i32ActualAngle;
    }
    else
    {
        /*如果当前角度大于前一次滤波值40度，则认为当前测得的角度值无效*/
        i32DAngleDt = i32ActualAngle - PositalPara.i32SensorAngle;
        if(abs(i32DAngleDt) < 40 * 100)
        {
            PositalPara.i32SensorAngle = Filter_MovingAverage(PositalPara.i32AngleBuff,i32ActualAngle,AVG_TEMP_LENGH);
            PositalPara.i32PreAngle = PositalPara.i32SensorAngle;
        }
        else
        {
            PositalPara.i32SensorAngle = PositalPara.i32PreAngle;
        }
    }

    return PositalPara.i32SensorAngle;
}

/** \brief  上臂旋转关节角度计算；
 *          本关节使用博斯特绝对位置编码器，安装在电机轴上，通过伺服控制器读取绝对位置；
 *          读取的值为绝对位置脉冲数，单圈分辨率为17bit，上臂机械旋转范围为-60度~+60度；
 * \param   int32_t angleOffset   零度位置偏移，根据实际位置测量得出 -300000inc
 * \return  int32_t 关节实际角度，范围: -60度~ +60度
 *
 */     

int32_t CalcJointUperArmRotateAngle(int32_t angleOffset)
{
    int32_t li32AngleTemp = 0;
    
    li32AngleTemp = (JointCtrl.Motion.i32JointActualPosition - angleOffset)/713;
    return li32AngleTemp;
}

