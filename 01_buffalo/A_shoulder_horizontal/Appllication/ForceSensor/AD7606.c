#include "AD7606.h"
#include "chipHAL_GPIO.h"

#if JOINTBOARD_TYPE == 'F'

#define Kalman_Q	0.02
#define Kalman_R	53.3

void Ad7606_HardInit(void);

void Ad7606_StartConv(void);
double KalmanFilterEdit(const double Input_Data,
						double ProcessNoise_Q,
							double MeasureNoise_R,
								double *x_last,
									double *p_last);
								
AD7606_Data_t Ad7606Para;


/** \brief 
 *
 * \param 
 * \param 
 * \return 
 *
 */     
void AD7606Config(void)
{
	uint8_t index = 0;
	Ad7606_HardInit();		/*配置SPI通信口*/
	for (index = 0; index < FORCE_CH_NUM; index++)
	{
		Ad7606Para.Kalman_x_last[index] = 0.0;
		Ad7606Para.Kalman_p_last[index] = 0.0;
	}
}


void Ad7606_HardInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为32
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	

	/* 配置RESET GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_RESET_PIN;
	GPIO_Init(AD_RESET_GPIO_PORT, &GPIO_InitStructure);

	/* 配置CONVST GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_CONVST_PIN;
	GPIO_Init(AD_CONVST_GPIO_PORT, &GPIO_InitStructure);

	/* 配置RANGE GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_RANGE_PIN;
	GPIO_Init(AD_RANGE_GPIO_PORT, &GPIO_InitStructure);

	/* 配置OS0-2 GPIO */
	GPIO_InitStructure.GPIO_Pin = AD_OS0_PIN;
	GPIO_Init(AD_OS0_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS1_PIN;
	GPIO_Init(AD_OS1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD_OS2_PIN;
	GPIO_Init(AD_OS2_GPIO_PORT, &GPIO_InitStructure);
}



uint16_t Ad7606ReadWord(void)
{
	uint16_t SPIReadWord = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}

	//任何数据都可以
	//因为从模式是没法提供时钟的，所以主模式下必须要在接收的同时提供时钟。办法就是发送一个字节来实现.
	SPI_I2S_SendData(SPI1, 0xffff);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
	
	SPIReadWord = SPI_I2S_ReceiveData(SPI1);
	return SPIReadWord;
}


//void TIM4_IRQHandler(void)
//{
//	uint8_t index;
//	uint16_t tempValue;
//	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

//	for (index = 0; index < FORCE_CH_NUM; index++)
//	{
//		tempValue = Ad7606ReadWord();
//		Ad7606Para.i16ThreeAxisVal[index] = tempValue;
//		Ad7606Para.i16ThreeAxisFilterVal[index] = KalmanFilterEdit((double)tempValue,
//																	Kalman_Q,
//																	Kalman_R,
//																	&Ad7606Para.Kalman_x_last[index],
//																	&Ad7606Para.Kalman_p_last[index]);
//	}
//	Ad7606Para.i16Axis_x = Ad7606Para.i16ThreeAxisVal[0];
//	Ad7606Para.i16Axis_y = Ad7606Para.i16ThreeAxisVal[1];
//	Ad7606Para.i16Axis_z = Ad7606Para.i16ThreeAxisVal[2];

//	Ad7606_StartConv();
//}

double KalmanFilterEdit(const double Input_Data,
						double ProcessNoise_Q,
							double MeasureNoise_R,
								double *x_last,
									double *p_last)
{
	double R = MeasureNoise_R;
	double Q = ProcessNoise_Q;
	double x_mid = *x_last;
	double x_now;
	double p_mid;
	double p_now;
	double kg;
	
	x_mid = *x_last;
	p_mid = *p_last + Q;
	
	kg = p_mid/(p_mid + R);
	
	x_now = x_mid + (kg * (Input_Data - x_mid));
	p_now = (1.0 - kg) * p_mid;
	
	*p_last = p_now;
	*x_last = x_now;
	
	return x_now;
}

#endif
