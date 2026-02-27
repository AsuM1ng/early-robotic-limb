#include "ST_AbsEncoder.h"
#include "chipHAL_GPIO.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
//#include "delay.h"

#include "TestSlave.h"


#define BIT_NUM 18
#define LIST_LENGTH 40
#define ABS_ENCODER_MAX_RANGE 262144   // 18bit绝对式编码器能计数的最大值
#define DATA_CHANGE_THRESHOLD 3640     // 变化5°对应的绝对式编码器delta数值

volatile uint8_t g_bit_of_absolute_encoder_data = 0;
uint8_t clock_electrical_level = 0;
uint8_t clock_state = 0;


// 注意backupAbsoluteEncoderData();和loadAbsoluteEncoderData()两个函数，暂时被注释掉了

extern UNS32 set_clock_idle_times;
extern UNS32 set_clocK_start_times;
extern UNS32 set_clock_high_times;
extern UNS32 set_clock_low_times;
extern UNS32 set_clock_monoflop_times;

extern float joint_angle;
extern int joint_angle_int;

//int bbuugg=0;



uint32_t g_actual_data_of_absolute_encoder = 0;
uint32_t actual_data_of_absolute_encoder_pre = 0;
uint32_t original_data_of_absolute_encoder[LIST_LENGTH] = {0};//编码器原始数据，取50个是为了平均值滤波，下标越大，存放的值越老，下标越小，存放的值越新，最老的值需要移出数组，最新的值需要移进数组

//uint8_t flag_of_data_transformed = 0;
//uint32_t watch_origin_angle=0;

void getTempDataOfAbsoluteEncoder(void)
{
	static uint32_t temp_data_of_absolute_encoder = 0;//绝对编码器中间数据，对于一个10位精度的编码器，需要连续的十次外部中断该变量才能表示角度值	

    temp_data_of_absolute_encoder |= ((uint32_t)GPIO_ReadInputDataBit(ABSOLUTE_ENCODER_DATA_GPIO_PORT, ABSOLUTE_ENCODER_DATA_GPIO_PIN)) << (BIT_NUM - g_bit_of_absolute_encoder_data);
    if(g_bit_of_absolute_encoder_data == BIT_NUM)
    {
        g_bit_of_absolute_encoder_data = 0;
        g_actual_data_of_absolute_encoder = temp_data_of_absolute_encoder;
        temp_data_of_absolute_encoder = 0;
    }
}


void UpdateAbsEncoderDataQueue(void)
{
//	int value_err = 0;
//	
//	// update the absEncoder data only if the data is in the normal range
//	if(g_actual_data_of_absolute_encoder > 0 && actual_data_of_absolute_encoder_pre == 0)
//	{
//		actual_data_of_absolute_encoder_pre = g_actual_data_of_absolute_encoder;
//	}
//	else
//	{
//	    value_err = abs((int)g_actual_data_of_absolute_encoder - original_data_of_absolute_encoder[0]);	
//	}

//	if((g_actual_data_of_absolute_encoder <= ABS_ENCODER_MAX_RANGE) && (value_err < DATA_CHANGE_THRESHOLD))
	if(g_actual_data_of_absolute_encoder <= ABS_ENCODER_MAX_RANGE)		
	{
		for(int i = (LIST_LENGTH-2); i >= 0; i--)
		{
			original_data_of_absolute_encoder[i+1] = original_data_of_absolute_encoder[i];
		}
		original_data_of_absolute_encoder[0] = g_actual_data_of_absolute_encoder;	
	}
}

//UNS32 clock_idle_time     = 5;
//UNS32 clocK_start_time    = 5;
//UNS32 clock_high_time     = 8;//不管clock_monoflop_time+clock_idle_time的值为多少，超过12似乎采集回来的数据就有问题
//UNS32 clock_low_time      = 8;//不管clock_monoflop_time+clock_idle_time的值为多少，超过12似乎采集回来的数据就有问题
//UNS32 clock_monoflop_time = 20;
//extern s_timer_entry timers[MAX_NB_TIMER];
//TIMER_HANDLE absoluteEncoderTimer = TIMER_NONE;
//void absoluteEncoderClockIdle(CO_Data* d, UNS32 id)
//{
//    setChipBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    
////    clock_electrical_level = 1;
////    clock_state = 1;
//    
//    absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockIdle, d);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
//    absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockStart, clock_idle_time, 0);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
////    if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////    {
////        if(timers[0].callback == absoluteEncoderClockStart)
////        {
////            if(timers[0].state == 0)
////            {
////                bbuugg++;
////            }
////        }
////    }
//}
//void absoluteEncoderClockStart(CO_Data* d, UNS32 id)
//{
//    GPIO_ResetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    
////    clock_electrical_level = 0;
////    
////    clock_state = 2;
//    
//    absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockStart, d);
//    CAN_ITConfig(d->canHandle, CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
//    absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockHigh, clocK_start_time, 0);
//    CAN_ITConfig(d->canHandle, CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
////    if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////    {
////        if(timers[0].callback == absoluteEncoderClockHigh)
////        {
////            if(timers[0].state == 0)
////            {
////                bbuugg++;
////            }
////        }
////    }
//}
//void absoluteEncoderClockHigh(CO_Data* d, UNS32 id)
//{
//    g_bit_of_absolute_encoder_data++;
//    
//    setChipBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    
////    clock_electrical_level = 1;
////    
////    clock_state = 3;
//    
//    absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockHigh, d);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
//    absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockLow, clock_high_time, 0);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
////    if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////    {
////        if(timers[0].callback == absoluteEncoderClockLow)
////        {
////            if(timers[0].state == 0)
////            {
////                bbuugg++;
////            }
////        }
////    }
//}


//void absoluteEncoderClockLow(CO_Data* d, UNS32 id)
//{   
//    setChipBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    
////    clock_electrical_level = 0;
////    
////    clock_state = 4;

//    if((g_bit_of_absolute_encoder_data >= 1) && (g_bit_of_absolute_encoder_data <= BIT_NUM - 1)) 
//    {
//        absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockLow, d);
//        CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//        
//        absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockHigh, clock_low_time, 0);
//        CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//        
////        if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////        {
////            if(timers[0].callback == absoluteEncoderClockHigh)
////            {
////                if(timers[0].state == 0)
////                {
////                    bbuugg++;
////                }
////            }
////        }        
//    }
//    if(g_bit_of_absolute_encoder_data == BIT_NUM) 
//    {   
//        absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockLow, d);
//        CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//        
//        absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockMonoflop, clock_low_time, 0); 
//        CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断 
//        
////        if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////        {
////            if(timers[0].callback == absoluteEncoderClockMonoflop)
////            {
////                if(timers[0].state == 0)
////                {
////                    bbuugg++;
////                }
////            }
////        }
//    }
//    getTempDataOfAbsoluteEncoder();
//}

//void absoluteEncoderClockMonoflop(CO_Data* d, UNS32 id)
//{
//    setChipBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    
////    clock_electrical_level = 1;
////    
////    clock_state = 5;
//    
//    absoluteEncoderTimer = DelAlarm (absoluteEncoderTimer, &absoluteEncoderClockMonoflop, d);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
//    absoluteEncoderTimer = SetAlarm (&TestSlave_Data, 0, &absoluteEncoderClockIdle, clock_monoflop_time, 0);
//    CAN_ITConfig(d->canHandle,CAN_IT_FMP0, ENABLE);//该函数现在可允许被中断打断
//    
////    joint_angle = get_abs_encoder();
////    joint_angle_int = joint_angle;
//    
////    if(set_clock_idle_times == 1 || set_clocK_start_times == 1 || set_clock_high_times == 1 || set_clock_low_times == 1 || set_clock_monoflop_times == 1)
////    {
////        if(timers[0].callback == absoluteEncoderClockIdle)
////        {
////            if(timers[0].state == 0)
////            {
////                bbuugg++;
////            }
////        }
////    }
//}


/*
 插入排序法
 参数是：需要排序的数组，数组元素个数
 无返回
*/
void InsertSort(uint32_t absolute_encoder_data[],int n)  
{
	int i;
    for(i= 1; i<n; i++)
	{
        if(absolute_encoder_data[i] < absolute_encoder_data[i-1])
		{
            int j= i-1;   
            int x = absolute_encoder_data[i];         
            absolute_encoder_data[i] = absolute_encoder_data[i-1];          
            while(x < absolute_encoder_data[j])
			{
                absolute_encoder_data[j+1] = absolute_encoder_data[j];  
                j--;
            }
            absolute_encoder_data[j+1] = x;      
        }
    }
}

/**
	*@brief	将编码器输出的模拟电压转换为位置信息 在system_infomation.c中更新g_system_information.Raise的值
            进行绝对编码器的校正和过线处理，此时磁珠随便塞进去固定即可，不用考虑角度的问题，因为以下的程序对这个问题已做了处理
	*@note
		绝对编码器安装在电机的输出端，绝对编码器4096线
		ADCConvertedValue_16为绝对编码器的输出（数字的），
		此时的角度=ADCConvertedValue_16*360°/4096
	*@return 此时的连杆所处的角度
	*/


//#define ABS_ENCODER_TEST
//extern uint32_t original_data_of_absolute_encoder[LIST_LENGTH];
double Encoder_when_key = 0;//校正绝对编码按下按键时绝对编码器的原始角度

#define LIMIT_ANGLE 0.0
//#define LIMIT_ANGLE_KNEE -2.0
#define LIMIT_ANGLE_KNEE 0.0

uint32_t arry[LIST_LENGTH];

//                           done, done, done, done
int fixed_zero_offset[4] = {184, 78, 77, 165};
float watch_angle = 0 ;

// 临时解决方案,后面再解耦，目标应该是通过给函数不同的参数处理不同的ABS 编码器
float get_abs_encoder(int serial_number)
{
	double angle;
	float abs_encoder_cnts_average=0.0f;
	int sum=0;
	char i=0;
	
	double Encoder_measure = 0;	//编码器测到的角度的实时值
	int Encoder_measure_int = 0;	//编码器测到的角度的实时值	
	
	uint8_t start = 10; // choose the middle part of the data to do the meanvalue filter
	uint8_t end = 30;	

	UpdateAbsEncoderDataQueue();
	
	/*平均值滤波*/
	for(i = 0; i < LIST_LENGTH; i++)
	{
		arry[i] = original_data_of_absolute_encoder[i];
		
		#if JOINTBOARD_TYPE == 'B'
		arry_nopower[i] = original_data_of_absolute_encoder_nopower[i];
		#endif
    }
	InsertSort(arry,LIST_LENGTH);
	
	for(i = start; i < end;i++)
	{
		sum += arry[i];	
	}
	abs_encoder_cnts_average = sum / 20.0f;//ADC采集到的数字值（0~4096）
	Encoder_measure = abs_encoder_cnts_average / 262144.0f * 360.0f;//数字值转化成角度值（0~360度）
	Encoder_measure_int = (int)Encoder_measure;
	
	watch_angle = Encoder_measure;

	if(key_Encoder)//如果同时按下了两个按键
	{
		Encoder_when_key = abs_encoder_cnts_average/262144.0f*360.0f;//捕捉按下按键瞬间的绝对编码器角度值
//		buzzer_on();
//		delay_ms(1000);
//      backupAbsoluteEncoderData();
//		STMFLASH_Write(0x0800C000,(uint32_t*)&Encoder_when_key,1);
//		buzzer_off();
		key_Encoder = 0;
//		Encoder_when_key=STMFLASH_ReadWord(0x0800C000);    //从Flash中恢复Encoder_when_key的值
//        loadAbsoluteEncoderData();
	}
	
	switch(*TestSlave_Data.bDeviceNodeId)
	{
        case 1: 
			
		Encoder_when_key = fixed_zero_offset[0];		// temporary asigned a fixed offset
        if(Encoder_when_key >= 0 && Encoder_when_key <= 180)
        {
            if(Encoder_measure <= Encoder_when_key)
            {
                angle = -(Encoder_when_key - Encoder_measure) - LIMIT_ANGLE;
            } 
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            { 
                angle = Encoder_measure - Encoder_when_key - LIMIT_ANGLE;
            }
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = -(360 - Encoder_measure + Encoder_when_key) - LIMIT_ANGLE;
            }
        }
        if(Encoder_when_key >= 180 && Encoder_when_key <= 360)
        {
            if(Encoder_measure > Encoder_when_key)
            {
                angle = Encoder_measure - Encoder_when_key - LIMIT_ANGLE;
            }           
            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = -(Encoder_when_key - Encoder_measure) - LIMIT_ANGLE;
            }         
            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = 360 - Encoder_when_key + Encoder_measure - LIMIT_ANGLE;
            }
        }
        break;
			
        case 2: 
		Encoder_when_key = fixed_zero_offset[1];		// temporary asigned a fixed offset			
        if(Encoder_when_key >= 0 && Encoder_when_key <= 180)
        {
            if(Encoder_measure <= Encoder_when_key)
            {
                angle = -(Encoder_when_key - Encoder_measure) + LIMIT_ANGLE_KNEE;
            }          
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = Encoder_measure - Encoder_when_key + LIMIT_ANGLE_KNEE;
            }         
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = -(360 - Encoder_measure + Encoder_when_key) + LIMIT_ANGLE_KNEE;
            }                    
        }
		
        if(Encoder_when_key >= 180 && Encoder_when_key <= 360)
        {
            if(Encoder_measure >= Encoder_when_key)
            {
                angle = Encoder_measure - Encoder_when_key + LIMIT_ANGLE_KNEE;
            }                 
            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = -(Encoder_when_key - Encoder_measure) + LIMIT_ANGLE_KNEE;
            }          
            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = 360 - Encoder_when_key + Encoder_measure + LIMIT_ANGLE_KNEE;
            }                
        }

        break;

        case 3: 
		Encoder_when_key = fixed_zero_offset[2];		// temporary asigned a fixed offset
        if(Encoder_when_key >= 0 && Encoder_when_key <= 180)
        {
            if(Encoder_measure <= Encoder_when_key)
            {
                angle = Encoder_when_key - Encoder_measure - LIMIT_ANGLE;
            }                
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = -(Encoder_measure - Encoder_when_key) - LIMIT_ANGLE;
            }                
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = 360 - Encoder_measure + Encoder_when_key - LIMIT_ANGLE;
            }                
        }

        if(Encoder_when_key >= 180 && Encoder_when_key <= 360)
        {
            if(Encoder_measure > Encoder_when_key)
            {
                angle = -(Encoder_measure - Encoder_when_key) - LIMIT_ANGLE;
            }                      
            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = -(360 - Encoder_when_key + Encoder_measure) - LIMIT_ANGLE;
            }                          
            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = Encoder_when_key - Encoder_measure - LIMIT_ANGLE;
            }                       
        }
        break;

        case 4: 
		Encoder_when_key = fixed_zero_offset[3];		// temporary asigned a fixed offset			
        if(Encoder_when_key >= 0 && Encoder_when_key <= 180)
        {
            if(Encoder_measure <= Encoder_when_key)
            {
                angle = Encoder_when_key - Encoder_measure + LIMIT_ANGLE_KNEE;
            }                    
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = -(Encoder_measure - Encoder_when_key) + LIMIT_ANGLE_KNEE;
            }                      
            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = 360 - Encoder_measure + Encoder_when_key + LIMIT_ANGLE_KNEE;
            }
        }

        if(Encoder_when_key >= 180 && Encoder_when_key <= 360)
        {
            if(Encoder_measure > Encoder_when_key)
            {
                angle = -(Encoder_measure - Encoder_when_key) + LIMIT_ANGLE_KNEE;
            }   
            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 180)
            {
                angle = -(360 - Encoder_when_key + Encoder_measure) + LIMIT_ANGLE_KNEE;
            }                
            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 180)
            {
                angle = Encoder_when_key - Encoder_measure + LIMIT_ANGLE_KNEE;
            }                
        }
        break;
  }
	return angle;
}



//float get_abs_encoder(void)
//{
//	double angle;
//	float ADCConvertedValue_16_Average=0.0f;
//	int sum=0;
//	char i=0;

//	/*平均值滤波*/
//	for(i = 0; i < LIST_LENGTH; i++)
//	{
//		 arry[i] = original_data_of_absolute_encoder[i];
//    }
//	InsertSort(arry,LIST_LENGTH);
//	for(i = start; i < end;i++)
//	{
//		sum += arry[i];
//	}
//	ADCConvertedValue_16_Average = sum/20.0f;//ADC采集到的数字值（0~4096）
//	Encoder_measure = ADCConvertedValue_16_Average / 262144.0f * 360.0f;//数字值转化成角度值（0~360度）
//	Encoder_measure_int = (int)Encoder_measure;
//	if(key_Encoder)//如果同时按下了两个按键
//	{
//		Encoder_when_key = ADCConvertedValue_16_Average/262144.0f*360.0f;//捕捉按下按键瞬间的绝对编码器角度值
////		buzzer_on();
////		delay_ms(1000);
////        backupAbsoluteEncoderData();
////		STMFLASH_Write(0x0800C000,(uint32_t*)&Encoder_when_key,1);
////		buzzer_off();
//		key_Encoder = 0;
////		Encoder_when_key=STMFLASH_ReadWord(0x0800C000);    //从Flash中恢复Encoder_when_key的值
////        loadAbsoluteEncoderData();
//	}
//	
//	switch(*TestSlave_Data.bDeviceNodeId)
//	{
//        case 1: 
//        if(Encoder_when_key >= 0 && Encoder_when_key <= 140)
//        {
//            if(Encoder_measure <= Encoder_when_key)
//            {
//                angle = -(Encoder_when_key - Encoder_measure) - LIMIT_ANGLE;
//            }         
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 140)
//            {
//                angle = Encoder_measure - Encoder_when_key - LIMIT_ANGLE;
//            }
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 140)
//            {
//                angle = -(360 - Encoder_measure + Encoder_when_key) - LIMIT_ANGLE;
//            }
//        }
//        if(Encoder_when_key > 140 && Encoder_when_key < 220)
//        {
//            angle = Encoder_measure-Encoder_when_key - LIMIT_ANGLE;
//        }
//        if(Encoder_when_key >= 220 && Encoder_when_key <= 360)
//        {
//            if(Encoder_measure > Encoder_when_key)
//            {
//                angle = Encoder_measure - Encoder_when_key - LIMIT_ANGLE;
//            }           
//            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 140)
//            {
//                angle = -(Encoder_when_key - Encoder_measure) - LIMIT_ANGLE;
//            }         
//            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 140)
//            {
//                angle = 360 - Encoder_when_key + Encoder_measure - LIMIT_ANGLE;
//            }
//        }
//        break;
//			
//        case 2: 
//        if(Encoder_when_key >= 0 && Encoder_when_key <= 130)
//        {
//            if(Encoder_measure <= Encoder_when_key)
//            {
//                angle = -(Encoder_when_key - Encoder_measure) + LIMIT_ANGLE_KNEE;
//            }          
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 130)
//            {
//                angle = Encoder_measure - Encoder_when_key + LIMIT_ANGLE_KNEE;
//            }         
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 130)
//            {
//                angle = -(360 - Encoder_measure + Encoder_when_key) + LIMIT_ANGLE_KNEE;
//            }                    
//        }
//        if(Encoder_when_key > 130 && Encoder_when_key < 230)
//        {
//            angle = Encoder_measure-Encoder_when_key + LIMIT_ANGLE_KNEE;
//        }
//        if(Encoder_when_key >= 230 && Encoder_when_key <= 360)
//        {
//            if(Encoder_measure >= Encoder_when_key)
//            {
//                angle = Encoder_measure - Encoder_when_key + LIMIT_ANGLE_KNEE;
//            }                 
//            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 130)
//            {
//                angle = -(Encoder_when_key - Encoder_measure) + LIMIT_ANGLE_KNEE;
//            }          
//            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 130)
//            {
//                angle = 360 - Encoder_when_key + Encoder_measure + LIMIT_ANGLE_KNEE;
//            }                
//        }
//        break;

//        case 3: 
//        if(Encoder_when_key >= 0 && Encoder_when_key <= 140)
//        {
//            if(Encoder_measure <= Encoder_when_key)
//            {
//                angle = Encoder_when_key - Encoder_measure - LIMIT_ANGLE;
//            }                
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 140)
//            {
//                angle = -(Encoder_measure - Encoder_when_key) - LIMIT_ANGLE;
//            }                
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 140)
//            {
//                angle = 360 - Encoder_measure + Encoder_when_key - LIMIT_ANGLE;
//            }                
//        }
//        if(Encoder_when_key > 140 && Encoder_when_key < 220)
//        {
//            angle = Encoder_when_key - Encoder_measure - LIMIT_ANGLE;
//        }
//        if(Encoder_when_key >= 220 && Encoder_when_key <= 360)
//        {
//            if(Encoder_measure > Encoder_when_key)
//            {
//                angle = -(Encoder_measure - Encoder_when_key) - LIMIT_ANGLE;
//            }                      
//            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 140)
//            {
//                angle = -(360 - Encoder_when_key + Encoder_measure) - LIMIT_ANGLE;
//            }                          
//            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 140)
//            {
//                angle = Encoder_when_key - Encoder_measure - LIMIT_ANGLE;
//            }                       
//        }
//        break;

//        case 4: 
//        if(Encoder_when_key >= 0 && Encoder_when_key <= 130)
//        {
//            if(Encoder_measure <= Encoder_when_key)
//            {
//                angle = Encoder_when_key - Encoder_measure + LIMIT_ANGLE_KNEE;
//            }                    
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 130)
//            {
//                angle = -(Encoder_measure - Encoder_when_key) + LIMIT_ANGLE_KNEE;
//            }                      
//            if(Encoder_measure > Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 130)
//            {
//                angle = 360 - Encoder_measure + Encoder_when_key + LIMIT_ANGLE_KNEE;
//            }
//        }
//        if(Encoder_when_key > 130 && Encoder_when_key < 230)
//        {
//            angle = Encoder_when_key - Encoder_measure + LIMIT_ANGLE_KNEE;
//        }
//        if(Encoder_when_key >= 230 && Encoder_when_key <= 360)
//        {
//            if(Encoder_measure > Encoder_when_key)
//            {
//                angle = -(Encoder_measure - Encoder_when_key) + LIMIT_ANGLE_KNEE;
//            }   
//            if(Encoder_measure < Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) > 130)
//            {
//                angle = -(360 - Encoder_when_key + Encoder_measure) + LIMIT_ANGLE_KNEE;
//            }                
//            if(Encoder_measure <= Encoder_when_key && fabs(Encoder_measure-Encoder_when_key) <= 130)
//            {
//                angle = Encoder_when_key - Encoder_measure + LIMIT_ANGLE_KNEE;
//            }                
//        }
//        break;
//  }
//	return angle;
//}




//uint16_t clockGeneratorOfAbsoluteEncoder(uint8_t resolution, uint8_t t_start, uint8_t t_clockhigh, uint8_t t_clocklow, uint8_t t_monoflop)
//{
//    GPIO_SetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    delay_us(t_monoflop);
//    GPIO_ResetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    delay_us(t_start);
//    
//    /* 打开外部中断使能，以在时钟信号的下降沿处采集编码器输出数据 */
//    EXTI->IMR |= ABSOLUTE_ENCODER_CLOCK_EXTI_Line;

//    for(int i = 0; i < resolution; i++)
//    {
//        g_bit_of_absolute_encoder_data = i;
//        GPIO_SetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//        delay_us(t_clockhigh);
//        GPIO_ResetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//        delay_us(t_clocklow);
//    }
//    
//    /* 关闭外部中断使能，编码器输出数据已经结束了，不用再采集 */
//    EXTI->IMR &= ~ABSOLUTE_ENCODER_CLOCK_EXTI_Line;
//    
//    GPIO_SetBits(ABSOLUTE_ENCODER_CLOCK_GPIO_PORT,  ABSOLUTE_ENCODER_CLOCK_GPIO_PIN);
//    delay_us(t_monoflop);
//}

