#include "stdlib.h"
#include "stdio.h"
#include "Board_type_config.h"
#include "ST_AbsEncoder.h"
#include "CANopenObjConfig.h"
#include "TestSlave.h"
#include "logic_debug.h"
#include "chipHAL_TIM.h"
#include "chipHAL_IWDG.h"
#include "servo_driver_control.h"
#include "global.h"
#include "chipHAL_delay.h"
#include "servo_Control.h"
#include "CANopenMaster.h"
#include "servo_driver_control.h"
#include "read_write_SDO.h"
#include "JointControl.h"
#include "AngleSensor.h"
#include "chipHAL_GPIO.h"
#include "chipHAL_USART.h"



int main(void)
{
	int i = 0;
    boardHAL_init();
    
#if(PRINTF_EN)
    chipHAL_UsartInit(115200);
#else
    TestGpio_USART_Init();
#endif

    SoftTimerInit(&SoftTimer);
    delay_ms(500);
    
    /*CANOpen模块初始化，完成后即可与主控通信*/
	CanopenInit();

#if ('A' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE)
    while(!PositalPara.PositalDataReady);
    #endif

	while(1)
	{
		i = EposSwitchOn();
		i = EposHalt();
	}
	
	
    /*初始化伺服前，先判断伺服板是否上电，如果没上电就一直等待*/
//    while(EposCtrl.i16PowerSupply < 36)
//    {
//        GetServoPowerSupply();
//        delay_ms(20);
//    }
//    
//    delay_ms(200);
//    
//    /*EPOS4 伺服控制器初始化*/
//    while(!JointCtrl.u16JointStatus.Status_bit.b0_InitFinished)
//    {
//        JointControlParaInit();
//    }

//	while(1)
//	{
//        /*关节控制过程*/
//        if(TRUE == JointCtrl.bJointProcessStart)
//        {
//            JointCtrl.bJointProcessStart = FALSE;
//            JointControlProcess();
//        }


//        if(TRUE == JointCtrl.asynPdoEnable)
//        {
//            JointCtrl.asynPdoEnable = FALSE;
//            PDOSendJointAngle();
//        }

//#if ('B' == JOINTBOARD_TYPE)
//        BreakControlProcess(JointCtrl.Motion.BreakCtrl);
//        JointCtrl.Motion.BreakStatus = GetBreakStatus();
//#endif
//	}

	return 0;
}


