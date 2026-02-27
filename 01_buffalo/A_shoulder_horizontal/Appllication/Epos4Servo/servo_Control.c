#include "servo_Control.h"
#include "read_write_SDO.h"
#include "CANopenMaster.h"
#include "chipHAL_delay.h"
#include "TestSlave.h"
#include "canfestival.h"

SDORW_t Sdo_R_Para;
SDORW_t Sdo_W_Para;

ServoControl_t EposCtrl;

uint8_t ChangeProfileVelocity(uint32_t NewVelocity);

/*Epose4 极限参数初始化*/
void EposLimitParaInit(void)
{
    /*各模式通用极限参数设置*/
    EposCtrl.u32MaxAcceleration = MAXACCELERATION;
    EposCtrl.u32MotorAcceleration = PROFILEACCELERATION;
    EposCtrl.u32MotorDeceleration = PROFILEDECELERATION;
    EposCtrl.u32MotorQSDeceleration = QUICKSDECELERATION;
}



/*伺服驱动模式初始化*/
void ServoModeInit(EposOperationMode mode)
{
    /*modes of operation*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6060;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.dataType = OD_DATATYPE_U8;
    Sdo_W_Para.Data = mode;    /*伺服工作模式设定*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE0: Epos4 SDO write Mode Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*Max motor speed*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6080;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MotorMaxVelocity;    /*设定伺服最高速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE0: Epos4 SDO write Max motor speed Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*注意，在写0X607F前要先写0x6080, 且0x6080的值必须大于等于0x607F的值，不然写0x607F不成功*/
    /*Profile Max Velocity*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x607F;
    Sdo_W_Para.subIndex = 0;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MotorMaxVelocity;    /*设定伺服最高速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE1: Epos4 SDO write Max profile Velocity Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*Profile velocity*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6081;
    Sdo_W_Para.subIndex = 0;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    if(EposCtrl.u32ServoProfileVelocity > EposCtrl.u32MotorMaxVelocity)
    {
        EposCtrl.u32ServoProfileVelocity = EposCtrl.u32MotorMaxVelocity;
    }
    Sdo_W_Para.Data = EposCtrl.u32ServoProfileVelocity;  /*伺服设定速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE2: Epos4 SDO write profile Velocity Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*Motor Max Acceleration*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x60C5;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MaxAcceleration;    /*设定伺服最高加速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE0: Epos4 SDO write Max motor speed Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*Profile acceleration */
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6083;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MotorAcceleration;  /*伺服设定加速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE2: Epos4 SDO write Acceleration Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);

    if(CSP == mode)
    {
        /*Interpolation time period*/
        Sdo_W_Para.NodeId = 1;
        Sdo_W_Para.Index = 0x60C2;
        Sdo_W_Para.subIndex = 0x01;
        Sdo_W_Para.dataType = OD_DATATYPE_U8;
        Sdo_W_Para.Data = 10;
        if(SdoWriteProcess(&Sdo_W_Para))
        {
    #if(EPOSEMONITOR)
            printf("0xE0: Epos4 SDO write Max motor speed Fail!\r\n");
    #endif
            EposCtrl.enEpos4InitStatus = InitFail;
            return;
        }
        delay_ms(1);
    }
    
    /*Profile deceleration */
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6084;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MotorDeceleration;  /*伺服设定减速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE2: Epos4 SDO write Deceleration Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    
    /*Profile Quick stop deceleration*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6085;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Data = EposCtrl.u32MotorQSDeceleration;  /*伺服设定急停减速度*/
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("0xE2: Epos4 SDO write QSDeceleration Fail!\r\n");
#endif
        EposCtrl.enEpos4InitStatus = InitFail;
        return;
    }
    delay_ms(1);
    

    /*Shutdown*/
    EposShutdown();
    delay_ms(5);
    
    /*Switch on & Enable*/
    EposSwitchOn();
    delay_ms(5);
    
    EposCtrl.bEpos4InitFinished = TRUE;
    EposCtrl.enEpos4InitStatus = InitSuccess;
#if(EPOSEMONITOR)
    printf("0x00: Epos4 Control Para Init Success!\r\n");
#endif
}

/*EPOS 驱动器初始化*/
void Epos4ServoInit(ServoControl_t* servoCtlPara)
{
    EposLimitParaInit();

    ServoModeInit(PPMode);
}

/*更新伺服位置*/
uint8_t PPMMovetoNewPostion(int32_t NewPosition, uint32_t profileVelocity, ExecuteMode_en ExeMode)
{
    static uint32_t previousVelocity = 0;
    
    if(Shutdown == (EposCtrl.u16Statusword.u16AllStatus & 0x000F))
    {
        EposSwitchOn();
    }
    
    /*set new target position*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.dataType = OD_DATATYPE_I32;
    Sdo_W_Para.Index = 0x607A;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.Data = NewPosition;
//    return 0;
#if(EPOSEMONITOR)
    printf("write new position\t %d",NewPosition);
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
    printf("--fail\r\n");
    #endif
        return FAIL;
    }
#if(EPOSEMONITOR)
    printf("--succeed\r\n");
    #endif
    if(PPMode == EposCtrl.enEpos4OpMode)
    {
        if(previousVelocity != profileVelocity)
        {
            previousVelocity = profileVelocity;
            /*set new profile velocity*/
            ChangeProfileVelocity(profileVelocity);
        }
        
        /*write controlword*/
        Sdo_W_Para.NodeId = 1;
        Sdo_W_Para.Index = 0x6040;
        Sdo_W_Para.dataType = OD_DATATYPE_U16;
        EposCtrl.u16Controlword = ExeMode;  /*0x3F -- absolute position, start immediately*/
        Sdo_W_Para.Data = EposCtrl.u16Controlword;
#if(EPOSEMONITOR)
        printf("write control words:\t0x3F");
    #endif
        if(SdoWriteProcess(&Sdo_W_Para))
        {
#if(EPOSEMONITOR)
            printf("--fail\r\n");
        #endif
            return FAIL;
        }
#if(EPOSEMONITOR)
    printf("--succeed\r\n");
    #endif
        EposSwitchOn();
    }
    
    return SUCCEED;
}


/*电机急停*/
uint8_t EposQuickStop(void)
{
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6040;
    Sdo_W_Para.dataType = OD_DATATYPE_U16;
    Sdo_W_Para.Data = MOTORQUICKSTOP;  /*0x00B -- Quick stop*/
#if(EPOSEMONITOR)
        printf("write control words:\t quick stop");
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
    #endif
        return FAIL;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
    #endif
        return SUCCEED;
    }
}

/*电机停机*/
uint8_t EposShutdown(void)
{
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6040;
    Sdo_W_Para.dataType = OD_DATATYPE_U16;
    EposCtrl.u16Controlword = MOTORSHUTDOWN;  /*0x0006 -- Epose shutdown*/
    Sdo_W_Para.Data = EposCtrl.u16Controlword;
#if(EPOSEMONITOR)
    printf("write control word :shutdown\t");
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return FAIL;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
        #endif
        EposCtrl.enEposCurrStatus = Shutdown;
        return SUCCEED;
    }
}

/*电机使能*/
uint8_t EposSwitchOn(void)
{
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6040;
    Sdo_W_Para.dataType = OD_DATATYPE_U16;
    EposCtrl.u16Controlword = MOTORENABLE;  /*0x000F -- switch on & enable*/
    Sdo_W_Para.Data = EposCtrl.u16Controlword;
#if(EPOSEMONITOR)
    printf("write control word :switchon\t");
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return FAIL;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
        #endif
        EposCtrl.enEposCurrStatus = SwitchOn;
        return SUCCEED;
    }
}


/*电机保持当前位置不动*/
uint8_t EposHalt(void)
{
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.Index = 0x6040;
    Sdo_W_Para.dataType = OD_DATATYPE_U16;
    EposCtrl.u16Controlword = MOTORHALT;  /*0x00B -- 电机保持当前位置不动----立定*/
    Sdo_W_Para.Data = EposCtrl.u16Controlword;
#if(EPOSEMONITOR)
    printf("write control word :motor halt\t");
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return FAIL;    /*失败*/
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
        #endif
        return SUCCEED;    /*成功*/
    }
}

/** \brief PPM模式速度配置
 *  
 * \param
 * \param
 * \return 0
 *
 */
uint8_t ChangeProfileVelocity(uint32_t NewVelocity)
{
    /*set new profile velocity*/
    Sdo_W_Para.NodeId = 1;
    Sdo_W_Para.dataType = OD_DATATYPE_U32;
    Sdo_W_Para.Index = 0x6081;
    Sdo_W_Para.subIndex = 0x00;
    Sdo_W_Para.Data = NewVelocity;
#if(EPOSEMONITOR)
    printf("write new velocity :%d\t",NewVelocity);
    #endif
    if(SdoWriteProcess(&Sdo_W_Para))
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return FAIL;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
        #endif
        return SUCCEED;
    }
}

uint8_t MonitorEposError(void)
{
    Sdo_R_Para.NodeId = 1;
    Sdo_R_Para.dataType = OD_DATATYPE_U16;
    Sdo_R_Para.Index = 0x603F;
    Sdo_R_Para.subIndex = 0x00;
#if(EPOSEMONITOR)
    printf("read EPOS Error:\t");
    #endif
    if(CAN_READ_SUCCEED == SdoReadProcess(&Sdo_R_Para))
    {
#if(EPOSEMONITOR)
        printf("%d\r\n",Sdo_R_Para.Data);
        #endif
        EposCtrl.u16ErrorCode = Sdo_R_Para.Data;
        return CAN_READ_SUCCEED;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return CAN_READ_FAIL;
    }
}

uint8_t ClearServoFault(void)
{
    if(0 != EposCtrl.u16ErrorCode)
    {
        Sdo_W_Para.NodeId = 1;
        Sdo_W_Para.dataType = OD_DATATYPE_U16;
        Sdo_W_Para.Index = 0x6040;
        Sdo_W_Para.subIndex = 0x00;
        Sdo_W_Para.Data = CLEARFAULT;
#if(EPOSEMONITOR)
        printf("clear error:\t");
    #endif
        if(SdoWriteProcess(&Sdo_W_Para))
        {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
            return FAIL;
        }
        else
        {
#if(EPOSEMONITOR)
        printf("--succeed\r\n");
        #endif
            return SUCCEED;
        }
    }
}

uint16_t GetServoStatus(void)
{
    Sdo_R_Para.NodeId = 1;
    Sdo_R_Para.dataType = OD_DATATYPE_U16;
    Sdo_R_Para.Index = 0x6041;
    Sdo_R_Para.subIndex = 0x00;
#if(EPOSEMONITOR)
    printf("read EPOS Status:\t");
    #endif
    if(CAN_READ_SUCCEED == SdoReadProcess(&Sdo_R_Para))
    {
#if(EPOSEMONITOR)
        printf("%d\r\n",Sdo_R_Para.Data);
        #endif
        EposCtrl.u16Statusword.u16AllStatus = Sdo_R_Para.Data;
        return CAN_READ_SUCCEED;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return CAN_READ_FAIL;
    }
}


int32_t GetServoActualPosition(void)
{
    Sdo_R_Para.NodeId = 1;
    Sdo_R_Para.dataType = OD_DATATYPE_I32;
    Sdo_R_Para.Index = 0x6064;
    Sdo_R_Para.subIndex = 0x00;
#if(EPOSEMONITOR)
        printf("read actual position:\t");
    #endif
    if(SUCCEED == SdoReadProcess(&Sdo_R_Para))
    {
#if(EPOSEMONITOR)
        printf("%d\r\n",Sdo_R_Para.Data);
    #endif
        return Sdo_R_Para.Data;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        return 0;
    }
}


int16_t GetServoPowerSupply(void)
{
    Sdo_R_Para.NodeId = 1;
    Sdo_R_Para.dataType = OD_DATATYPE_U16;
    Sdo_R_Para.Index = 0x2200;
    Sdo_R_Para.subIndex = 0x01;
#if(EPOSEMONITOR)
    printf("read EPOS Power Supply:\t");
    #endif
    if(CAN_READ_SUCCEED == SdoReadProcess(&Sdo_R_Para))
    {
#if(EPOSEMONITOR)
        printf("%d\r\n",Sdo_R_Para.Data);
        #endif
        EposCtrl.i16PowerSupply = Sdo_R_Para.Data/10;
        return CAN_READ_SUCCEED;
    }
    else
    {
#if(EPOSEMONITOR)
        printf("--fail\r\n");
        #endif
        EposCtrl.i16PowerSupply = 0;
        return CAN_READ_FAIL;
    }
}



void PDOSendJointAngle(void)
{
	Message can_frame;
	can_frame.cob_id = 0x180 + SELF_NODE_ID;
	can_frame.rtr = 0;
	can_frame.len = 4;
    can_frame.data[0] = TestSlave_obj6067[SELF_NODE_ID - 3] & 0x000000FF;
    can_frame.data[1] = (TestSlave_obj6067[SELF_NODE_ID - 3] & 0x0000FF00) >> 8;
    can_frame.data[2] = (TestSlave_obj6067[SELF_NODE_ID - 3] & 0x00FF0000) >> 16;
    can_frame.data[3] = (TestSlave_obj6067[SELF_NODE_ID - 3] & 0xFF000000) >> 24;
    
    canSend(&TestSlave_Data, &can_frame);
}
