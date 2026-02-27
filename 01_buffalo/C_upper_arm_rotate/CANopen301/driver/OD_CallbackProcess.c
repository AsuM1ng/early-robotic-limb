#include "OD_CallbackProcess.h"
#include "read_write_SDO.h"
#include "objacces.h"
#include "TestSlave.h"
#include "CANopenMaster.h"
#include "servo_driver_control.h"
#include "global.h"

#include "servo_Control.h"
#include "JointControl.h"

//#define MAX_PVT_BUFFER 256


// CANopenMaster OD callback register
void CANopenMasterCallbackRegister(void)
{
	// receive change of the servo driver's statusword
	RegisterSetODentryCallBack(&CANopenMaster_Data, 0x603F0000, &servoErrorCodeCallback);             /*伺服控制字*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60410000, &servoStatuswordCallback);              /*伺服状态字*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60600000, &servoModesOperationCallback);          /*伺服模式设置*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60640000, &servoActualPositionChangeCallback);    /*伺服位置变化时的回调函数*/

    /*伺服位置变化*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x607A0000, &servoTargetPositionChangeCallback);    /*伺服目标位置变化*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60810000, &servoProfileVelocityChangeCallback);   /*伺服目标速度变化*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60830000, &servoProfileAccelerationCallback);     /*伺服加速度变化*/
    RegisterSetODentryCallBack(&CANopenMaster_Data, 0x60840000, &servoProfileDecelerationCallback);     /*伺服减速度变化*/
}


// CANopenSlave OD callback register
void CANopenSlaveCallbackRegister(void)
{
    RegisterSetODentryCallBack(&TestSlave_Data, 0x60010000, &OperationModeCallback);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x60030000, &NewAngleCallback);
    RegisterSetODentryCallBack(&TestSlave_Data, 0x60050000, &NewSpeedCallback);

    RegisterSetODentryCallBack(&TestSlave_Data, 0x18000000, &TPDO5Callback);

    for(int i = 0; i <= MAX_PVT_BUFFER; i++)
    {
        RegisterSetODentryCallBack(&TestSlave_Data, 0x60C10000+i, &interpolation_data_record_callback);
    }
}


uint16_t number_of_pvt_data = 0;
UNS32 interpolation_data_record_callback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
    number_of_pvt_data++;
	return 0;
}


/** \brief 运行模式回调函数：
 *          当主控通过CAN写新的运行模式命令时调用，用于切换节点板的工作模式
 * \param 
 * \param 
 * \return 0
 *
 */     
UNS32 OperationModeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)//主控传来了数据并且置位了驱动控制字的new_set_point
{
    if(JointCtrl.JointContrlMode != MasterCommand.enControlMode)
    {
        JointCtrl.JointPreviouMode = JointCtrl.JointContrlMode;
        JointCtrl.JointContrlMode = MasterCommand.enControlMode;
    }
    
	return 0;
}

/** \brief 位置设置回调函数：
 *          当主控通过SDO写新的位置信息到本节点对象0x6003时调用;
 * \param 
 * \param 
 * \return 0
 *
 */     
UNS32 NewAngleCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    JointCtrl.Motion.i32JointNewPosition = MasterCommand.i32CommTargetPosition;
    JointCtrl.Motion.i32JointNewAngle = MasterCommand.i32TargetAngle;
    return 0;
}

/** \brief 位置设置回调函数：
 *          当主控通过SDO写新的位置信息到本节点对象0x6003时调用;
 * \param 
 * \param 
 * \return 0
 *
 */     
UNS32 NewSpeedCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
    JointCtrl.Motion.u32ProfileSpeed = MasterCommand.i32CommProfileSpeed;
    return 0;
}


UNS32 TPDO5Callback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CANopenMaster OD callback
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// when servo driver's statusword is changed, the servo driver sends statusword to jointboard via TPDO, 
// the action jonitboard's receives the RPDO and mapping to OD(setODentry) will call the callback function,
// and then change the jointboard statusword according with the statusword from servo driver and the jointboard status.

/*0x603F callback function*/
UNS32 servoErrorCodeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
	//servo_driver_controlword_display;
    JointCtrl.u16ErrorCode = EposCtrl.u16ErrorCode;
    
    return 0;
}

/*0x6041 callback function*/
UNS32 servoStatuswordCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{

    return 0;
}

/*0x6060 callback function*/
UNS32 servoModesOperationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    if(WriteProcess == Sdo_W_Para.Sdo_Status)
//    {
//        SdoWriteProcess(&Sdo_W_Para);
//    }
    return 0;
}


/*0x6064 callback function*/
UNS32 servoActualPositionChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
    JointCtrl.Motion.i32JointActualPosition = EposCtrl.i32ServoActualPosition;

    return 0;
}

/*0x607A callback function*/
UNS32 servoTargetPositionChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    if(WriteProcess == Sdo_W_Para.Sdo_Status)
//    {
//        SdoWriteProcess(&Sdo_W_Para);
//    }
    return 0;
}

/*0x6081 callback function*/
UNS32 servoProfileVelocityChangeCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    if(WriteProcess == Sdo_W_Para.Sdo_Status)
//    {
//        SdoWriteProcess(&Sdo_W_Para);
//    }
    return 0;
}

/*0x6083 callback function*/
UNS32 servoProfileAccelerationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    if(WriteProcess == Sdo_W_Para.Sdo_Status)
//    {
//        SdoWriteProcess(&Sdo_W_Para);
//    }
    return 0;
}

/*0x6084 callback function*/
UNS32 servoProfileDecelerationCallback(CO_Data* d, const OD_OI_typeDef * unsused_indextable, UNS8 unsused_bSubindex)
{
//    if(WriteProcess == Sdo_W_Para.Sdo_Status)
//    {
//        SdoWriteProcess(&Sdo_W_Para);
//    }
    return 0;
}

