#include <stdlib.h>
#include <math.h>
#include "chipHAL_delay.h"
#include "chipHAL_TIM.h"
#include "JointControl.h"
#include "servo_Control.h"
#include "chipHAL_GPIO.h"
#include "read_write_SDO.h"
#include "ThreeAxisForceTransform.h"
#include "AngleSensor.h"
#include "global.h"
#include "MotionPathProcess.h"

#define PATH_TEST_BUFF_EN               0

Communication_t MasterCommand;
JointControl_t  JointCtrl;

JointFeature_t  ShoulderJoint_H_Para;       /*肩关节水平自由度*/
JointFeature_t  ShoulderJoint_V_Para;       /*肩关节垂直自由度*/
JointFeature_t  UpperarmRotatePara;         /*上臂旋转自由度*/
JointFeature_t  ElbowJointPara;             /*肘关节参数*/
JointFeature_t  ForearmRotatePara;          /*前臂旋转自由度*/
JointFeature_t  WristJointPara;             /*腕关节自由度*/



MotionIterm_t TestPataBuff[15] = {
{0, 0, 0.01},
{-1, -37, 1.01},
{-5264, 0, 2.21},
{-5266, 37, 2.70},
{6076, 37, 4.69},
{6076, -37,4.88},
{1139,-37,5.85},
{1137,37,6.11},
{6846,37,7.18},
{6845,-37,7.38},
{-4763,0,9.22},
{-4762,37,9.42},
{3291,0,10.92},
{3289,-37,11.04},
{-2262,0,12.01},
};


void EPOS_CSPTest(void);

/*EPOS 驱动器测试函数，使关节在规定范围内往复运动*/
void Joint_PPMTest(int32_t AbsPositionMax, int32_t AbsPositionMin);

/*关节在0点位置立定*/
void JointStandbyControl(void);

/*关节被动模式控制函数*/
void JointPassiveControl(void);

/*关节主动控制函数*/
void JointActiveControl(void);

///*关节主被动控制函数*/
//void JointActPassiveControl(void);

///*关节抗阻模式控制函数*/
//void JointImpedanceControl(void);

/*关节示教模式控制函数*/
void JointTeachingControl(void);

/*关节处方模式控制函数*/
void JointRecipeControl(void);

/*关节位置校准控制函数*/
void JointPositionCalibration(void);

/*关节保护*/
void ProtectProcess(void);

#if ('A' == JOINTBOARD_TYPE)    /*肩关节水平节点主动控制函数*/
int16_t ShoulderJoint_H_ActiveCtrl(int16_t Gravity);

#elif ('B' == JOINTBOARD_TYPE)  /*肩关节垂直节点主动控制函数*/
int16_t ShoulderJoint_V_ActiveCtrl(int16_t Gravity);

void MotorBreakProcess(void);

#elif ('C' == JOINTBOARD_TYPE)  /*上臂旋转主动模式控制函数*/
int16_t UpperarmRotate_ActiveCtrl(int16_t Gravity);

#elif ('D' == JOINTBOARD_TYPE)  /*肘关节主动模式触发条件判断*/
int16_t ElbowJointActiveCtrl(int16_t Gravity);

#elif ('E' == JOINTBOARD_TYPE)  /*前臂旋转主动模式触发条件判断*/

#elif ('F' == JOINTBOARD_TYPE)  /*腕部主动模式触发条件判断*/
int16_t WristJointActiveCtrl(int16_t Gravity);

#endif


void ForceSensorParaInit(void)
{
    HandMuscleForce.cpSensorName = "HandGripFroce";
    HandMuscleForce.i16Axis_v_ForceOffset = 0;          /*握力重力补偿 = 0*/

    /*手臂重力补偿，手臂质量按5kg计算，则手臂总重力为5 * 9.8 = 49N*/
    /*机械臂处于水平状态时，手臂重力对力传感器的作用力最大，且对不同的传感器作用力大小不同*/
    /*当机械臂处于垂直状态时，手臂重力对力传感器的作用力最小，可忽略*/
    WristForce.cpSensorName = "WristForce";
    WristForce.u16Axis_z_Range = 40;
    WristForce.i16Axis_v_ForceOffset = 10;              /*手腕重力补偿*//*具体补偿多少需要实际测试*/

    ForearmForce.cpSensorName = "ForearmForce";
    WristForce.i16Axis_v_ForceOffset = 10;              /*前臂重力补偿*//*具体补偿多少需要实际测试*/

    UpperarmForce.cpSensorName = "UpperarmForce";
    UpperarmForce.i16Axis_v_ForceOffset = 10;              /*手肘重力补偿*//*具体补偿多少需要实际测试*/
}



/** \brief 关节控制函数参数初始化
 *
 * \param
 * \param
 * \return 0
 *
 */
uint8_t JointControlParaInit(void)
{
    uint8_t errorCode = 0;
    static uint16_t u16ClearCount = 0;
#if(PRINTF_EN)
    printf("Joint Control para Init!\r\n");
#endif
    /*初始化力传感器参数*/
    ForceSensorParaInit();
    MonitorEposError();
    GetServoStatus();

    /*读取EPOS故障信息，并清除*/
    if((0 != EposCtrl.u16ErrorCode) || (EposCtrl.u16Statusword.Status_bit.b3_Fault))
    {
    #if(PRINTF_EN)
        printf("Servo with fault!\r\n");
        printf("Error Code: 0x%4X\r\n",EposCtrl.u16ErrorCode);
    #endif
        ClearServoFault();
        delay_ms(20);
        
        u16ClearCount += 1;
        if(u16ClearCount < 10)
        {
            return 0xFF;
        }
        else
        {
            #if(PRINTF_EN)
                printf("Clear fault fail!\r\n");
            #endif
        }
        return 0xFF;
    }
    else
    {
    #if(PRINTF_EN)
        printf("Servo Normal!\r\n");
    #endif    
    }


    EposSwitchOn();
    delay_ms(10);
    
    /*上电时，将电机停止在原位置*/
    EposHalt();
    
    /*上电初始化时，设置关节为待机模式*/
    /*关节运行模式的切换由主控发送命令，对象为0x6001*/
    JointCtrl.JointPreviouMode = StandbyMode;
    JointCtrl.JointContrlMode = StandbyMode;
    /*读取控制器编码器当前位置*/
    JointCtrl.u16JontControlCycle = JOINT_CTRL_CYCLE;  /*默认控制周期为10mS*/

    JointCtrl.Motion.i32JointActualPosition = GetServoActualPosition();
    JointCtrl.Motion.i32JointAngleOffset = JOINT_ANGLE_OFFSET;
    
#if (('A' == JOINTBOARD_TYPE)|| ('E' == JOINTBOARD_TYPE))
    /*读多圈编码器绝对位置，换算成关节角度*/
    JointCtrl.JointIncDir = Normal;
    JointCtrl.Motion.i32JointActualAngle = GetPositalActualAngle(JointCtrl.Motion.i32JointAngleOffset);
#elif ('C' == JOINTBOARD_TYPE)
    JointCtrl.Motion.i32JointActualAngle = CalcJointUperArmRotateAngle(JointCtrl.Motion.i32JointAngleOffset);
    JointCtrl.JointIncDir = Normal;
#elif (('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE))
    /*单绝对位置圈编码器，读取关节实际角度，上电第一次读取的值为0*/
    GetSingleTurnSensorAngle(0);
    JointCtrl.Motion.i32JointActualAngle = GetSingleTurnSensorAngle(JointCtrl.Motion.i32JointAngleOffset);
    
    #if ('B' == JOINTBOARD_TYPE)
    /*上电刹车不释放，初始化结束，电机使能后再释放刹车*/
    JointCtrl.Motion.BreakCtrl = Break;
    #endif
    
    #if ('D' == JOINTBOARD_TYPE)
    JointCtrl.JointIncDir = Inverse;
    #else
    JointCtrl.JointIncDir = Normal;
    #endif

    TestSlave_obj6067[SELF_NODE_ID - 3] = JointCtrl.Motion.i32JointActualAngle;
    
    #if ('F' == JOINTBOARD_TYPE)
    JointCtrl.JointIncDir = Inverse;
    JointCtrl.Motion.i16GravityOffset = 5;
    #endif
#endif

    delay_ms(10);
    /*关节位置极限值参数设定*/
    JointCtrl.Motion.i32JointAngleMax = (JOINT_ANGLE_MAX) * 100;
    JointCtrl.Motion.i32JointAngleMin = (JOINT_ANGLE_MIN) * 100;
    JointCtrl.Motion.i32JointStandbyAngle = JOINT_ANGLE_STANDBY * 100; /*关节待机角度*/

#if ('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE)
    /*关节相对位置极限值*/
    JointCtrl.Motion.i32JointAbsPositionMax = \
                    JointCtrl.Motion.i32JointActualPosition +\
                    ((JOINT_ANGLE_MAX - JOINT_ANGLE_STANDBY) * \
                    JOINT_GEAR_REDUCTION * (1 << ENCODER_BIT)/360);
                                                        
    JointCtrl.Motion.i32JointAbsPositionMin = \
                    JointCtrl.Motion.i32JointActualPosition +\
                    ((JOINT_ANGLE_MIN - JOINT_ANGLE_STANDBY) * \
                    JOINT_GEAR_REDUCTION * (1 << ENCODER_BIT)/360);
#endif
    JointCtrl.Motion.u32MotionSpeedMax = JOINT_VELOCITY_MAX;     /*电机机械频率30Hz*/
    JointCtrl.Motion.u32MotionSpeedMin = JOINT_VELOCITY_MIN;     /*电机机械频率2Hz*/
    JointCtrl.Motion.u32ProfileSpeed = JOINT_VELOCITY_DEF;       /*默认的电机机械频率为4Hz*/

    JointCtrl.Motion.u16MotionStepLengthMax = JOINT_STEPLENG_MAX;
    JointCtrl.Motion.u16MotionStepLengthMin = JOINT_STEPLENG_MIN;
    JointCtrl.Motion.u16Axis_h_Threshold = JOINT_TRIGGER_THRESHOLD;
    JointCtrl.Motion.u16Axis_v_Threshold = JOINT_TRIGGER_THRESHOLD + UpperarmForce.i16Axis_v_ForceOffset;
    JointCtrl.Motion.bMotionEnable = FALSE;
    
    EposCtrl.enEpos4OpMode = PPMode;
    EposCtrl.bEpos4InitFinished = FALSE;
    
    EposCtrl.i32SoftPositionLimitMax = JointCtrl.Motion.i32JointAbsPositionMax;
    EposCtrl.i32SoftPositionLimitMin = JointCtrl.Motion.i32JointAbsPositionMin;

    EposCtrl.u32ServoProfileVelocity = JointCtrl.Motion.u32ProfileSpeed;
    EposCtrl.u32MotorMaxVelocity = JointCtrl.Motion.u32MotionSpeedMax;
    EposCtrl.i32ServoActualPosition = JointCtrl.Motion.i32JointActualPosition;

    while(FALSE == EposCtrl.bEpos4InitFinished)
    {
        Epos4ServoInit(&EposCtrl);
    }

    JointCtrl.u16JointStatus.Status_bit.b0_InitFinished = TRUE;
    
    EposSwitchOn();
    #if(PRINTF_EN)
        printf("Servo Init Finish!\r\n");
    #endif
#if ('B' == JOINTBOARD_TYPE)
    JointCtrl.Motion.BreakCtrl = Release;
    #endif
    return errorCode;
}


/** \brief  关节控制函数
 *          被main函数调用，并由通用定时器中断周期使能，用于监听节点板各模块状态，控制节点板电机运动等功能
 *          使能周期暂定5mS
 * \param
 * \param
 * \return 0
 *
 */
void JointControlProcess(void)
{
#if ('B' == JOINTBOARD_TYPE) || ('D' == JOINTBOARD_TYPE) || ('F' == JOINTBOARD_TYPE)
    JointCtrl.Motion.i32JointActualAngle = GetSingleTurnSensorAngle(JointCtrl.Motion.i32JointAngleOffset);
#elif (('A' == JOINTBOARD_TYPE) || ('E' == JOINTBOARD_TYPE))
    JointCtrl.Motion.i32JointActualAngle = GetPositalActualAngle(JointCtrl.Motion.i32JointAngleOffset);
#elif ('C' == JOINTBOARD_TYPE)
    JointCtrl.Motion.i32JointActualAngle = CalcJointUperArmRotateAngle(JointCtrl.Motion.i32JointAngleOffset);
#endif
    
    TestSlave_obj6067[SELF_NODE_ID - 3] = JointCtrl.Motion.i32JointActualAngle;
    ForearmRotatePara.i32JointActualAngle = TestSlave_obj6067[Forearm_R];
    UpperarmRotatePara.i32JointActualAngle = TestSlave_obj6067[UpperArm_R];

    /*设备和人员保护做在最前面，触发了保护直接停机，不执行其它动作*/

//    ProtectProcess();
#if ('B' == JOINTBOARD_TYPE)
//    MotorBreakProcess();
    #endif
    
    if(!EposCtrl.u16Statusword.Status_bit.b1_SwitchOn)
    {
        EposSwitchOn();
    }

		
//		JointCtrl.JointPreviouMode = StandbyMode;
//    JointCtrl.JointContrlMode = TeachingMode;
		
    switch(JointCtrl.JointContrlMode)
    {
        case StandbyMode:       /*待机模式*/
            /*进入待机模式时，如果控制器没有工作在PPM模式，则重新初始化控制器为PPM*/
            if(PPMode != EposCtrl.enEpos4OpMode)
            {
                EposCtrl.enEpos4OpMode = PPMode;
                JointCtrl.synPdoEnable = FALSE;
                ServoModeInit(EposCtrl.enEpos4OpMode);
            }
            JointStandbyControl();
        break;

        case PassiveMode:       /*被动模式*/
            JointPassiveControl();
        break;

        case ActiveMode:        /*主动模式*/
        #if('E' == JOINTBOARD_TYPE)
            JointCtrl.JointContrlMode = StandbyMode;
        #else
            JointActiveControl();
        #endif
        break;



        case TeachingMode:      /*示教模式*/
        #if('E' == JOINTBOARD_TYPE)
            JointCtrl.JointContrlMode = StandbyMode;
        #else
            JointTeachingControl();
        #endif
        break;

        case RecipeMode:        /*处方模式*/
        #if('E' == JOINTBOARD_TYPE)
            JointCtrl.JointContrlMode = StandbyMode;
        #else
            JointRecipeControl();
        #endif
        break;

        case PositionCali:
            JointPositionCalibration(); /*位置校准*/
        break;

        case MotorTest:
            Joint_PPMTest(JointCtrl.Motion.i32JointAbsPositionMax,
                            JointCtrl.Motion.i32JointAbsPositionMin);
        break;

        case AppShutdown:
            EposShutdown();
        break;

        case AppSwitchOn:
            EposSwitchOn();
            break;
        
        case ClearError:        /*清除故障*/
            ClearServoFault();
            break;
        
        default :
            JointCtrl.JointContrlMode = StandbyMode;
            break;
    }
}


/*
关节测试函数，驱动关节在设定范围内往复运动，
关节运动的两个端头位置由函数调用的地方指定。
如果两个端头都能到达，测试结束时，程序自动进入standby模式。
*/
void Joint_PPMTest(int32_t AbsPositionMax, int32_t AbsPositionMin)
{
    static uint16_t DivCount = 0;   /*延时计数，延时的目的是不要频繁的发送CANOPEN命令，浪费CAN带宽*/
    static uint16_t TestStatus = 0;
    int32_t i32PositionTemp = 0;
    
    if(AbsPositionMax < AbsPositionMin)
    {
        i32PositionTemp = AbsPositionMax;
        AbsPositionMax = AbsPositionMin;
        AbsPositionMin = i32PositionTemp;
    }
    
    if(JointCtrl.u16JointStatus.Status_bit.b0_InitFinished)   /*初始化完成标志*/
    {
        if(0x0003 != TestStatus)
        {
            if(clockwise == EposCtrl.enDirection)
            {
                if(DivCount >= 10)  /*100mS发送一次目标位置*/
                {
                    DivCount = 0;
                    JointCtrl.Motion.i32JointTargetPosition = AbsPositionMax;
//                    PositionLimit(&JointCtrl.Motion.i32JointTargetPosition,
//                                    JointCtrl.Motion.i32JointAbsPositionMax,
//                                    JointCtrl.Motion.i32JointAbsPositionMin);
                    PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition,
                                        JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
                }
                else
                {
                    DivCount ++;
                }

                if((EposCtrl.u16Statusword.Status_bit.b10_TargetReach) &&
                    (JointCtrl.Motion.i32JointActualPosition >= (AbsPositionMax - (1 << ENCODER_BIT))))
                {
                    DivCount = 0;
                    EposCtrl.enDirection = anticlockwise;
                    JointCtrl.Motion.u16ActionTime = 0;
                    EposCtrl.u16Statusword.Status_bit.b10_TargetReach = FALSE;
                    TestStatus |= 0x0001;
                }
                else
                {
                    JointCtrl.Motion.u16ActionTime ++;
                    if(JointCtrl.Motion.u16ActionTime > 2000)   /*overtime 20S*/
                    {
                        JointCtrl.u16ErrorCode = 0xFA;  /*关节故障，不能到达目标位置*/
                        JointCtrl.u16JointStatus.Status_bit.b1_TestFinished = FALSE;
                    }
                }
            }
            else
            {
                if(DivCount >= 10)  /*100mS发送一次目标位置*/
                {
                    DivCount = 0;
                    JointCtrl.Motion.i32JointTargetPosition = AbsPositionMin;
//                    PositionLimit(&JointCtrl.Motion.i32JointTargetPosition,
//                                    JointCtrl.Motion.i32JointAbsPositionMax,
//                                    JointCtrl.Motion.i32JointAbsPositionMin);
                    PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition,
                                        JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
                }
                else
                {
                    DivCount ++;
                }

                if((EposCtrl.u16Statusword.Status_bit.b10_TargetReach) &&
                    (JointCtrl.Motion.i32JointActualPosition <= (AbsPositionMin + (1 << ENCODER_BIT))))
                {
                    DivCount = 0;
                    EposCtrl.enDirection = clockwise;
                    JointCtrl.Motion.u16ActionTime = 0;
                    EposCtrl.u16Statusword.Status_bit.b10_TargetReach = FALSE;
                    TestStatus |= 0x0002;
                }
                else
                {
                    JointCtrl.Motion.u16ActionTime ++;
                    if(JointCtrl.Motion.u16ActionTime > 2000)   /*overtime 20S*/
                    {
                        JointCtrl.u16ErrorCode = 0xFA;  /*关节故障，不能到达目标位置*/
                        JointCtrl.u16JointStatus.Status_bit.b1_TestFinished = FALSE;
                    }
                }
            }
            
            if(0x0003 == (0x0003 & TestStatus))
            {
                TestStatus = 0x0000;
                JointCtrl.Motion.u16ActionTime = 0;
                JointCtrl.u16ErrorCode &= !0xFA;
                JointCtrl.u16JointStatus.Status_bit.b1_TestFinished = TRUE;

                /*测试结束，回到 Standby 模式*/
                JointCtrl.JointContrlMode = StandbyMode;
            }
        }
    }
}


/** \brief 关节待机控制模式
 *          待机模式下，控制节点电机运行到待机位置，0点位置设置为待机位置
 *          横梁电机可设置为整个运动范围的中点？
 * \param
 * \param
 * \return 0
 *
 */
void JointStandbyControl(void)
{
    int32_t i32TempPosition = 0;
    /*如果目标位置不为0，或者实际位置与0点位置偏差大于100个点，则向控制器写0位置*/
    if((JointCtrl.Motion.i32JointActualAngle < (JointCtrl.Motion.i32JointStandbyAngle - 150)) ||\
        (JointCtrl.Motion.i32JointActualAngle > (JointCtrl.Motion.i32JointStandbyAngle + 150)))
    {
        /*此处有待优化，先求出从当前位置转动零点位置的最优路线(因为理论上到达零点位置可以正传也可以反转，但实际关节有限位)，再发送目标位置给控制器*/
        i32TempPosition = JointCtrl.JointIncDir * AngleConvertToAbsPosition(JointCtrl.Motion.i32JointActualAngle,
                                    JointCtrl.Motion.i32JointStandbyAngle,
                                    ENCODER_BIT, JOINT_GEAR_REDUCTION);

        JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointActualPosition + i32TempPosition;
//        return;
        PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition,
                            JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
    }
    else
    {
        /*动态修正关节极限值*/
        JointCtrl.Motion.i32JointAbsPositionMax = (JointCtrl.Motion.i32JointActualPosition +\
                    (JointCtrl.JointIncDir * (((JOINT_ANGLE_MAX - JOINT_ANGLE_STANDBY) * \
                    JOINT_GEAR_REDUCTION * (1 << ENCODER_BIT)/360))));

        JointCtrl.Motion.i32JointAbsPositionMin = (JointCtrl.Motion.i32JointActualPosition +\
                    (JointCtrl.JointIncDir * (((JOINT_ANGLE_MIN - JOINT_ANGLE_STANDBY) * \
                    JOINT_GEAR_REDUCTION * (1 << ENCODER_BIT)/360))));

        EposCtrl.i32SoftPositionLimitMax = JointCtrl.Motion.i32JointAbsPositionMax;
        EposCtrl.i32SoftPositionLimitMin = JointCtrl.Motion.i32JointAbsPositionMin;
        

//        return;
        EposHalt();
    }
}

/** 
 * \param
 * \param
 * \return 0
 *
 */
void JointPassiveControl(void)
{
    int32_t i32TempPosition = 0;
    /*位置有更新，一度的误差*/
    if(JointCtrl.Motion.i32JointNewAngle > (JointCtrl.Motion.i32JointActualAngle + 100) ||
        (JointCtrl.Motion.i32JointNewAngle < (JointCtrl.Motion.i32JointActualAngle - 100)))
    {
        if((JointCtrl.Motion.i32JointNewAngle > JointCtrl.Motion.i32JointAngleMax) || 
            (JointCtrl.Motion.i32JointNewAngle < JointCtrl.Motion.i32JointAngleMin))
        {
            JointCtrl.Motion.i32JointNewAngle = JointCtrl.Motion.i32JointActualAngle;
            EposHalt();
            return; /*位置非法*/
        }
        else
        {
            if(JointCtrl.Motion.i32JointPreAngle != JointCtrl.Motion.i32JointNewAngle)
            {
                JointCtrl.Motion.i32JointPreAngle = JointCtrl.Motion.i32JointNewAngle;
                /*将原始角度数据转换为控制器目标位置*/
                i32TempPosition = JointCtrl.JointIncDir * AngleConvertToAbsPosition(JointCtrl.Motion.i32JointActualAngle,
                                                        JointCtrl.Motion.i32JointNewAngle,
                                                        ENCODER_BIT, JOINT_GEAR_REDUCTION);
                JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointActualPosition + i32TempPosition;
                
                PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition, JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
                EposCtrl.u16Statusword.Status_bit.b10_TargetReach = FALSE;
            }
        }
    }

    /*速度限定*/
    if(JointCtrl.Motion.u32ProfileSpeed < JointCtrl.Motion.u32MotionSpeedMin)
    {
        JointCtrl.Motion.u32ProfileSpeed = JointCtrl.Motion.u32MotionSpeedMin;
    }
    else if(JointCtrl.Motion.u32ProfileSpeed > JointCtrl.Motion.u32MotionSpeedMax)
    {
        JointCtrl.Motion.u32ProfileSpeed = JointCtrl.Motion.u32MotionSpeedMax;
    }

    if(!EposCtrl.u16Statusword.Status_bit.b10_TargetReach)
    {
        PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition, JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
    }
    else    /*到达目标位置，电机立定*/
    {
        EposHalt();
    }
}

/** \brief  
 * \param
 * \param
 * \return 0
 *
 */
void JointActiveControl(void)
{
    static uint16_t lu16MotionCount = 0;
    static uint16_t lu16MotionDisCount = 0;

#if ('A' == JOINTBOARD_TYPE)    /*肩关节水平*/
    
#if(HANDTYPE == RIGHTHAND)
    JointCtrl.Motion.i16TriggerResuForce = -ShoulderJoint_H_ActiveCtrl(0);
    #else
    JointCtrl.Motion.i16TriggerResuForce = ShoulderJoint_H_ActiveCtrl(0);
#endif
    
#elif ('B' == JOINTBOARD_TYPE) /*肩关节垂直*/
    
    JointCtrl.Motion.i16TriggerResuForce = ShoulderJoint_V_ActiveCtrl(0);
    
#elif ('C' == JOINTBOARD_TYPE) /*上臂旋转*/
    
    JointCtrl.Motion.i16TriggerResuForce = UpperarmRotate_ActiveCtrl(0);
    
#elif ('D' == JOINTBOARD_TYPE) /*肘关节*/
    
    JointCtrl.Motion.i16TriggerResuForce = ElbowJointActiveCtrl(0);
    


#elif ('F' == JOINTBOARD_TYPE) /*腕关节*/
    
    JointCtrl.Motion.i16TriggerResuForce = WristJointActiveCtrl(JointCtrl.Motion.i16GravityOffset);
    
#endif

    if(abs(JointCtrl.Motion.i16TriggerResuForce) > JointCtrl.Motion.u16Axis_h_Threshold)
    {
        lu16MotionCount += 1;
        lu16MotionDisCount = 0;
        if(lu16MotionCount >= 10)/*如果水平方向的力持续100mS大于运动门限值，则将运动使能*/
        {
            lu16MotionCount = 0;
            JointCtrl.Motion.bMotionEnable = TRUE;
        }
    }
    else
    {
        lu16MotionDisCount += 1;
        /*只要出现一次力小于于门限，则将运动使能计数清零*/
        lu16MotionCount = 0;
        /*20*5mS 100mS消抖, 取100mS是否合适需要根据传感器信号特征进行验证*/
        if(lu16MotionDisCount >= 5)
        {
            /*如果水平方向的力持续100mS小于运动门限值，则将运动禁止*/
            lu16MotionDisCount = 0;
            JointCtrl.Motion.bMotionEnable = FALSE;
        }
    }
    
    if(JointCtrl.Motion.bMotionEnable)
    {
        /*根据力的大小，计算目标位置增量，位置增量与力的大小成正比*/
        JointCtrl.Motion.u16MotionLength = JointCtrl.Motion.u16MotionStepLengthMin +
                                                        (((abs(JointCtrl.Motion.i16TriggerResuForce) - JointCtrl.Motion.u16Axis_h_Threshold) *
                                                        (JointCtrl.Motion.u16MotionStepLengthMax - JointCtrl.Motion.u16MotionStepLengthMin))/
                                                        (WristForce.u16Axis_z_Range - JointCtrl.Motion.u16Axis_h_Threshold));

        /*根据力的大小，计算电机转速，转速于力的大小成正比*/
        if(abs(JointCtrl.Motion.i16TriggerResuForce) > JointCtrl.Motion.u16Axis_h_Threshold)
        {
            JointCtrl.Motion.u32ProfileSpeed = JointCtrl.Motion.u32MotionSpeedMin +
                                                        (((abs(JointCtrl.Motion.i16TriggerResuForce) - JointCtrl.Motion.u16Axis_h_Threshold) *
                                                        (JointCtrl.Motion.u32MotionSpeedMax - JointCtrl.Motion.u32MotionSpeedMin))/
                                                        (WristForce.u16Axis_z_Range - JointCtrl.Motion.u16Axis_h_Threshold));
        }
        else
        {
            JointCtrl.Motion.u32ProfileSpeed = 0;
        }

        /*根据力的方向，确定电机目标位置的方向，力 > 0 递增, 力 < 0 递减*/
        if(JointCtrl.Motion.i16TriggerResuForce > 0)
        {
            JointCtrl.Motion.enMotonDirection = clockwise;
            JointCtrl.Motion.i32JointTargetPosition += JointCtrl.Motion.u16MotionLength;
            
            /*位置限定，快接近极限位置时，不继续增加*/
            if(JointCtrl.Motion.i32JointTargetPosition > (JointCtrl.Motion.i32JointAbsPositionMax - 2048))
            {
                JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointAbsPositionMax - 2048;
            }
        }
        else
        {
            JointCtrl.Motion.enMotonDirection = anticlockwise;
            JointCtrl.Motion.i32JointTargetPosition -= JointCtrl.Motion.u16MotionLength;
            /*位置限定，快接近极限位置时，不继续减小*/
            if(JointCtrl.Motion.i32JointTargetPosition < (JointCtrl.Motion.i32JointAbsPositionMin + 4096))
            {
                JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointAbsPositionMin + 4096;
            }
        }
        
        /*软件限位，如果实际角度 > 最大角度或实际位置 < 最小位置时，电机停止*/
//        if(JointCtrl.Motion.i32JointActualAngle > JointCtrl.Motion.i32JointAngleMax)
//        {
//            JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointAbsPositionMax - 4096;
//        }
//        else if(JointCtrl.Motion.i32JointActualAngle < JointCtrl.Motion.i32JointAngleMin)
//        {
//            JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointAbsPositionMin + 4096;
//        }
        PositionLimit(&JointCtrl.Motion.i32JointTargetPosition,
                    JointCtrl.Motion.i32JointAbsPositionMax,
                    JointCtrl.Motion.i32JointAbsPositionMin);
//        return;
        PPMMovetoNewPostion(JointCtrl.Motion.i32JointTargetPosition, JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
    }
    else
    {
        /*电机停转，立定，注意！不是关机(disable)*/
        JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointActualPosition;
        EposHalt();
    }
}



static void ClearPVTBuff(void)
{
    JointCtrl.synPdoEnable = FALSE;
    JointCtrl.MotionPath.bPathRecordFinish = FALSE;
    JointCtrl.MotionPath.bReturnPathInitFlag = FALSE;
    
    JointCtrl.MotionPath.bPathRecordEn = TRUE;
    JointCtrl.MotionPath.bReappearStart = FALSE;

    memset(JointCtrl.MotionPath.PathBuff, 0, sizeof(JointCtrl.MotionPath.PathBuff));
    memset(JointCtrl.PVT_struct.pvt_table_struct, 0, sizeof(JointCtrl.PVT_struct.pvt_table_struct));

    JointCtrl.MotionPath.u32TimeCount = 0;
    JointCtrl.MotionPath.u16TimeDelay = 0;

    JointCtrl.MotionPath.u16PathIndex = 0;
    JointCtrl.MotionPath.u16TotalPoint = 0;
    JointCtrl.MotionPath.u32HaltDelay = HALTDELAY/JointCtrl.u16JontControlCycle;/*示教结束，电机的停止缓冲时间*/
    
    JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty = TRUE;
}
/** \brief 关节示教控制模式
 *  示教模式状态机：
 *
 * \param
 * \param
 * \return 0
 *
 */
void JointTeachingControl(void)
{
    /*示教模式下采集示教路线时，需要将电机切换为PPM模式*/
    if(!JointCtrl.MotionPath.bPathRecordFinish)
    {
        if(PPMode != EposCtrl.enEpos4OpMode)
        {
            EposCtrl.enEpos4OpMode = PPMode;
            ServoModeInit(EposCtrl.enEpos4OpMode);
            JointCtrl.synPdoEnable = FALSE;
        }
        JointActiveControl();
    }
    
    switch(JointCtrl.MotionPath.u16CtrlWowrds)
    {
        /*清除缓存*/
        case eClearBuff:
            ClearPVTBuff();
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        /*开始记录*/
        case eRecordStart:
            
            if(!JointCtrl.MotionPath.u16StatusWords.Status_bit.b0_RecordStart)
            {
                ClearPVTBuff();
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b0_RecordStart = TRUE;
                
                /*主控发送了开始记录的命令后，节点板循环判断路径记录是否完毕，并调用主动控制函数控制关节运动*/
                JointCtrl.MotionPath.bPathRecordEn = TRUE;
                JointCtrl.MotionPath.bFirstPoint = TRUE;
                
                JointCtrl.MotionPath.u16PathIndex = 0;
                JointCtrl.MotionPath.u32TimeCount = 0;
                JointCtrl.MotionPath.u16TotalPoint = 0;
                JointCtrl.PVT_struct.u16P_index = 0;

                JointCtrl.MotionPath.bReappearStart = FALSE;
            }
            
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
            /*停止记录*/
        case eRecordStop:
            if(0 != JointCtrl.MotionPath.u32HaltDelay)
            {
                /*电机保持原位置停止*/
                EposHalt();
                JointCtrl.MotionPath.u32HaltDelay--;
                break;
            }
            JointCtrl.MotionPath.bPathRecordEn = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b0_RecordStart = FALSE;
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        /*路径恢复*/
        case eReappearStart:
            if(!JointCtrl.MotionPath.bReappearStart)
            {
                /*开始恢复示教路线，*/
                if((TRUE == JointCtrl.MotionPath.bPathRecordFinish)
                    &&(FALSE == JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError)
                    &&(FALSE == JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty))
                {
                    /*路径恢复时，需要将电机切换为CSP模式*/
                    if(CSP != EposCtrl.enEpos4OpMode)
                    {
                        EposCtrl.enEpos4OpMode = CSP;
                        ServoModeInit(EposCtrl.enEpos4OpMode);
                    }
                    MotionPathSwitch();
                    JointCtrl.PVT_struct.pauseNext = FALSE;
                    JointCtrl.PVT_Back_struct.pauseNext = FALSE;
                }
                else
                {
                    JointCtrl.PVT_struct.start = FALSE;
                    JointCtrl.PVT_Back_struct.start = FALSE;
                    JointCtrl.MotionPath.bReappearStart = FALSE;
                    JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = FALSE;
                    JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = FALSE;
                }
            }

            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        /*停止路径恢复*/
        case eReappearStop:
            /*主控发送路径恢复停止命令时，电机不能马上停止，*/
            /*如果收到停止命令的时刻，电机正在重现示教路径，则电机停止最近的一个PVT速度零点*/
            if(JointCtrl.PVT_Back_struct.start &&
                (0 == JointCtrl.MotionPath.u32HaltDelay) &&
                (JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus))
            {
                JointCtrl.PVT_Back_struct.pauseNext = TRUE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.PVT_Back_struct.i32PVT_P_Target;
            }
            /*如果收到停止命令的时刻，电机正在归零，则电机停止最近的一个PVT速度零点*/
            else if(JointCtrl.PVT_struct.start &&
                (0 == JointCtrl.MotionPath.u32HaltDelay) &&
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus)
            {
                JointCtrl.PVT_struct.pauseNext = TRUE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.PVT_struct.i32PVT_P_Target;
            }
            /*其它情况下，电机的目标位置为当前电机的实际位置*/
            else
            {
                JointCtrl.PVT_struct.pauseNext = TRUE;
                JointCtrl.PVT_Back_struct.pauseNext = TRUE;
                JointCtrl.MotionPath.bReappearStart = FALSE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.Motion.i32JointActualPosition;
            }
            /*回归路线初始化标志清零*/
            JointCtrl.MotionPath.bReturnPathInitFlag = FALSE;
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        case eNoop:
        default:
            break;
    }
}


/** \brief 关节处方控制模式
 *
 * \param
 * \param
 * \return 0
 *
 */
void JointRecipeControl(void)
{
    switch(JointCtrl.MotionPath.u16CtrlWowrds)
    {
        /*主控命令情况缓存*/
        case eClearBuff:
        #if(PRINTF_EN)
            printf("Clear Buff\r\n");
        #endif
            memset(JointCtrl.MotionPath.PathBuff, 0, sizeof(JointCtrl.MotionPath.PathBuff));
            memset(JointCtrl.PVT_struct.pvt_table_struct, 0, sizeof(JointCtrl.PVT_struct.pvt_table_struct));
            JointCtrl.MotionPath.u16PathIndex = 0;
            JointCtrl.MotionPath.u16TotalPoint = 0;
            JointCtrl.MotionPath.bRestorePathInitFlag = FALSE;
            JointCtrl.MotionPath.bReturnPathInitFlag = FALSE;
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        /*主控命令恢复处方路线开始*/
        case eReappearStart:
            if(!JointCtrl.MotionPath.bReappearStart)
            {
            #if(PRINTF_EN)
                printf("Start, num = %d\r\n",JointCtrl.MotionPath.u16TotalPoint);
            #endif
#if(PATH_TEST_BUFF_EN)
                JointCtrl.MotionPath.bRestorePathInitFlag = 0;
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty = FALSE;
                JointCtrl.MotionPath.u16TotalPoint = 15;
                #endif
                if(!JointCtrl.MotionPath.bRestorePathInitFlag)
                {
                    if((JointCtrl.MotionPath.u16TotalPoint >= 2) && 
                        (TRUE == JointCtrl.MotionPath.u16BuffFillFinish))
                    {
                        for(uint16_t index = 0; index < JointCtrl.MotionPath.u16TotalPoint; index++)
                        {
#if(PATH_TEST_BUFF_EN)
                            JointCtrl.PVT_struct.pvt_table_struct[index].position = JointCtrl.Motion.i32JointStanbyPosi +\
                                                AngleConvertToAbsPosition(JointCtrl.Motion.i32JointStandbyAngle,
                                                                        TestPataBuff[index].i32Angle, ENCODER_BIT, JOINT_GEAR_REDUCTION);
                            JointCtrl.PVT_struct.pvt_table_struct[index].velocity = TestPataBuff[index].i32Velocity;
                            JointCtrl.PVT_struct.pvt_table_struct[index].time = TestPataBuff[index].fTime;
                        #else
                            JointCtrl.PVT_struct.pvt_table_struct[index].position = JointCtrl.Motion.i32JointStanbyPosi + \
                                                        AngleConvertToAbsPosition(JointCtrl.Motion.i32JointStandbyAngle,
                                                        JointCtrl.MotionPath.PathBuff[index].i32Angle, ENCODER_BIT, JOINT_GEAR_REDUCTION);
                            JointCtrl.PVT_struct.pvt_table_struct[index].velocity = JointCtrl.MotionPath.PathBuff[index].i32Velocity;
                            JointCtrl.PVT_struct.pvt_table_struct[index].time = JointCtrl.MotionPath.PathBuff[index].fTime;
                        #endif
                        }
                        JointCtrl.MotionPath.u16BuffFillFinish = 0;
                        JointCtrl.MotionPath.bRestorePathInitFlag = TRUE;
                        JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError = FALSE;
                        JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty = FALSE;
                        PVT_MotionPathInit(&JointCtrl.PVT_struct, 0.01 ,0,0,JointCtrl.MotionPath.u16TotalPoint - 1);
                    }
                    else
                    {
                        JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError = TRUE;
                    }
                }
                
                if((TRUE == JointCtrl.MotionPath.bRestorePathInitFlag)
                    &&(FALSE == JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError)
                    &&(FALSE == JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty))
                {
                    if(CSP != EposCtrl.enEpos4OpMode)   /*如果电机不是在CSP模式，将电机重新初始化*/
                    {
                        EposCtrl.enEpos4OpMode = CSP;
                        ServoModeInit(EposCtrl.enEpos4OpMode);
                    }
                    
                    MotionPathSwitch();
                    JointCtrl.PVT_struct.pauseNext = FALSE;
                    JointCtrl.PVT_Back_struct.pauseNext = FALSE;
                }
                else
                {
                    JointCtrl.PVT_struct.start = FALSE;
                    JointCtrl.PVT_Back_struct.start = FALSE;
                    JointCtrl.MotionPath.bReappearStart = FALSE;
                    JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = FALSE;
                    JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = FALSE;
                }
            }
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
            
        case eReappearStop:
        #if(PRINTF_EN)
            printf("Stop\r\n");
        #endif
            /*主控发送路径恢复停止命令时，电机不能马上停止，*/
            /*如果收到停止命令的时刻，电机正在重现示教路径，则电机停止最近的一个PVT速度零点*/
            if(JointCtrl.PVT_Back_struct.start &&
                (0 == JointCtrl.MotionPath.u32HaltDelay) &&
                (JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus))
            {
                JointCtrl.PVT_Back_struct.pauseNext = TRUE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.PVT_Back_struct.i32PVT_P_Target;
            }
            /*如果收到停止命令的时刻，电机正在归零，则电机停止最近的一个PVT速度零点*/
            else if(JointCtrl.PVT_struct.start &&
                (0 == JointCtrl.MotionPath.u32HaltDelay) &&
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus)
            {
                JointCtrl.PVT_struct.pauseNext = TRUE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.PVT_struct.i32PVT_P_Target;
            }
            /*其它情况下，电机的目标位置为当前电机的实际位置*/
            else
            {
                JointCtrl.PVT_struct.pauseNext = TRUE;
                JointCtrl.PVT_Back_struct.pauseNext = TRUE;
                JointCtrl.MotionPath.bReappearStart = FALSE;
                JointCtrl.Motion.i32JointHaltPosi = JointCtrl.Motion.i32JointActualPosition;
            }
            /*回归路线初始化标志清零*/
            JointCtrl.MotionPath.bReturnPathInitFlag = FALSE;
            JointCtrl.MotionPath.u16CtrlWowrds = eNoop;
        break;
        
        case eNoop:
        default:
            break;
    }
}

/** \brief 位置校准
 *
 * \param
 * \param
 * \return 0
 *
 */
void JointPositionCalibration(void)
{
    if(0x0000 == (EposCtrl.u16Statusword.u16AllStatus & 0x000F))
    {
        EposSwitchOn();
    }

    PPMMovetoNewPostion(JointCtrl.Motion.i32JointCaliTarget, JointCtrl.Motion.u32ProfileSpeed, Abs_Position_Imm);
}


/** \brief LED指示灯控制
 *
 * \param
 * \param
 * \return 0
 *
 */
void LEDTwinkleProcess(_Bool OnOff)
{
    /*运行灯*/
    if(OnOff)
        GPIO_SetBits(LED_GPIO_PORT, LED_G);
    else
        GPIO_ResetBits(LED_GPIO_PORT, LED_G);

    /*告警灯*/
    if(0 != JointCtrl.u16ErrorCode)
    {
        GPIO_SetBits(LED_GPIO_PORT, LED_R);
    }
    else
    {
        GPIO_ResetBits(LED_GPIO_PORT, LED_R);
    }
}


/** \brief 根据绝对位置编码器的脉冲编码计算实际角度(0~360度)
 * \param int32_t standbyPosition: 电机待机零点角度的脉冲编码值
 * \param int32_t currentPosition: 编码器脉冲编码原始值
 * \param uint16_t SingleturnResolution：编码器单圈分辨率
 * \param uint16_t gearReduction：减速器减速比
 * \return int32_t 返回的是关节实际角度乘以100，相当于double型数据保留两位小数
 *
 */
int32_t encoderPulseConvertToAngle(int32_t standbyPosition,
                                    int32_t currentPosition,
                                    uint16_t SingleturnResolution,
                                    uint16_t gearReduction)
{
    int32_t angle = 0;
    int64_t tempData1 = 0;
    int64_t tempData2 = 0;
    int64_t tempData3 = 0;

    tempData1 = (int64_t)(1 << SingleturnResolution);
    tempData2 = (int64_t)gearReduction;
    tempData3 = (int64_t)(currentPosition - standbyPosition);
    tempData3 = tempData3 * 36000;
    /*乘以36000 = 360 * 100， 目的是将角度放大100倍，计算的结果相当于保留了两位小数*/
    tempData3 = tempData3 / (tempData1 * tempData2);

    if(tempData3 > 0)
    {
        if(tempData3 > 36000)
        {
            angle = (int32_t)(tempData3 % 36000);
        }
        else
        {
            angle = (int32_t)tempData3;
        }
    }
    else
    {
        if(tempData3 < -36000)
        {
            angle = (int32_t)(tempData3 % -36000);
        }
        else
        {
            angle = (int32_t)tempData3;
        }
    }

    return angle;
}




/** \brief 根据关节目标角度，计算控制器目标位置
 *
 * \param int16_t actualAngle   关节实际角度
 * \param int16_t targetAngle   关节目标角度
 * \param uint16_t SingleturnResolution     电机增量编码器单圈分辨率
 * \param uint16_t gearReduction            关节减速器减速比
 * \return int32_t 控制器绝对目标位置
 *
 */
int32_t AngleConvertToAbsPosition(int16_t i16ActualAngle,
                                int16_t i16TargetAngle,
                                uint16_t u16SingleturnResolution,
                                uint16_t u16GearReduction)
{
//    int16_t i16Temp = 0;
    int32_t i32AbsPosition;
    int64_t i64AbsPosition = 0;
    int64_t i64TempPosition = 0;

    i64AbsPosition = i16TargetAngle - i16ActualAngle;

    i64AbsPosition = (i64AbsPosition * u16GearReduction)/100;   /*角度误差，结果为电机轴需要旋转的角度*/

    i64TempPosition = ((1 << u16SingleturnResolution) * i64AbsPosition)/360;

    i32AbsPosition = (int32_t) i64TempPosition;

    return i32AbsPosition;
}


#if ('A' == JOINTBOARD_TYPE)    /*肩关节水平*/
int16_t ShoulderJoint_H_ActiveCtrl(int16_t Gravity)
{
    int16_t li16TriggerForce = 0;
    
    /*触发肩关节水平节点运动的信号源有3个，分别是上臂力传感器、前臂力传感器和腕关节力传感器*/
    /*三个力传感器水平方向的合力即为触发肩关节水平运动的力*/
    /************************************************************************************************************/
    /*******************************************上臂力传感器水平力计算*******************************************/
    /************************************************************************************************************/
    /*
    上臂力传感器水平力与上臂力传感器绕Y轴旋转的角度有关，和Y轴的水平夹角无关。
    计算外力的水平方向力的大小时，需要获取传感器随机械臂旋转的角度数据
    */
    /*获取上臂传感器旋转的角度*/
    UpperarmForce.dfAngle_Sensor = (double)TestSlave_obj6067[UpperArm_R]/100;
    /*计算上臂力传感器的水平作用力*/
    UpperarmForce.dfAngle_Pitch = (double)TestSlave_obj6067[Shoulder_V]/100;
    /*计算上臂力传感器S1水平力*/
    ThreeAxisForceTrans(&UpperarmForce, UpperarmForce.dfAngle_Sensor, UpperarmForce.dfAngle_Pitch);
    
    /************************************************************************************************************/
    /****************************************前臂和手腕力传感器水平力计算****************************************/
    /************************************************************************************************************/
    /*计算腕部传感器和前臂力传感器的俯仰角*/
    ForearmForce.dfAngle_Pitch = GetForceSensorPitchAngle(TestSlave_obj6067[UpperArm_R]/100, TestSlave_obj6067[Elbow]/100) +\
                                TestSlave_obj6067[Shoulder_V]/100;
    
    /*计算腕部和前臂力传感器的旋转角*/
    /*腕部和前臂力传感器的旋转角 = 前臂旋转角 + 上臂旋转角*/
    ForearmForce.dfAngle_Sensor = (TestSlave_obj6067[UpperArm_R] + TestSlave_obj6067[Forearm_R])/100;

    /*由于腕部力传感器和前臂力传感器安装位置在同一轴线上，且腕关节变化范围很小，近似认为他们的旋转角和俯仰角相同*/
    ThreeAxisForceTrans(&ForearmForce, ForearmForce.dfAngle_Sensor,ForearmForce.dfAngle_Pitch);
    ThreeAxisForceTrans(&WristForce, ForearmForce.dfAngle_Sensor,ForearmForce.dfAngle_Pitch);
    
    li16TriggerForce = UpperarmForce.i16axis_h_Force;// +\
                        /*ForearmForce.i16axis_h_Force +\
                        WristForce.i16axis_h_Force;*/
    
    return li16TriggerForce;
}
#endif

/*肩关节垂直节点*/

/********************************************************************
 * @brief	肩关节垂直节点主动模式控制函数
 * @param   int16_t Gravity 重力参数
 *
 * @return  int16_t 触发肩关节垂直节点运动的等效力数据
 */
#if ('B' == JOINTBOARD_TYPE) 
int16_t ShoulderJoint_V_ActiveCtrl(int16_t Gravity)
{
    int16_t li16TriggerForce = 0;
    /*
    Note:
    触发肩关节垂直节点运动的因素有：
    1、安装在大臂旋转部件上的三维力传感器的垂直方向分量，任何状态下都可触发肩关节垂直节点；
    2、安装在腕关节和前臂旋转部件上的两个三维力传感器的垂直力分量，
        这部分垂直力分量对肩关节垂直节点运动的作用力与肘关节的角度有关，
        设肘关节角度为theta、触发肩关节垂直运动的等效力为F, 腕关节和前臂上三维力传感器的垂直力分量合力为f，
        作用到肩关节垂直节点的等效力为：
        F = cos(theta) * f;
    3、触发肩关节垂直运动的力为以上两个因素的合力；
    */
    /***********************************************************************************************/
    /*手肘力传感器S1旋转角 = 上臂旋转角度*/
    UpperarmForce.dfAngle_Sensor = (double)TestSlave_obj6067[UpperArm_R]/100;
    /*手肘力传感器S1俯仰角 = 肩关节垂直关节角度*/
    UpperarmForce.dfAngle_Pitch = (double)TestSlave_obj6067[Shoulder_V]/100;
    /*计算手肘力传感器S1垂直力和水平力*/
    ThreeAxisForceTrans(&UpperarmForce, UpperarmForce.dfAngle_Sensor, UpperarmForce.dfAngle_Pitch);
    
    /***********************************************************************************************/
    /*计算腕部传感器和前臂力传感器的俯仰角*/
    ForearmForce.dfAngle_Pitch = GetForceSensorPitchAngle(TestSlave_obj6067[UpperArm_R]/100, TestSlave_obj6067[Elbow]/100) +\
                                TestSlave_obj6067[Shoulder_V]/100;
    
    /*计算腕部和前臂力传感器的旋转角*/
    /*腕部和前臂力传感器的旋转角 = 前臂旋转角 + 上臂旋转角*/
    ForearmForce.dfAngle_Sensor = (TestSlave_obj6067[UpperArm_R] + TestSlave_obj6067[Forearm_R])/100;

    /*由于腕部力传感器和前臂力传感器安装位置在同一轴线上，且腕关节变化范围很小，近似认为他们的旋转角和俯仰角相同*/
    ThreeAxisForceTrans(&ForearmForce, ForearmForce.dfAngle_Sensor,ForearmForce.dfAngle_Pitch);
    ThreeAxisForceTrans(&WristForce, ForearmForce.dfAngle_Sensor,ForearmForce.dfAngle_Pitch);
    
    li16TriggerForce = ForearmForce.i16axis_v_Force + \
                        WristForce.i16axis_v_Force + \
                        UpperarmForce.i16axis_v_Force - Gravity;

    return li16TriggerForce;
}
#endif

/*上臂旋转节点*/
#if ('C' == JOINTBOARD_TYPE) 
/********************************************************************
 * @brief	上臂旋转节点主动模式控制函数
 * @param   int16_t Gravity 重力参数
 *
 * @return  int16_t 触发上臂旋转节点运动的力矩(放大了100倍)
 */
int16_t UpperarmRotate_ActiveCtrl(int16_t Gravity)
{
    int16_t li16TriggerForce = 0;
    double lfTorque = 0.0;
    double lfTempForce = 0.0;
    double lfArmofForce = 0.0;
    double ldAngle = 0.0;
    double lfSinValue = 0.0;
    double lfCosValue = 0.0;
    
    /***************************************计算前臂力传感器的力矩***************************************/
    /*Step1 获取肘关节角度*/
    ldAngle = TestSlave_obj6067[Elbow]/100;
    /*Step2 计算前臂传感器力臂*/
    
//    lfArmofForce = JointCtrl.Motion.i16ArmofForce1 * sin(ldAngle * PI/180);
    
    /*Step3 计算前臂力传感器对上臂电机轴切向力分量*/
    lfSinValue = sin((TestSlave_obj6067[Forearm_R]/100) * PI/180);
    lfCosValue = cos((TestSlave_obj6067[Forearm_R]/100) * PI/180);
    lfTempForce = ForearmForce.i16Axis_x_Force * lfCosValue +\
                    ForearmForce.i16Axis_z_Force * lfSinValue -\
                    Gravity * cos((TestSlave_obj6067[UpperArm_R]/100) * PI/180);    /*减去重力*/
    
    /*Step4 计算前臂力传感器对肘关节电机的力矩，力矩 = 力臂 * 力*/
    lfTorque = lfArmofForce * lfTempForce;

    /***************************************计算腕关节力传感器的力矩***************************************/
    /*Step1 计算手腕力传感器的等效力臂*/
//    lfArmofForce = JointCtrl.Motion.i16ArmofForce2 * sin(ldAngle * PI/180);
    /*Step2 计算腕关节力传感器对上臂电机轴切向力分量*/
    lfTempForce += WristForce.i16Axis_x_Force * lfCosValue +\
                    WristForce.i16Axis_z_Force * lfSinValue -\
                    Gravity * cos((TestSlave_obj6067[UpperArm_R]/100) * PI/180);    /*减去重力*/;

    /*施加到肘关节电机轴的力矩 = 腕关节力矩 + 前臂力矩*/
    lfTorque += lfArmofForce * lfTempForce;
    
    li16TriggerForce = (int16_t) lfTorque * 100;
    
    return li16TriggerForce;
}
#endif

/*肘关节节点，主动控制模式函数*/
/*
肘关节运动条件分析：
1. 前臂三维力传感器和腕部力传感器水平方向的合力可触发肘关节运动；
2. 需要知道前臂旋转的角度，因为腕部力传感器和前臂是整体运动，他们的角度在任何时刻都保持一致；
3. 需要测得前臂力传感器数据和腕部力传感器数据
*/
#if ('D' == JOINTBOARD_TYPE) /*肘关节*/
/** \brief 根据手腕和前臂力传感器上的力参数和前臂旋转参数、上臂旋转参数计算触发肘关节运动的力
 *
 * \param   int16_t Gravity 前臂加手腕的重力
 * \return  int16_t 返回的触发肘关节运动的力分量，包含重力补偿数据
 * \data    2019-06-20
 */
int16_t ElbowJointActiveCtrl(int16_t Gravity)
{
    int16_t li16TriggerForce = 0;
    double lfSinTheta = 0.0;
    double lfCosTheta = 0.0;
    
    double lfSinAlpha = 0.0;
    double lfCosAlpha = 0.0;
    
    double lfRotaTheta = 0.0;
    double lfPitchAlpha = 0.0;
    
    double lfTempForce1 = 0.0;
    double lfTempForce2 = 0.0;
    
    double lfGravityOffset = 0.0;

    /*前臂旋转角*/
    lfRotaTheta = TestSlave_obj6067[Forearm_R]/100;
    /*计算俯仰角*/
    lfPitchAlpha = GetForceSensorPitchAngle(lfRotaTheta, TestSlave_obj6067[Elbow]/100);
    lfPitchAlpha += (TestSlave_obj6067[Shoulder_V]/100);

    lfSinTheta = sin(lfRotaTheta * PI/180);
    lfCosTheta = cos(lfRotaTheta * PI/180);
    
    lfSinAlpha = sin(lfPitchAlpha * PI/180);
    lfCosAlpha = cos(lfPitchAlpha * PI/180);
    
    lfTempForce1 = (ForearmForce.i16Axis_z_Force/100) * lfCosTheta + \
                    (ForearmForce.i16Axis_x_Force/100) * lfSinTheta;

    lfTempForce2 = (WristForce.i16Axis_z_Force/100) * lfCosTheta + \
                    (WristForce.i16Axis_x_Force/100) * lfSinTheta;
                    
    /*重力补偿原理见文档<<重力补偿原理及仿真_20190620>>*/
    lfGravityOffset = 2 * Gravity * lfCosAlpha * lfCosTheta * lfSinTheta;

    /*触发肘关节运动的力 = 腕关节力传感器的力与前臂力传感器的合力 减去重力补偿*/
    li16TriggerForce = (int16_t)(lfTempForce1 + lfTempForce2 - lfGravityOffset);
    
    return li16TriggerForce;
}
#endif


/*腕关节主动控制模式*/
/*影响腕关节主动模式运动的因素分析：
1：腕关节运动只和腕关节三维力传感器x轴方向的力有关；
2：力的方向决定了关节的运动方向；
3：力的大小决定了关节的运动速度；
4：X轴方向的力与重力有耦合，需要综合其它关节角度判断腕关节在空间中的状态；
综述：
1. 控制腕关节需要获取腕关节当前角度；
2. 获取腕关节三维力传感器X轴力的大小和方向；
3. 根据其它关节角度计算腕关节力传感器在三维空间中的状态；
*/
#if ('F' == JOINTBOARD_TYPE) /*腕关节*/
int16_t WristJointActiveCtrl(int16_t Gravity)
{
    int32_t li32AngleTemp = 0;
    int16_t li16TriggerForce = 0;

    /*Step1: 感知自身状态，绕水平径向轴旋转的角度*/
    li32AngleTemp = (ForearmRotatePara.i32JointActualAngle + UpperarmRotatePara.i32JointActualAngle)/100;
    /*人体手臂对X轴的真实作用力 = X轴测量得到的力Fx - 手部重力 * cos(重力与X轴的夹角)*/
    /*如果测得的力大于0，则需要抵消重力*/
    if(abs(WristForce.i16Axis_x_Force/100) > JointCtrl.Motion.u16Axis_h_Threshold)
    {
        if(WristForce.i16Axis_x_Force > 0)
        {
            li16TriggerForce = (WristForce.i16Axis_x_Force/100) - \
                                    (Gravity * cos(li32AngleTemp));
        }
        else
        {
            li16TriggerForce = (WristForce.i16Axis_x_Force/100);
        }
    }
    
    return (JointCtrl.JointIncDir * li16TriggerForce);
}
#endif


#if ('B' == JOINTBOARD_TYPE) /*腕关节*/
void MotorBreakProcess(void)
{
    /*
    触发刹车的条件有：
    1、伺服错误码不为0；
    2、伺服状态字Fault位不为0；
    有待丰富
    */
    if((0 != EposCtrl.u16ErrorCode)
        || (0 != EposCtrl.u16Statusword.Status_bit.b3_Fault)
        || (0 == EposCtrl.u16Statusword.Status_bit.b1_SwitchOn)
        || (0 == EposCtrl.u16Statusword.Status_bit.b2_OpEnable)
        || (0 == EposCtrl.u16Statusword.Status_bit.b4_VoltEnable))
    {
        JointCtrl.Motion.BreakCtrl = Break;
    }
    else
    {
        JointCtrl.Motion.BreakCtrl = Release;
    }
}
#endif


void ProtectProcess(void)
{
    ;
}

void PositionLimit(int32_t *targetPosition, int32_t positionMax, int32_t positionMin)
{
    if(*targetPosition > positionMax)
    {
        *targetPosition = positionMax;
    }
    else if(*targetPosition < positionMin)
    {
        *targetPosition = positionMin;
    }
}


/*此函数只在CSP模式下可用，用于限定*/
void CSP_PositionLimit(void)
{

}


//#endif
