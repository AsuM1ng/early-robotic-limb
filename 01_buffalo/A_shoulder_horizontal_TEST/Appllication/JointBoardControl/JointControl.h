#ifndef JOINTCONTROL_H_INCLUDED
#define JOINTCONTROL_H_INCLUDED

#include <stdint.h>
#include "Board_type_config.h"
#include "servo_Control.h"
#include "ThreeAxisForceTransform.h"
#include "BreakControl.h"
#include "MotionPathProcess.h"
#include "HandSwitch.h"
#include "AngleSensor.h"



/*关节特有参数定义，不同关节可能不一样，Epos驱动器初始化时需要参考以下参数*/
#if ('A' == JOINTBOARD_TYPE)    /*肩关节水平*/

#if(HANDTYPE == RIGHTHAND)
#define JOINT_ANGLE_MAX             315.0       /*关节运动角度最大值*/
#define JOINT_ANGLE_STANDBY         270.0       /*关节待机角度*/
#define JOINT_ANGLE_MIN             220.0       /*关节运动角度最小值*/
#else
#define JOINT_ANGLE_MAX             135.0       /*关节运动角度最大值*/
#define JOINT_ANGLE_STANDBY         90          /*关节待机角度*/
#define JOINT_ANGLE_MIN             40.0        /*关节运动角度最小值*/
#endif

#define JOINT_ANGLE_OFFSET          -27301       /*83.00关节安装角度偏移*/

#define JOINT_VELOCITY_MAX          2400        /*关节电机最高转速60Hz*/
#define JOINT_VELOCITY_MIN          600         /*关节电机最低转速10Hz*/
#define JOINT_VELOCITY_DEF          240         /*默认转速4Hz*/

#define JOINT_STEPLENG_MAX          1600         /*关节最大步进*/
#define JOINT_STEPLENG_MIN          800          /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     4           /*关节运动触发门限,单位 N.m*/

#define ABSENCODER_IN_GEAR_SIDE     TRUE
#define JOINT_GEAR_REDUCTION        120         /*关节减速器减速比*/
#define ENCODER_BIT                 12          /*编码器单圈分辨率*/

#elif ('B' == JOINTBOARD_TYPE)  /*肩关节垂直*/

#if(LEFTHAND == HANDTYPE)
    #define JOINT_ANGLE_MAX             320.0       /*关节运动角度最大值*/
    #define JOINT_ANGLE_STANDBY         270.0       /*关节待机角度*/
    #define JOINT_ANGLE_MIN             235.0       /*关节运动角度最小值*/
#elif(RIGHTHAND == HANDTYPE)    /*2019年8月5日，只调试右手，换手后面再添加*/
    #define JOINT_ANGLE_MAX             125.0       /*关节运动角度最大值*/
    #define JOINT_ANGLE_STANDBY         90          /*关节待机角度*/
    #define JOINT_ANGLE_MIN             40.0        /*关节运动角度最小值*/
#endif

#define JOINT_ANGLE_OFFSET          25937       /*关节安装角度偏移*/
#define JOINT_VELOCITY_MAX          1800        /*关节电机最高转速30Hz*/
#define JOINT_VELOCITY_MIN          120         /*关节电机最低转速2Hz*/
#define JOINT_VELOCITY_DEF          600         /*默认转速10Hz*/

#define JOINT_STEPLENG_MAX          200         /*关节最大步进*/
#define JOINT_STEPLENG_MIN          50          /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     2           /*关节运动触发门限,单位 N.m*/

#define JOINT_GEAR_REDUCTION        120         /*关节减速器减速比*/
#define ENCODER_BIT                 14          /*编码器单圈分辨率*/

#elif ('C' == JOINTBOARD_TYPE)

#define JOINT_ANGLE_MAX             60.0        /*关节运动角度最大值*/
#define JOINT_ANGLE_STANDBY         0           /*关节待机角度*/
#define JOINT_ANGLE_MIN             -60.0       /*关节运动角度最小值*/
#define JOINT_ANGLE_OFFSET          2050870     /*关节安装角度偏移，此关节比较特殊，偏移值为脉冲增量，实际测量得到*/

#define JOINT_VELOCITY_MAX          2800        /*关节电机最高转速30Hz*/
#define JOINT_VELOCITY_MIN          800         /*关节电机最低转速2Hz*/
#define JOINT_VELOCITY_DEF          1200        /*默认转速4Hz*/

#define JOINT_STEPLENG_MAX          1600         /*关节最大步进*/
#define JOINT_STEPLENG_MIN          600          /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     2           /*关节运动触发门限,单位 N.m*/

#define JOINT_GEAR_REDUCTION        193         /*关节减速器减速比*/
#define ENCODER_BIT                 17          /*编码器单圈分辨率*/

#elif ('D' == JOINTBOARD_TYPE)  /*肘关节*/

#define JOINT_ANGLE_MAX             111.0       /*关节运动角度最大值*/
#define JOINT_ANGLE_MIN             1.0         /*关节运动角度最小值*/
#define JOINT_ANGLE_STANDBY         1.0         /*关节待机角度*/
#define JOINT_ANGLE_OFFSET          21460       /*关节安装角度偏移*/

#define JOINT_VELOCITY_MAX          2400        /*关节电机最高转速50Hz*/
#define JOINT_VELOCITY_MIN          600         /*关节电机最低转速10Hz*/
#define JOINT_VELOCITY_DEF          600         /*默认转速5Hz*/

#define JOINT_STEPLENG_MAX          1600         /*关节最大步进*/
#define JOINT_STEPLENG_MIN          800          /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     2           /*关节运动触发门限,单位 N.m*/

#define JOINT_GEAR_REDUCTION        120         /*关节减速器减速比*/
#define ENCODER_BIT                 12          /*编码器单圈分辨率*/

#elif ('E' == JOINTBOARD_TYPE)  /*肘关节*/

#define JOINT_ANGLE_MAX             58.0         /*关节运动角度最大值*/
#define JOINT_ANGLE_STANDBY         0.0         /*关节待机角度*/
#define JOINT_ANGLE_MIN             -58.0       /*关节运动角度最小值*/
#define JOINT_ANGLE_OFFSET          -31265      /*关节安装角度偏移*/

#define JOINT_VELOCITY_MAX          2400        /*关节电机最高转速50Hz*/
#define JOINT_VELOCITY_MIN          600         /*关节电机最低转速10Hz*/
#define JOINT_VELOCITY_DEF          600         /*默认转速5Hz*/

#define JOINT_STEPLENG_MAX          1600        /*关节最大步进*/
#define JOINT_STEPLENG_MIN          800         /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     2           /*关节运动触发门限,单位 N.m*/

#define JOINT_GEAR_REDUCTION        233         /*关节减速器减速比*/
#define ENCODER_BIT                 12          /*编码器单圈分辨率*/

#elif ('F' == JOINTBOARD_TYPE)  /*腕关节运动参数*/

#define JOINT_ANGLE_OFFSET          25464       /*关节安装角度偏移*/
#define JOINT_ANGLE_MAX             69.0        /*关节运动角度最大值*/
#define JOINT_ANGLE_STANDBY         0           /*关节待机角度*/
#define JOINT_ANGLE_MIN             -69.0       /*关节运动角度最小值*/

#define JOINT_VELOCITY_MAX          7000        /*关节电机最高转速50Hz*/
#define JOINT_VELOCITY_MIN          3000         /*关节电机最低转速10Hz*/
#define JOINT_VELOCITY_DEF          3000         /*默认转速6Hz*/

#define JOINT_STEPLENG_MAX          4000         /*关节最大步进*/
#define JOINT_STEPLENG_MIN          1000          /*关节最小步进*/
#define JOINT_TRIGGER_THRESHOLD     2           /*关节运动触发门限,单位 N.m*/

#define JOINT_GEAR_REDUCTION        132         /*关节减速器减速比*/
#define ENCODER_BIT                 12          /*编码器单圈分辨率*/

#else

#error "You forget define the board type !!"

#endif

typedef enum{
    Shoulder_H  = 0,    /*肩关节水平*/
    Shoulder_V  = 1,    /*肩关节垂直*/
    UpperArm_R  = 2,    /*上臂旋转*/
    Elbow       = 3,    /*肘关节*/
    Forearm_R   = 4,    /*前臂旋转*/
    Wrist_R     = 5     /*腕关节旋转*/
}JointName_en;


/*示教模式的状态机*/
typedef enum{
    ePathRecord     = 0,
    ePointVerify    = 1,
    ePathReappear   = 2,
    eReturnOrigin   = 3
}TeachSection_en;

/*示教模式下的控制字命令*/
typedef enum{
    eRecordStart        = 0xA1,     /*开始记录路径*/
    eRecordStop         = 0xA5,     /*停止记录路径*/
    eReappearStart      = 0xB2,     /*路径重现开始*/
    eReappearStop       = 0xB5,     /*路径重现停止*/
    eClearBuff          = 0xCE,     /*清除缓存*/
    ePathFillFinish     = 0xFA,     /*路径数据写入完成*/
    eNoop               = 0x00
}TeachCommand_en;




typedef enum{
    CommLost = 0,       /*和主控通讯故障*/
    CommNormal = 1,     /*通信正常*/
}CommStatus_en;

/*工作模式定义*/
typedef enum{
    StandbyMode     = 0x00,     /*待机模式*/
    PassiveMode     = 0xA1,     /*被动模式*/
    ActiveMode      = 0xB2,     /*主动模式*/
    ActPassive      = 0xC3,     /*主被动模式*/
    ImpedanceMode   = 0xD4,     /*抗阻模式*/
    TeachingMode    = 0xE5,     /*示教模式*/
    RecipeMode      = 0xF6,     /*处方模式*/
    ChangeHand      = 0xBC,     /*自动换手模式*/
    MotorTest       = 0xC2,     /*电机测试模式*/
    PositionCali    = 0xCC,     /*位置校准*/
    AppShutdown     = 0x55,     /*控制器关闭*/
    AppSwitchOn     = 0xAA,      /*使能*/
    ClearError      = 0xCE      /*清除控制器故障*/
}AppMode_en;

typedef struct{
    CommStatus_en enCommuStatus;        /*通信状态，主要指节点板和主控的通信*/
    AppMode_en enControlMode;           /*主控发送给节点板的模式命令*/
    uint16_t u16Statusword;             /*设备工作状态*/
    int32_t i32TargetAngle;             /*目标角度*/
    int32_t i32CommProfileSpeed;         /*主控发送的目标速度信息*/
    uint16_t u16ErrorCode;              /*节点板错误代码*/
}Communication_t;



typedef struct{
    _Bool   bMotionEnable;                  /*关节运动使能*/
    uint16_t u16ActionTime;
    
    int16_t i16GravityOffset;               /*重力补偿*/
    
    uint16_t u16Axis_v_Threshold;           /*肩关节水平节点电机垂直运动的启动力矩*/
    uint16_t u16Axis_h_Threshold;           /*肩关节水平节点电机水平运动的启动力矩*/
    
    int16_t i16TriggerResuForce;            /*触发关节运动的合力*/

#if ('B' == JOINTBOARD_TYPE)            /*只有肩关节垂直节点才有刹车*/
    BreakControl_en BreakCtrl;          /*刹车释放使能*/
    BreakStatus BreakStatus;            /*刹车状态*/
#endif

    MotorDirection enMotonDirection;    /*电机运动方向*/

    uint16_t u16MotionStepLengthMax;    /*步进最大值*/
    uint16_t u16MotionStepLengthMin;    /*步进最小值*/
    uint16_t u16MotionLength;           /*位置增量的步进*/
    
    uint32_t u32MotionSpeedMax;         /*电机最高速度*/
    uint32_t u32MotionSpeedMin;         /*电机最低速度*/
    uint32_t u32ProfileSpeed;           /*电机设定速度*/
    
    /*因为每次上电，关节所处的绝对位置不同，且电机本身的增量编码器不具备存储绝对位置功能，故每次上电时需要重新计算*/
    int32_t i32JointAbsPositionMax;     /*关节最大绝对位置*/
    int32_t i32JointAbsPositionMin;     /*关节最小绝对位置*/
    
    int32_t i32JointNewPosition;        /*新的目标位置*/
    int32_t i32JointNewAngle;           /*新的目标角度*/
    int32_t i32JointPreAngle;           /*主控上一次发送的角度*/
    
    int32_t i32JointTargetPosition;     /*关节目标位置*/
    int32_t i32JointCaliTarget;         /*关节校准位置*/
    int32_t i32JointActualPosition;     /*关节实际位置*/
    int32_t i32JointStanbyPosi;         /*关节零点位置*/
    int32_t i32JointHaltPosi;           /*电机停止位置*/
    
    int16_t i16JointDeltaPosi;          /*关节位置增量*/
    
    int32_t i32JointAngleMax;           /*关节最大角度*/
    int32_t i32JointAngleMin;           /*关节最小角度*/
    int32_t i32JointStandbyAngle;       /*关节待机位置*/
    int32_t i32JointAngleOffset;        /*关节角度安装位置偏移*/
    int32_t i32JointActualAngle;        /*关节当前角度*/
}MotionControl_t;

/*示教模式下的状态字定义*/
typedef struct Joint_status_field
{
    uint16_t b0_InitFinished    : 1;
    uint16_t b1_TestFinished    : 1;
    uint16_t b2_ServoPowerOn    : 1;
    uint16_t bx_Reserved        : 13;
}stc_JointStatus_field_t;

typedef union JointStatus
{
    stc_JointStatus_field_t Status_bit;
    uint16_t u16AllStatus;
}JointStatusField_t;


typedef struct{
    _Bool bJointProcessStart;           /*节点控制使能*/
    uint16_t u16ErrorCode;              /*节点错误码*/
    JointStatusField_t u16JointStatus;  /*节点状态*/

    HandType_en HandType;               /*左/右手选择*/
    
    EncoderIncDir_en JointIncDir;       /*绝对位置传感器与电机增量编码器方向--同向/反向*/
    
    _Bool asynPdoEnable;                /*异步PDO发送使能*/
    _Bool synPdoEnable;                 /*同步PDO发送使能*/
    
    AppMode_en JointContrlMode;         /*关节运行模式*/
    AppMode_en JointPreviouMode;        /*前一次模式*/
    uint16_t u16JontControlCycle;       /*关节控制周期，单位：mS*/
    
    MotionControl_t     Motion;
    MotionPath_t        MotionPath;
    PVT_Typedef         PVT_struct;
    PVT_Typedef         PVT_Back_struct;
}JointControl_t;

typedef struct{
    int32_t i32JointMaxPosition;     /*关节最大行程位置*/
    int32_t i32JointMinPosition;     /*关节最小行程位置*/

    int32_t i32ProfileVelocity;     /*关节运动速度*/
    int32_t i32JointMaxVelocity;    /*关节最高速度*/
    int32_t i32JointMinVelocity;    /*关节最低速度*/

    int32_t i32JointStandbyAngle;   /*关节待机角度*/
    int32_t i32JointActualAngle;    /*关节实时角度*/
}JointFeature_t;




extern Communication_t MasterCommand;

extern JointControl_t JointCtrl;     /*关节控制相关变量*/


uint8_t JointControlParaInit(void);
void JointControlProcess(void);
void LEDTwinkleProcess(_Bool OnOff);
int32_t encoderPulseConvertToAngle(int32_t standbyPosition,
                                    int32_t currentPosition,
                                    uint16_t SingleturnResolution,
                                    uint16_t gearReduction);
int32_t AngleConvertToAbsPosition(int16_t i16ActualAngle,
                                int16_t i16TargetAngle,
                                uint16_t u16SingleturnResolution,
                                uint16_t u16GearReduction);
void PositionLimit(int32_t *targetPosition,
                    int32_t positionMax,
                    int32_t positionMin);

#endif // JOINTCONTROL_H_INCLUDED
