#ifndef SERVOCONTROL_H_INCLUDED
#define SERVOCONTROL_H_INCLUDED

#include <stdint.h>
#include "Board_type_config.h"


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#define CLEARFAULT              0x0080  /*Epos 清除错误操作码*/

#define MOTORSHUTDOWN           0x0006  /*电机停机，断电*/
#define MOTORENABLE             0x000F  /*电机使能*/
#define MOTORQUICKSTOP          0x000B  /*电机急停*/
#define MOTORHALT               0x010F  /*电机保持原位置通知，立定*/

#define PPM_ABS                 0x001F  /*绝对位置模式*/
#define PPM_ABS_IMM             0x003F  /*绝对位置模式，立即执行*/
#define PPM_REL                 0x005F  /*相对位置模式*/
#define PPM_REL_IMM             0x007F  /*相对位置模式，立即执行*/


#define MAXACCELERATION         3000    /*最大加速度*/
#define PROFILEACCELERATION     2000    /*设置加速度*/
#define PROFILEDECELERATION     2000    /*设置减速度*/
#define QUICKSDECELERATION      3000    /*急停减速度*/
#define MOTORMAXVELOCITY        3000    /*电机最高速度*/
#define PROFILEVELOCITY         1500    /*电机默认速度*/

typedef enum{
    Abs_Position        = 0x1F,
    Abs_Position_Imm    = 0x3F,
    Relat_Position      = 0x5F,
    Relat_Position_Imm  = 0x7F
}ExecuteMode_en;

/*EPOS Operation MODE, EPOS OD index: 0x6060*/
typedef enum{
    PPMode  = 0x01,
    PVMode  = 0x03,
    CSP     = 0x08,
    CSV     = 0x09,
    CST     = 0x0A
}EposOperationMode;


typedef enum{
    InitSuccess = 1,
    InitFail    = 0xFF
}EposInitStatus;

typedef enum{
    clockwise       = 0,
    anticlockwise   = 1
}MotorDirection;

typedef enum{
    Shutdown = 0,
    SwitchOn = 1,
    Halt     = 2
}EposStatus_en;


/*示教模式下的状态字定义*/
typedef struct servo_status_field
{
    uint16_t b0_Ready            : 1;
    uint16_t b1_SwitchOn         : 1;
    uint16_t b2_OpEnable         : 1;
    uint16_t b3_Fault            : 1;
    uint16_t b4_VoltEnable       : 1;
    uint16_t b5_QuickStop        : 1;
    uint16_t b6_SwitchOnDis      : 1;
    uint16_t b7_Warning          : 1;
    uint16_t b8_reserved         : 1;
    uint16_t b9_Remote           : 1;
    uint16_t b10_TargetReach     : 1;
    uint16_t b11_LimitActive     : 1;
    uint16_t b12_ServoACK        : 1;
    uint16_t b13_FollowError     : 1;
    uint16_t b14_reserved        : 1;
    uint16_t b15_PRtHP           : 1;
}stc_servoStatus_field_t;

typedef union ServoStatus
{
    stc_servoStatus_field_t Status_bit;
    uint16_t u16AllStatus;
}SvervoStatusField_t;


typedef struct{
    _Bool   bEpos4InitFinished;         /*伺服初始化完成标志*/
    EposInitStatus  enEpos4InitStatus;  /*Epos 初始化状态*/
    EposOperationMode enEpos4OpMode;    /*伺服驱动器工作模式*/
    _Bool bMotorTargetReach;            /*电机到达设定值(位置/速度)*/
    
    EposStatus_en enEposCurrStatus;      /*EPOS 当前状态*/
    
    int16_t i16PowerSupply;
    
    MotorDirection enDirection;
//    uint8_t u8Direction;
    
    uint16_t u16Controlword;            /*对象0x6040, 伺服控制字*/
    SvervoStatusField_t u16Statusword;  /*对象0x6041, 伺服状态字*/
    uint16_t u16ErrorCode;              /*对象0x603F, 伺服错误状态码*/
    
    int32_t i32SoftPositionLimitMax;    /*伺服最大位置限定值,限位智能软限位，如果用伺服控制器硬限位会引起伺服报错并停机*/
    int32_t i32SoftPositionLimitMin;    /*伺服最小位置限定值,限位智能软限位，如果用伺服控制器硬限位会引起伺服报错并停机*/
    
    int32_t i32ServoInitPosition;       /*伺服初始位置设置*/
    int32_t i32ServoPreviousPosition;   /*伺服前一次位置*/
    int32_t i32ServoTargetPosition;     /*伺服目标位置*/
    int32_t i32ServoActualPosition;     /*伺服当前实际位置*/
    int32_t i32ServoDemandPosition;     /**/
    int16_t i16TorqueAverageValue;       /*伺服当前实际输出力矩*/
    
    int32_t i32ServoInitVelocity;       /*伺服初始速度设置*/
    uint32_t u32ServoProfileVelocity;   /*伺服设定速度*/
    uint32_t u32MotorMaxVelocity;       /*电机最高速度*/
    int32_t i32ServoTargetVelocity;     /*伺服目标速度*/
    int32_t i32ServoActualVelocity;     /*伺服当前实际速度*/
    
    uint32_t u32MaxAcceleration;        /*电机最大加速度*/
    uint32_t u32MotorAcceleration;      /*电机加速度*/
    uint32_t u32MotorDeceleration;      /*电机减速度*/
    uint32_t u32MotorQSDeceleration;    /*电机急停减速度*/
    
    int32_t i32ServoAveragedCurrent;    /*伺服平均电流*/
    int32_t i32ServoActualCurrent;      /*伺服实际电流*/
}ServoControl_t;

extern ServoControl_t EposCtrl;



void Epos4ServoInit(ServoControl_t* servoCtlPara);

/*电机急停*/
uint8_t EposQuickStop(void);

/*电机保持原位置停止*/
uint8_t EposHalt(void);

/*控制器使能*/
uint8_t EposSwitchOn(void);

/*控制器关闭*/
uint8_t EposShutdown(void);


/*PPM 模式下，发送新的位置给控制器*/
uint8_t PPMMovetoNewPostion(int32_t NewPosition, uint32_t profileVelocity, ExecuteMode_en ExeMode);


uint8_t MonitorEposError(void);

uint16_t GetServoStatus(void);


uint8_t ClearServoFault(void);

/*主动读取伺服当前位置*/
int32_t GetServoActualPosition(void);

/*读取伺服控制的电源电压*/
int16_t GetServoPowerSupply(void);

/*手动发送PDO实时角度数据*/
void PDOSendJointAngle(void);

/*驱动器模式切换*/
void ServoModeInit(EposOperationMode mode);

#endif // SERVOCONTROL_H_INCLUDED
