#ifndef MOTIONPATHPROCESS_H_INCLUDED
#define MOTIONPATHPROCESS_H_INCLUDED

#include <stdint.h>

#define MAX_PVT_OBJECT      65          /*PVT缓存长度*/
#define MAX_PVT_BUFFER      256         /*PVT记录轨迹的一级缓存长度*/

#define RECORDTIME          30          /*记录时长，单位(秒/S)*/
#define HALTDELAY           2000        /*一个动作结束，下一个动作的间隔时间，单位mS*/
#define THRESHOLD_V         60          /*PVT点记录的速度门限，低于此速度的点才符合要求，单位Rpm*/
#define ENDTOSTARTTIME      4.0        /*示教模式下，关节从运动路线结束点返回到起始点的时间长度, 单位:秒/S*/

typedef struct
{
	int position;
	int velocity;
	float time;
}PVT_Table_Typedef;


typedef struct
{
    volatile    uint8_t start;
    volatile    uint8_t pauseNext;
	uint8_t stop_flag;
    
    PVT_Table_Typedef pvt_table_struct[MAX_PVT_BUFFER];
    PVT_Table_Typedef Temp_Buff[MAX_PVT_BUFFER];
    uint16_t u16P_index;
	
	volatile PVT_Table_Typedef * p_start;
	volatile PVT_Table_Typedef * p_next;
	volatile PVT_Table_Typedef * p_end;
    
    int32_t i32PosTarget;
    
    int32_t i32PVT_P_Target;
	int32_t i32PVT_V_Ref;
    
	uint16_t number_of_interpolation;
	uint16_t now_inner_point;				//插值点之间的内点

	
	float t;
	
	float s3;
	float s2;
	float s1;
	float s0;
	
	float v2;
	float v1;
	float v0;
	
	float MC_period_time;
	float i_pos_fdbk;
	float i_vel_fdbk;
	float o_pos_ref;
	float o_vel_ref;
	uint8_t MP[4];
	float a;
	float b;
	float c;
	float d;
}PVT_Typedef;



typedef struct{
    int32_t i32Angle;
    int32_t i32Velocity;
    float   fTime;
}MotionIterm_t;


/*示教模式下的状态字定义*/
typedef struct record_status_field
{
    uint16_t b0_RecordStart     : 1;    /*路径记录开始标志位            0：记录未开始；     1：正在记录*/
    uint16_t b1_RecordFinish    : 1;    /*路径记录完成标志位            0：记录未完成；     1：记录完成*/
    uint16_t b2_PathPointError  : 1;    /*路径记录数据点错误标志位      0：数据无错误：     1：数据错误*/
    uint16_t b3_RestoreStatus   : 1;    /*路径还原开始标志位            0：路径还原停止：   1：正在还原路径*/
    uint16_t b4_BufferEmpty     : 1;    /*路径缓存区清除标志位          0：缓存区不为空：   1：缓存区为空*/
    uint16_t b5_ReurnStatus     : 1;    /*示教路径返回标志位            0：路径返回完成；   1：正在返回中*/
    uint16_t b6_Reserved        : 1;    /*处方模式下，路径写入完成标志  0：路径未写入；     1：路径写入完成*/
}stc_status_field_t;

typedef union allStatus
{
    stc_status_field_t Status_bit;
    uint16_t u16AllStatus;
}StatusField_t;


typedef struct{
    uint16_t u16BuffMaxSize;            /*路径缓存大小*/
    
    uint16_t u16CtrlWowrds;             /*路径控制字*/
    StatusField_t u16StatusWords;       /*路径控制状态字*/
    uint32_t u32HaltDelay;              /*电机立定延时*/
    
    _Bool   bPathRecordEn;                      /*路径记录使能*/
    _Bool   bPathRecordFinish;                  /*路径记录完成标志*/
    MotionIterm_t PathBuff[MAX_PVT_OBJECT];     /*路径点三元素*/
    
    uint16_t u16BuffFillFinish;         /*示教模式下，路径填充完成*/
    
    uint32_t u32TimeCount;              /*路径记录计时*/
    uint16_t u16TimeDelay;              /*时间延时*/
    
    _Bool bReturnPathInitFlag;          /*示教返回路线初始化标志*/
    _Bool bRestorePathInitFlag;         /*重构路径初始化完成标志*/

    _Bool bReappearStart;               /*路径重现开始*/
    _Bool bReturnStart;                 /*返回起始点*/
    uint8_t u8ClearBuff;                /*清除缓存数据*/
    
    uint16_t u16TotalPoint;             /*路径数据点总数*/
    uint16_t u16PathIndex;              /*有效路径数据序号*/
    _Bool    bFirstPoint;               /*第一个点*/
}MotionPath_t;


void PVT_MotionPathInit(PVT_Typedef *pPVT_struct, float pos_Ts,uint8_t mp, uint16_t startIndex, uint16_t endIndex);

uint8_t PVT_Generator(PVT_Typedef *pvt_struct);

extern int pvt_o_vel_ref;
extern uint32_t IntCount;

void InitTeachingPath(void);
/*初始化返回路径*/
_Bool InitPathReturn(void);
void MotionPathRecorder(void);
void InitTeachingPath(void);
void MotionPathRestore(void);
void MotionPathSwitch(void);

#endif // MOTIONPATHPROCESS_H_INCLUDED
