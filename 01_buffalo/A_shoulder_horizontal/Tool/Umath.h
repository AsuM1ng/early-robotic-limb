/*****************************************************************
*
*             中文名称：数学模块
*             版本2.0,
*             最近修改日期：
*             作者:
*			  修改记录：
			  20140123 新增倒数表
*             20160320 将正弦表变为Q11格式
******************************************************************/
#ifndef _U_MATH_H_
#define _U_MATH_H_

#include <stdint.h>

#define GLOBLE_Q   15

#define UM_PI (3.14159265)
#define UM_e  (2.71828182)

typedef enum 
{
	BFALSE = 0, 
	BTRUE = !BFALSE
} bool_t;

typedef int32_t intQ1;
typedef int32_t intQ2;
typedef int32_t intQ3;
typedef int32_t intQ4;
typedef int32_t intQ5;
typedef int32_t intQ6;
typedef int32_t intQ7;
typedef int32_t intQ8;
typedef int32_t intQ9;
typedef int32_t intQ10;
typedef int32_t intQ11;
typedef int32_t intQ12;
typedef int32_t intQ13;
typedef int32_t intQ14;
typedef int32_t intQ15;
typedef int32_t intQ16;

typedef uint32_t uintQ1;
typedef uint32_t uintQ2;
typedef uint32_t uintQ3;
typedef uint32_t uintQ4;
typedef uint32_t uintQ5;
typedef uint32_t uintQ6;
typedef uint32_t uintQ7;
typedef uint32_t uintQ8;
typedef uint32_t uintQ9;
typedef uint32_t uintQ10;
typedef uint32_t uintQ11;
typedef uint32_t uintQ12;
typedef uint32_t uintQ13;
typedef uint32_t uintQ14;
typedef uint32_t uintQ15;
typedef uint32_t uintQ16;

extern int32_t UM_IQ15MulQ15(int32_t a , int32_t b); 
extern int32_t UM_IQ15MulQ7(int32_t a , int32_t b); 

/* QN to Float */
#define UM_Q1ToF(a) 	((a)*0.5)
#define UM_Q2ToF(a) 	((a)*0.25)
#define UM_Q3ToF(a) 	((a)*0.125)
#define UM_Q4ToF(a) 	((a)*0.0625)
#define UM_Q5ToF(a) 	((a)*0.03125)
#define UM_Q6ToF(a) 	((a)*0.015625)
#define UM_Q7ToF(a)		((a)*0.0078125)
#define UM_Q8ToF(a) 	((a)*0.00390625)
#define UM_Q9ToF(a) 	((a)*0.001953125)
#define UM_Q10ToF(a) 	((a)*0.0009765625)
#define UM_Q11ToF(a) 	((a)*0.00048828125)
#define UM_Q12ToF(a) 	((a)*0.000244140625)
#define UM_Q13ToF(a) 	((a)*0.0001220703125)
#define UM_Q14ToF(a) 	((a)*0.00006103515625)
#define UM_Q15ToF(a) 	((a)*0.000030517578125)	//Q15 to Float,Float=Q15*1/32768=Q15*0.000030517578125
#define UM_Q16ToF(a) 	((a)*0.0000152587890625)

/* float to QN */
#define  UM_Q1(a) 		( (int32_t)((a)*2.0) 		)
#define  UM_Q2(a) 		( (int32_t)((a)*4.0) 		)
#define  UM_Q3(a) 		( (int32_t)((a)*8.0) 		)
#define  UM_Q4(a) 		( (int32_t)((a)*16.0)		)
#define  UM_Q5(a) 		( (int32_t)((a)*32.0) 		)
#define  UM_Q6(a) 		( (int32_t)((a)*64.0) 		)
#define  UM_Q7(a) 		( (int32_t)((a)*128.0) 		)
#define  UM_Q8(a) 		( (int32_t)((a)*256.0) 		)
#define  UM_Q9(a) 		( (int32_t)((a)*512.0) 		)
#define  UM_Q10(a) 		( (int32_t)((a)*1024.0) 	)
#define  UM_Q11(a) 		( (int32_t)((a)*2048.0) 	)
#define  UM_Q12(a) 		( (int32_t)((a)*4096.0) 	)
#define  UM_Q13(a) 		( (int32_t)((a)*8192.0) 	)
#define  UM_Q14(a) 		( (int32_t)((a)*16384.0) 	)
#define  UM_Q15(a) 		( (int32_t)((a)*32768.0) 	)
#define  UM_Q16(a) 		( (int32_t)((a)*65536.0) 	)

/* Q乘法：返回QN。 */
#define  UM_QnMulQ0(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 0 )			//QN * Q0  --> 返回QN
#define  UM_QnMulQ1(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 1 )			//QN * Q1  --> 返回QN
#define  UM_QnMulQ2(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 2 )			//QN * Q2  --> 返回QN
#define  UM_QnMulQ3(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 3 )			//QN * Q3  --> 返回QN
#define  UM_QnMulQ4(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 4 )			//QN * Q4  --> 返回QN
#define  UM_QnMulQ5(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 5 )			//QN * Q5  --> 返回QN
#define  UM_QnMulQ6(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 6 )			//QN * Q6  --> 返回QN
#define  UM_QnMulQ7(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 7 )			//QN * Q7  --> 返回QN
#define  UM_QnMulQ8(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 8 )			//QN * Q8  --> 返回QN
#define  UM_QnMulQ9(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 9 )			//QN * Q9  --> 返回QN
#define  UM_QnMulQ10(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 10)			//QN * Q10 --> 返回QN
#define  UM_QnMulQ11(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 11)			//QN * Q11 --> 返回QN
#define  UM_QnMulQ12(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 12)			//QN * Q12 --> 返回QN
#define  UM_QnMulQ13(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 13)			//QN * Q13 --> 返回QN
#define  UM_QnMulQ14(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 14)			//QN * Q14 --> 返回QN
#define  UM_QnMulQ15(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 15)			//QN * Q15 --> 返回QN
#define  UM_QnMulQ16(a,b) 	( ((int64_t)(a)*(int64_t)(b)) >> 16)			//QN * Q16 --> 返回QN


#define UM_IntQ1(a) 	( (int32_t)(a) >> 1 )
#define UM_IntQ2(a) 	( (int32_t)(a) >> 2 )
#define UM_IntQ3(a) 	( (int32_t)(a) >> 3 )
#define UM_IntQ4(a) 	( (int32_t)(a) >> 4 )
#define UM_IntQ5(a) 	( (int32_t)(a) >> 5 )
#define UM_IntQ6(a) 	( (int32_t)(a) >> 6 )
#define UM_IntQ7(a) 	( (int32_t)(a) >> 7 )
#define UM_IntQ8(a) 	( (int32_t)(a) >> 8 )
#define UM_IntQ9(a) 	( (int32_t)(a) >> 9 )
#define UM_IntQ10(a) 	( (int32_t)(a) >> 10 )
#define UM_IntQ11(a) 	( (int32_t)(a) >> 11 )
#define UM_IntQ12(a) 	( (int32_t)(a) >> 12 )
#define UM_IntQ13(a) 	( (int32_t)(a) >> 13 )
#define UM_IntQ14(a) 	( (int32_t)(a) >> 14 )
#define UM_IntQ15(a) 	( (int32_t)(a) >> 15 )//对Q15表示的a取整:Q15>>15=Q0
#define UM_IntQ16(a) 	( (int32_t)(a) >> 16 )

//GLOBLE_Q 不能低于10，否则很多算法要变
#if  GLOBLE_Q==10

#define UM_QMATH 		10
#define UM_Q(a)   		UM_Q10(a)
#define UM_IQMul(a,b) 	UM_IQ10MulQ10(a,b)
#define UM_IntQ(a)    	UM_IntQ10(a)
#define UM_QToF(a)    	UM_Q10ToF(a)


#elif GLOBLE_Q==15

#define UM_QMATH		15
#define UM_Q(a)			UM_Q15(a)			//将普通数据转换成Q15格式,这里将Q15化后的数据用int32_t表示出来，因此Q15化数据表示范围：-65536~65535.9999
#define UM_IQMul(a,b)	UM_IQ15MulQ15(a,b)	//两个Q15数相乘，并返回Q15-->Q15=Mul(Q15,Q15)
#define UM_IntQ(a)		UM_IntQ15(a)		//对Q15格式的数据a取整：Q15>>15=Q0！
#define UM_QToF(a)		UM_Q15ToF(a)		//Q15 to Float

#endif	//end if GLOBLE_Q==15

#ifndef UM_QMATH

#error();

#endif

#define UM_abs(a)       ( (a)>0?(a):-(a))    
#define UM_Sign(a)      ( (a)>0?1:-1)  

/*************************角度单位转换************************/
#define	PI		3.1415926

#define UM_Rad2Round(a)	((a) * 0.159155)		//弧度转换为转：Round= Rad/2/pi
#define UM_Round2Rad(a)	((a) * 6.283185)		//弧度转换为转：Rad= Round*2*pi

#define UM_Rad2Deg(a)	((a) * 57.2958)			//弧度转换为度:	Deg=Rad*57.2958
#define UM_Deg2Rad(a)	((a) * 0.0174533)		//度转换为弧度:	Rad=Deg*0.0174533

#define UM_Rad2Step(a)	((a) * 5215.19)			//弧度转换为step值:	Step=Rad*5215.19
#define UM_Step2Rad(a)	((a) * 0.0001917476)	//step值转换为弧度:	Rad=Step*0.0001917476

#define UM_Deg2Step(a)	((a) * 91.0222)			//度转换为step值:	Step=Deg*91.0222
#define UM_Step2Deg(a)	((a) * 0.0109863)		//step值转换为度:	Deg=Step*0.0109863

#define UM_RadQ152Step(a)	((a) * 0.159155)	//弧度(Q15)转换为step:	Step=Rad_Q15*0.159155
#define UM_Step2RadQ15(a)	((a) * 6.283185)	//step转换为弧度(Q15):	Rad_Q15=Step*6.283185
/*************************************************************/
/*clarke_park模块是一个坐标变换模块，这个模块是一个数学工具，*/
/*任何其他模块都可以调用这个公用模块。						 */
/*************************************************************/
#include "stdint.h"
#include "math.h"

typedef struct
{
	intQ15 axis_A;
	intQ15 axis_B;
	intQ15 axis_C;

}Frame_ABC_TypeDef;

typedef struct
{
	intQ15 axis_alpha;
	intQ15 axis_beta;

}Frame_AlphaBeta_TypeDef;

typedef struct
{
	intQ15 axis_d;
	intQ15 axis_q;

}Frame_DQ_TypeDef;

void Clarke_Current_Transform(Frame_AlphaBeta_TypeDef* frame_alphabeta, Frame_ABC_TypeDef* frame_ABC);	//Coordinate3_static-->Coordinate2_static
void RevClarke_Current_Transform(Frame_ABC_TypeDef* frame_ABC, Frame_AlphaBeta_TypeDef* frame_alphabeta);//Coordinate2_static-->Coordinate3_static
void Park_Transform(Frame_DQ_TypeDef* frame_dq, Frame_AlphaBeta_TypeDef* frame_alphabeta,
	intQ15 sin_theta, intQ15 cos_theta);																//Coordinate2_static-->Coordinate2_rotary
void RevPark_Transform(Frame_AlphaBeta_TypeDef* frame_alphabeta, Frame_DQ_TypeDef* frame_dq,
	intQ15 sin_theta, intQ15 cos_theta);																//Coordinate2_rotary-->Coordinate2_static
void RevPark_CircleLimitation(Frame_DQ_TypeDef* frame_dq);



float UM_GetE(float *data, uint16_t n);

float UM_GetD2E(float *data, uint16_t n);

void UM_ArryFrontMove(float * data , uint16_t n, uint16_t move);

float UM_MATH_GetRecip(uint32_t x);

float UM_F32Sqrt(float number); //快速开平方根算法，算法时间估计1us
float UM_F32SqrtRecip( float number ); //快速开平方根的倒数算法，算法时间估计1us
int32_t UM_Q7SqrtRecip( intQ7 number);//快速开整数平方根的倒数算法，算法时间估计1us
uint32_t UM_IntSqrt(uint32_t x);

uint32_t UM_GetRecQ15(uint32_t x); // 求倒数，查表法

uint32_t UM_atanStep(int32_t y, int32_t x) ;

void UM_SinCosQ15(int32_t theta_step, intQ15 * sinVal, intQ15 * cosVal);

void UM_Sin3Balance(int32_t the, intQ15 *sin, intQ15 *sin240, intQ15 *sin120);
//返回值0-3；
int32_t  UM_Get_Quadrant(int32_t theta);

//严格判断theta 象限：0-3, 5表示不严格
int32_t UM_StrictSector(int32_t theta);

typedef struct
{
      /*************
		输入
	 *************/
	int32_t 	num;			//f0*2^N--分子
	int32_t 	den;  			//fs--分母
	int32_t 	round_ea_step;	//N=16-->2^N=32768.一个电角度周期分为32768step.
	
	/*************
	 中间变量
	*************/
	int32_t 	integer;		//整数，即商
	int32_t 	fraction;		//小数，即余数
	int32_t 	left;			//算法run时的余数
	int8_t		dir;			//DDS生成方向
	
	/*************
	 输出变量
	*************/
	int32_t		sum_out;		//当前输出总和，范围：0~num
	int32_t 	out;			//输出，算法run时的商
	
	/* add */
	int32_t 	round_flag;		//跨越边界0的flag
}UM_DDS_Type;

void DDS_Init(UM_DDS_Type * p, int32_t num, int32_t den);
int32_t DDS_Run(UM_DDS_Type * p);
void DDS_Set(UM_DDS_Type * p, int32_t num, int32_t den);
void DDS_Set_Dir(UM_DDS_Type * p, int8_t dir);
void DDS_Rst(UM_DDS_Type * p);

void UM_DDS_Init(UM_DDS_Type * p ,int32_t num , int32_t  den);
void UM_DDS_Run(UM_DDS_Type * p ,uint8_t dir);

/******************** Transplant from Zhoufan's DDS (Start)*******************/
typedef struct
{
     /*************
	 参数
	 *************/
	int32_t prm_num; //分子
	int32_t prm_den;  	//分母，即除数
	int32_t prm_max_out;
	int32_t prm_min_out;
	/*************
	 中间变量
	*************/
	int32_t prm2_section;
	int32_t integer;		//整数，即商
	int32_t remainder;		//小数，即余数
	int32_t left;			//算法run时的余数
	
  /*************
	 输出变量
	*************/
  int32_t out;				//输出，算法run时的商
	
	
}QM_DDS_TypeDef;

void QM_DDS_Init(QM_DDS_TypeDef * p ,int32_t num , int32_t  den , int32_t max_out , int32_t min_out);
int32_t QM_DDS_Run(QM_DDS_TypeDef * p ,int32_t dt_val);
void QM_DDS_Rst(QM_DDS_TypeDef * p);


typedef struct _SinGenParm
{
	float amp;			//正弦幅值
	float frq;			//正弦频率
	int32_t cycles;		///< sin cyles is going to run.
}SinGenParam_Typedef;
typedef struct
{
	float amp;			//正弦幅值
	float frq;			//正弦频率
	int32_t cycles;		///< sin cyles is going to run.
	
	int8_t cmd;			//cmd: 1--Init, 2--run
	
	/* 实现手段--DDS */
	int32_t DDS_frq;	//DDS基频，若取20K,则DDS在ADC中断中运行；若取10K，则DDS在SERVO中断中运行。这里特别要注意DDS_frq和Sin_Gen调用的位置！
	UM_DDS_Type DDS_st;
	
	/* Genover flag */
	int8_t gen_over_flag;
	
	/* 正弦输出 */
	float sinval; 
	float cosval;
	
	/* 方波输出 */
	int32_t sqr_unit;	//单位方波信号输出
	int32_t o_sqr_amp;	//方波输出
}SinGen_Typedef;

void Sin_Gen_Init(SinGen_Typedef *that);
void Sin_Gen_Period(SinGen_Typedef *that);
void Sin_Gen_Set(SinGen_Typedef *that, float amp, float frq, int32_t cycles);
void Sin_Gen_Rst(SinGen_Typedef *that);
/******************** Transplant from Zhoufan's DDS (End)*******************/

/**
* timer
**/
/* servo timer base on BHAL servo timer set. eg. When we set
   BHAL servo timer's tick = 100us, Here does the servo timer.*/
typedef struct _MyTimer
{
	uint32_t arr;						///< reload value
	
	uint32_t counter;					///< counter
	uint32_t elapsed_val;				///< time set
	
	uint32_t elapsed_flag;				///< flag indicate that time set is elapsed
}Timer_Typedef;

void initTimer(Timer_Typedef *timer);
void rstTimer(Timer_Typedef *timer);
void setTimerElapsedVal(Timer_Typedef *timer, uint32_t time);
uint32_t getTimerCnt(Timer_Typedef *timer);
uint32_t getTimerElapsedVal(Timer_Typedef *timer);
uint32_t getTimerElapsedFlag(Timer_Typedef *timer);
void runTimerPeriod(Timer_Typedef *timer);

/* Filter */
#define FILTER_N		10
int32_t Filter_MovingAverage(int32_t *pFilterBuf, const int32_t CurVal, const int32_t n);

#endif
