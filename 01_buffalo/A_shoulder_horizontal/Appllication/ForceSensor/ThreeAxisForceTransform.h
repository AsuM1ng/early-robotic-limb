#ifndef THREEAXISFORCESENSOR_H_INCLUDED
#define THREEAXISFORCESENSOR_H_INCLUDED

#include "stdint.h"
#include "TestSlave.h"
#include "canfestival.h"

#define PI      3.1415926

typedef enum
{
    FirstQuad   = 1,
    SecondQuad  = 2,
    ThirdQuad   = 3,
    FourthQuad  = 4
}ForceQuad_en;  /*力在平面中的象限*/


typedef struct
{
    char *cpSensorName;             /*传感器名称*/
    
    uint16_t u16Axis_x_Range;       /*力传感器x轴测量范围*/
    uint16_t u16Axis_y_Range;       /*力传感器y轴测量范围*/
    uint16_t u16Axis_z_Range;       /*力传感器z轴测量范围*/

	int16_t i16Axis_x_Force;	    /*X轴方向力*/
	int16_t i16Axis_y_Force;	    /*y轴方向力*/
	int16_t i16Axis_z_Force;	    /*z轴方向力*/
	
	int16_t i16axis_h_Force;	    /*水平方向的力*/
	int16_t i16axis_v_Force;	    /*垂直方向的力*/
    
    int16_t i16Axis_v_ForceOffset;  /*垂直方向力的偏移量，由人手臂的自然重力产生*/
	
	double dfAngle_Sensor;		    /*传感器绕Y轴旋转的角度*/
    double dfAngle_Pitch;           /*传感器Y轴俯仰角*/

	double dfAngleForce_h;		    /*外力水平方向夹角*/
    double dfAngleForce_v;		    /*外力垂直方向夹角*/
    
	double dfAngle_x_axis;		    /*外力与X轴的夹角*/
    double dfAngle_y_axis;          /*外力与y轴的夹角*/
    double dfAngle_z_axis;          /*外力与z轴的夹角*/
}ThreeAxis_Force_t;


extern ThreeAxis_Force_t HandMuscleForce;   /*握力传感器*/
extern ThreeAxis_Force_t WristForce;        /*安装在手腕位置的力传感器*/
extern ThreeAxis_Force_t ForearmForce;      /*安装在前臂的力传感器*/
extern ThreeAxis_Force_t UpperarmForce;     /*安装在上臂旋转位置的力传感器*/


void GetForceInHorAndVer(ThreeAxis_Force_t *SensorForce);
void ThreeAxisForceTrans(ThreeAxis_Force_t *SensorForce,
                        double RotationAngle, double PitchAngle);
double GetForceSensorPitchAngle(double rotationAngle, double includeAngle);

#endif // THREEAXISFORCESENSOR_H_INCLUDED
