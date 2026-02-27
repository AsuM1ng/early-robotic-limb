#include <math.h>
#include <stdlib.h>
#include "ThreeAxisForceTransform.h"


ThreeAxis_Force_t HandMuscleForce;      /*握力传感器只有一维数据*/
ThreeAxis_Force_t WristForce;           /*腕关节力传感器    S3*/
ThreeAxis_Force_t ForearmForce;         /*前臂力传感器      S2*/
ThreeAxis_Force_t UpperarmForce;        /*上臂力传感器      S1*/


int16_t FindMaxForce(int16_t x_Force, int16_t y_Force, int16_t z_Force);


/** \brief 计算力所处的象限，将平面空间分为4个象限，
                |
       II       |       I
                |
________________|________________
                |
       III      |       IIII
                |
                |
 * \param int16_t x_Force 力在x轴的大小
 * \param int16_t z_Force 力在y轴的大小
 * \return ForceQuad_en 力的象限
 *
 */   
ForceQuad_en getForceQuad(int16_t x_Force, int16_t z_Force)
{
    //ForceQuad_en forceQuad = FirstQuad;
    if(z_Force > 0)
    {
        if(x_Force > 0)
        {
            return FirstQuad;
        }
        else
        {
            return FourthQuad;
        }
    }
    else
    {
        if(x_Force > 0)
        {
            return SecondQuad;
        }
        else
        {
            return ThirdQuad;
        }
    }
}


/*
* 定义：水平方向为X轴，垂直方向为Z轴，与X-Z所构成平面垂直的轴为Y轴，Y轴与机械臂径向平行
*/
void GetForceInHorAndVer(ThreeAxis_Force_t *SensorForce)
{
	int32_t ForceTemp = 0;
	double dfTemp = 0.0;
	double fForce = 0.0;
    double dAngleTemp = 0.0;

	ForceQuad_en forceQuad;

    dfTemp = SensorForce->i16Axis_x_Force;
	dfTemp = dfTemp/SensorForce->i16Axis_z_Force;     /*tan(theta) = x/z*/

    /*反正切函数，计算出外力与x的轴夹角*/
    SensorForce->dfAngle_z_axis = atan(dfTemp)* 180/PI;   /*theta = atan(z/x)*/

    /*根据力在x轴和y轴上投影的正负值，判断力所处的象限*/
    forceQuad = getForceQuad(SensorForce->i16Axis_x_Force, SensorForce->i16Axis_z_Force);

    switch(forceQuad)
    {
    case FirstQuad:
        SensorForce->dfAngle_z_axis = SensorForce->dfAngle_z_axis + 0;
        break;
    case SecondQuad:
    case ThirdQuad:
        SensorForce->dfAngle_z_axis = SensorForce->dfAngle_z_axis + 180;
        break;
    case FourthQuad:
        SensorForce->dfAngle_z_axis = SensorForce->dfAngle_z_axis + 360;
        break;
    default :
        break;
    }
    /*根据力与x轴的夹角和x轴与水平方向旋转的夹角计算力与水平方向的夹角*/
	SensorForce->dfAngleForce_h = SensorForce->dfAngle_z_axis + SensorForce->dfAngle_Sensor;
	if(SensorForce->dfAngleForce_h < 0)
    {
        SensorForce->dfAngleForce_h = 360 + SensorForce->dfAngleForce_h;
    }

    /*计算力的绝对值*/
	ForceTemp = SensorForce->i16Axis_x_Force * SensorForce->i16Axis_x_Force +\
                SensorForce->i16Axis_z_Force * SensorForce->i16Axis_z_Force;
	fForce = sqrt((double)ForceTemp);

    SensorForce->i16axis_h_Force = (int16_t) (fForce * cos(PI * SensorForce->dfAngleForce_h/180));
    SensorForce->i16axis_v_Force = (int16_t) (fForce * sin(PI * SensorForce->dfAngleForce_h/180));
}

/** \brief 以y轴为作动轴，即坐标系绕y轴旋转，和/或y轴有俯仰角时，
 *          计算外力与水平方向力的分量和与垂直方向力的分量
 * \param double RotationAngle 坐标系绕y轴的旋转角
 * \param double PitchAngle y轴的俯仰角
 * \return void
 * \author Fang Zh
 * \date 2019/06/12
 */
void ThreeAxisForceTrans(ThreeAxis_Force_t *SensorForce, double RotationAngle, double PitchAngle)
{
//    uint32_t u32TempData = 0;
    double lfTempForce = 0.0;
    double lfTempData1 = 0.0;
    double lfTempData2 = 0.0;
    double lfTempData3 = 0.0;
    int16_t i16symbol = 1;

//    /*计算力的绝对值*/
//    u32TempData = SensorForce->i16Axis_x_Force * SensorForce->i16Axis_x_Force +\
//                    SensorForce->i16Axis_y_Force * SensorForce->i16Axis_y_Force +\
//                    SensorForce->i16Axis_z_Force * SensorForce->i16Axis_z_Force;

//    lfTempForce = sqrt(u32TempData);

//    /*力与x轴的夹角*/
//    if((abs(SensorForce->i16Axis_x_Force)/100) > 2)
//    {
//        SensorForce->dfAngle_x_axis = acos(SensorForce->i16Axis_x_Force/lfTempForce) * 180 / PI;
//    }
//    else
//    {
//        SensorForce->dfAngle_x_axis = 0;
//    }

//    /*力与y轴的夹角*/
//    if((abs(SensorForce->i16Axis_y_Force)/100) > 2)
//    {
//        SensorForce->dfAngle_y_axis = acos(SensorForce->i16Axis_y_Force/lfTempForce) * 180 / PI;
//    }
//    else
//    {
//        SensorForce->dfAngle_y_axis = 0;
//    }

//    /*力与z轴的夹角*/
//    if((abs(SensorForce->i16Axis_z_Force)/100) > 2)
//    {
//        SensorForce->dfAngle_z_axis = acos(SensorForce->i16Axis_z_Force/lfTempForce) * 180 / PI;
//    }
//    else
//    {
//        SensorForce->dfAngle_z_axis = 0;
//    }

    /*计算力的水平分量*/    /*算法可能不准确，需要继续测试验证*/
    /*计算Z轴分量*/
    lfTempForce = cos(RotationAngle * PI/180) * (-SensorForce->i16Axis_z_Force)/100;
    /*计算X轴分量，并与Z轴分量求和*/
    SensorForce->i16axis_h_Force = lfTempForce + (sin(RotationAngle * PI/180) * SensorForce->i16Axis_x_Force)/100;
    
    /*计算力的垂直分量*/
    lfTempData1 = (SensorForce->i16Axis_x_Force * cos(RotationAngle * PI/180))/100;
    lfTempData2 = (SensorForce->i16Axis_y_Force * sin(PitchAngle * PI/180))/100;
    lfTempData3 = (SensorForce->i16Axis_z_Force * cos((90 - RotationAngle) * PI/180))/100;
    
    lfTempForce = lfTempData1 * lfTempData1 + \
                    lfTempData2 * lfTempData2 + \
                    lfTempData3 * lfTempData3;
    SensorForce->i16axis_v_Force = (int16_t) sqrt(lfTempForce);
    
    /*垂直分量的力的方向*/
    i16symbol = FindMaxForce(SensorForce->i16Axis_x_Force, 
                            SensorForce->i16Axis_y_Force, 
                            SensorForce->i16Axis_z_Force);
    SensorForce->i16axis_v_Force = SensorForce->i16axis_v_Force * i16symbol;
}


/*找出三个力中的最大力，并返回其符号*/
int16_t FindMaxForce(int16_t x_Force, int16_t y_Force, int16_t z_Force)
{
    if(abs(x_Force) > abs(y_Force))
    {
        if(abs(x_Force) > abs(z_Force))
        { /*x 轴最大*/
            if(x_Force > 0)
                return 1;
            else
                return -1;
        }
        else
        {   /*z 轴最大*/
            if(z_Force > 0)
                return 1;
            else
                return -1;
        }
    }
    else
    {
        if(abs(y_Force) > abs(z_Force))
        {   /*y 轴最大*/
            if(y_Force > 0)
                return 1;
            else
                return -1;
        }
        else
        {   /*z 轴最大*/
            if(z_Force > 0)
                return 1;
            else
                return -1;
        }
    }
}



/** \brief 计算手腕力传感器和前臂力传感器的俯仰角
 * \param double rotationAngle 坐标系绕y轴的旋转角
 * \param double includedAngle 两手臂S1和S2的夹角
 * \return double 传感器的俯仰角
 * \author Fang Zh
 * \date 2019/06/18
 */
double GetForceSensorPitchAngle(double rotationAngle, double includedAngle)
{
    double ldTempAngle = 0.0;
    double ldTempData = 0.0;

    if(90.0 == includedAngle)
    {
        ldTempAngle = 89.9;
    }
    else
    {
        ldTempAngle = includedAngle;
    }

    ldTempData = tan(ldTempAngle * PI/180) * sin(rotationAngle * PI/180);

    ldTempData = atan(ldTempData) * 180/PI;

    return ldTempData;
}





