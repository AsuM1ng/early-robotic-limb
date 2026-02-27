#include "filter.h"
/**********************************************************
* @FuncName: Filter_LPF
* @Param:
* @Return:
* @Brief: 一阶低通滤波器, Y[n]=a*X[n]+(1-a)*Y[n-1],
*			a为滤波器系数,表示本次采样值占比。a: 0~1.
*			一阶低通滤波器截止频率：fc = a/((2*pi)*(1-a)*T).
* @addtion: 设计时根据需要的截止频率fc和采样时间T确定滤波系数a。
**********************************************************/
int32_t Filter_LPF(int32_t x_cur, float k)
{
	static int32_t x[6], index=0;
	static int32_t y_cur = 0, y_prev = 0;
	//可以使用队列实现
	
	y_cur = k * x_cur + (1-k)*y_prev;
	
	return (y_cur);
}