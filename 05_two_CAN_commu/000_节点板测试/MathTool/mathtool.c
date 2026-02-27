#include "main.h"
#include "math.h"

u8 back0time(int position)
{
	u16 time;
	u32 pos = fabs(position);
	
	if(pos <= 4096)
		time = 10;
	else if(pos > 4096)
		time  = 10 * ( (pos-4096)/4096/2+1 );
	
	return time;
}

//已验证
int angle_to_inc_120(int angle)			//整体度数换算成电机的实际角度，减速比120的部件用，肩关节水平、肩关节垂直、肘关节旋转
{
	return (int)trunc(angle * 120 * 4096 / 360);
}

//已验证
int angle_to_inc_50(int angle)			//整体度数换算成电机的实际角度，减速比120的部件用，肩关节水平、肩关节垂直、肘关节旋转
{
	return (int)trunc(angle * 50 * 4096 / 360);
}



//已验证
//32位16进制分成4段，方便输入电机数组中。速度和位置都是有符号的，因此统一计算有符号数
void num_to_motor_data(int num)
{
		MD.MotorData[0] = (u8)((u32)num & 0x000000FF);
		MD.MotorData[1] = (u8)(((u32)num & 0x0000FF00) >> 8);
		MD.MotorData[2] = (u8)(((u32)num & 0x00FF0000) >> 16);
		MD.MotorData[3] = (u8)(((u32)num & 0xFF000000) >> 24);
}



