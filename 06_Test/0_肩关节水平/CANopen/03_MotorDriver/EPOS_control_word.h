#ifndef _EPOS_CONTROL_WORD_H
#define	_EPOS_CONTROL_WORD_H

/****************************/
/********  控制操作  ********/
/****************************/
extern unsigned char activ_EPOS[2];
extern unsigned char activ_PPM[5];
extern unsigned char activ_PVM[5];
extern unsigned char EPOS_disable[6];
extern unsigned char EPOS_enable[6];
extern unsigned char SSI_immediately[6];
extern unsigned char Halt[6];
extern unsigned char Error_Clear[6];


/****************************/
/********  参数设置  ********/
/****************************/
extern unsigned char Profile_Velocity[8];
extern unsigned char Profile_Acceleration[8];
extern unsigned char Profile_Deceleration[8];
extern unsigned char shoulder_start_forward_20rpm[8];
extern unsigned char shoulder_circulate_forward_90rpm[8];
extern unsigned char shoulder_circulate_reverse_90rpm[8];
extern unsigned char Posotion_Target[8];
extern unsigned char Move_to_Zero[8];
extern unsigned char shoulder_target_120rpm[8];
extern unsigned char Max_Acceleration[8];

extern unsigned char Target_Position[8];
extern unsigned char Target_Velocity[8];
extern unsigned char Target_Acceleretion[8];
extern unsigned char Target_Deceleretion[8];


/****************************/
/********  参数读取  ********/
/****************************/
extern unsigned char Read_Actual_Position[4];
extern unsigned char Read_Powersupply[4];
extern unsigned char Read_Errorstate[4];
extern unsigned char Read_EPOS_Controlword[8];
extern unsigned char Read_Target_Velocity[4];
extern unsigned char Read_Profile_Acceleration[4];
extern unsigned char Read_Profile_Deceleration[4];
extern unsigned char Read_EPOS_Statusword[8];


#endif


