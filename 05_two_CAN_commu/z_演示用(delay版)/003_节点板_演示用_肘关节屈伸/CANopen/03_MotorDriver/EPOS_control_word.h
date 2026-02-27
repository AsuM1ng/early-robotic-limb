#ifndef _EPOS_CONTROL_WORD_H
#define	_EPOS_CONTROL_WORD_H

extern unsigned char activ_PPM[5];
extern unsigned char Profile_Velocity[8];
extern unsigned char Profile_Acceleration[8];
extern unsigned char Profile_Deceleration[8];
extern unsigned char Posotion_Target[8];
extern unsigned char SSI_immediately[6];

//Œª“∆µΩ0Œª÷√
extern unsigned char Move_to_Zero[8];

extern unsigned char Actual_Position[4];

extern unsigned char activ_PVM[5];
extern unsigned char EPOS_disable[6];
extern unsigned char EPOS_enable[6];
extern unsigned char shoulder_target_120rpm[8];
extern unsigned char shoulder_target_n_120rpm[8];
extern unsigned char Halt[6];
extern unsigned char shoulder_start_forward_20rpm[8];
extern unsigned char shoulder_circulate_forward_90rpm[8];
extern unsigned char shoulder_circulate_reverse_90rpm[8];


#endif


