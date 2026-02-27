//

#include "MC_ProfilePosition.h"

/**********************************************************
* @FuncName: PTP_Init
* @Param: *PTP_struct
* @Return: None
* @Brief: PTP初始化
* @addtion: 
**********************************************************/
void PTP_Init(PTP_Typedef *PTP_struct, float pos_Ts)
{
	memset(PTP_struct, 0, sizeof(PTP_Typedef));
	PTP_struct->prm_PTP_mode = 0;								//0-->绝对模式；1-->增量模式
	PTP_struct->prm_pos_Ts = 0.001;								//位置参考生成器的采样周期:0.001s
	PTP_struct->pending_flag = 1;								//初始处于挂起状态
	
	PTP_struct->pos_radius = 500;	
}

/**********************************************************
* @FuncName: PTP_Clear_Output
* @Param: *PTP_gen_struct, *PTP_reg_struct
* @Return: None
* @Brief: PTP清除输出(Generator和Regulator)
* @addtion:
**********************************************************/
void PTP_Clear_Output(PTP_Typedef *PTP_struct)
{
	PTP_struct->o_vel_ref = 0;
	PTP_struct->pending_flag = 1;								//清除后默认回到挂起状态
	PTP_struct->update_flag = 0;									//默认目标位置未更新.
}

//只有从PTP模式切出去的时候才要将Vref清除
void PTP_Set_value(PTP_Typedef *PTP_struct, int32_t acc, int32_t dec, int32_t vel_limit,
					int32_t pos_start, int32_t pos_end)
{
	/************** 限制 ***************/
	pos_end = pos_end >  8000000 ?  8000000 : pos_end;		//绝对位置限制：8000,000cnt = 4000r      --> 机械输出轴:4000r/400 = +10r
	pos_end = pos_end < -8000000 ? -8000000 : pos_end;		//绝对位置限制：-8000,000cnt = -4000r 	 --> 机械输出轴:-4000r/400 = -10r
	acc = acc >  1600000 ?  1600000 : acc;					//加速度限制：1600,000cnt/s^2 = 800r/s^2 --> 机械输出轴:800r/400 = +2r/s^2 = 12.5rad/s^2
	acc = acc < -1600000 ? -1600000 : acc;					//加速度限制：1600,000cnt/s^2 = 800r/s^2 --> 机械输出轴:800r/400 = +2r/s^2 = 12.5rad/s^2
	dec = dec >  1600000 ?  1600000 : dec;					//加速度限制：1600,000cnt/s^2 = 800r/s^2 --> 机械输出轴:800r/400 = +2r/s^2 = 12.5rad/s^2
	dec = dec < -1600000 ? -1600000 : dec;					//加速度限制：1600,000cnt/s^2 = 800r/s^2 --> 机械输出轴:800r/400 = +2r/s^2 = 12.5rad/s^2
	vel_limit = vel_limit >  (NORMINAL_SPEED/60*ENCODER_PPR) ?  0 : vel_limit;	                            //速度限制：286,720cnt/s = 70r/s = (4250r/m) / (60s/m)       --> 机械输出轴:70r/120 = +0.5833r/s = 3.6633rad/s
	vel_limit = vel_limit < (-NORMINAL_SPEED/60*ENCODER_PPR) ? (-NORMINAL_SPEED/60*ENCODER_PPR) : vel_limit;	//速度限制：286,720cnt/s = 70r/s = (4250r/m) / (60s/m)	     --> 机械输出轴:70r/120 = -0.5833r/s =-3.6633rad/s
	
	/* 初始化PTP运动参数 */
	PTP_struct->i_acc = acc;
	PTP_struct->i_dec = dec;
	PTP_struct->i_vel_limit = vel_limit;
	PTP_struct->i_pos_start = pos_start;
	PTP_struct->i_pos_end = pos_end;
	
	/************ 计算加减速度增量 ***********/	/* acc至少1000cnt/s^2，否则delta_acc=0 */
	PTP_struct->delta_acc = PTP_struct->i_acc * PTP_struct->prm_pos_Ts;
	PTP_struct->delta_dec = PTP_struct->i_dec * PTP_struct->prm_pos_Ts;
	
	PTP_struct->pending_flag = 0;				//取消挂起，此位置也表示到达的意思
	PTP_struct->update_flag = 1;				//更新参数标志位
}

void PTP_Input(PTP_Typedef *PTP_struct, int32_t pos_fdbk, int32_t vel_fdbk)
{
	PTP_struct->i_pos_fdbk = pos_fdbk;
	PTP_struct->i_vel_fdbk = vel_fdbk;
}

int32_t ob_err_pos = 0;
int32_t ob_pos_dec_rqst = 0;
int32_t ob_err_sub_dec = 0;
int32_t tst_ptp_posref = 0;
//本程序有两种参考轨迹生成思路: 1基于参考速度vel_ref;2基于实际速度vel_fdbk.
//问题：
//突然修改位置目标值pos_end时，基于的起始位置很关键！如果基于当前的实际位置，那么由于
//在PTP运行时生成的pos_ref和实际的pos_fdbk始终有一个差值，约pos_err=800，当突然改变pos_end，即会
//读取当前的pos_fdbk为起始位置，并赋给pos_ref，相当于pos_ref瞬间被拉低到和pos_fdbk一样，
//再经过一次generator,那么产生的pos_err = pos_ref-pos_fdbk = vref*Ts=100左右,pos_err=800-->100，
//因此赋给PID位置控制器的误差突减，导致其算出来的v_ref突降！！！
void PTP_Generator(PTP_Typedef *PTP_struct)
{
	int32_t err_pos = PTP_struct->i_pos_end - PTP_struct->i_pos_fdbk;		
	int32_t err_pos_abs = err_pos > 0 ? err_pos : -err_pos;				//对err_pos取绝对值
	
	int32_t vel_ref = PTP_struct->o_vel_ref ;							//当前参考速度
	int32_t vel_ref_abs = vel_ref > 0 ? vel_ref : -vel_ref;				//速度绝对值
	int32_t vel_tmp_abs = vel_ref_abs;									//记录当前abs_vel
	
	int32_t pos_dec_rqst_abs = 0;										//从当前位置以设定减速度进行减速需要的减速距离的绝对值
	
	PTP_struct->err_pos = err_pos;										//更新全局位置误差
	
	/* 更新PTP参考位置 -- 来自异步set */ /* 下面几句可以放在set接口中!（解决异步set时突然阶跃式的修改pos_ref的问题.） */
	if(PTP_struct->update_flag)											//set目标位置pos_end后，才将当前参考位置更新为当前的起始位置.
	{																	
		PTP_struct->update_flag = 0;
		if(0 == PTP_struct->o_vel_ref)									//处于目标位置，此时速度为0，使用当前反馈位置pos_fdbk更新输出参考o_pos_ref。
		{
			PTP_struct->o_pos_ref = PTP_struct->i_pos_fdbk;
		}
		//如果没有处于目标位置，即在途中，此时速度为非0值，考虑到pos_ref和pos_fdbk有一定的差和pos_ref的连续性，因此不用pos_fdbk更新pos_ref!
	}
	
	/* 计算当前参考速度下减速，所需位置增量 */
	pos_dec_rqst_abs = (int64_t)vel_ref * (int64_t)vel_ref / PTP_struct->i_dec / 2;	
	
	/* 4种情况:
		1.始终正向运行，此时err_pos>pos_radius且vref>0;
		2.正向运行时，突然接到set指令修改pos_end，并且反向,从set指令开始,err_pos<-pos_radius且vref>0;
		3.始终反向运行，此时err_pos<-pos_radius且vref<0;
		4.反向运行时，突然接到set指令修改pos_end，并且反向,从set指令开始,err_pos>pos_radius且vref<0;*/
	/* 注意：1.err_pos方向的确定主要是为了确定运动方向，即确定vel_limit该不该加负号; */
//	if(err_pos > PTP_struct->pos_radius)			//误差为正: vref>0和vref<0
    if(err_pos > 140)			//误差为正: vref>0和vref<0
	{
		if(vel_ref > PTP_struct->i_vel_limit)			//当前速度超过了设定速度则减速
		{
			vel_ref -= PTP_struct->delta_dec;
		}
		else if(pos_dec_rqst_abs > err_pos_abs)			//vel_ref <= vel_limit且剩余距离不够减速
		{
			vel_ref -= PTP_struct->delta_dec;
		}
		else if(vel_ref != PTP_struct->i_vel_limit)		//vel_ref <= vel_limit且剩余距离够减速，再判断是否达到设定速度
		{
			vel_ref += PTP_struct->delta_acc;
		}
		
		PTP_struct->o_vel_ref = vel_ref;
		PTP_struct->o_pos_ref += vel_ref * PTP_struct->prm_pos_Ts;
	}
//	else if(err_pos < -PTP_struct->pos_radius)		//误差为负: vref>0和vref<0
    else if(err_pos < -140)		//误差为负: vref>0和vref<0
	{
		if(vel_ref < -PTP_struct->i_vel_limit)		//当前速度超过设定速度(这里比较的是绝对值)
		{
			vel_ref += PTP_struct->delta_dec;
		}
		else if(pos_dec_rqst_abs > err_pos_abs)		//当前速度未超过设定速度且剩余距离不足够减速则减速
		{
			vel_ref += PTP_struct->delta_dec;
		}
		else if(vel_ref != -PTP_struct->i_vel_limit)	//剩余距离足够减速且当前速度未到达设定速度
		{
			vel_ref -= PTP_struct->delta_acc;
		}
		
		PTP_struct->o_vel_ref = vel_ref;
		PTP_struct->o_pos_ref += vel_ref * PTP_struct->prm_pos_Ts;
	}
	else											//当前位置很接近目标位置了, -radius < err_pos < radius.
	{
		/* 切换成PID_P，用于静态刚度. */ 
		/* 静态情况下，如果PTP输出pos_ref，则静态时，由于处于PTP模式，轨迹仍在规划，因此转动一小点角度范围内，位置容易小震动，不利于静态刚度！
						如果PTP输出vel_ref，则静态刚度主要依赖PTP加减速度的大小！
			因此，一定要在PTP模式位置快要到达时(pos_radius)，将位置切换为PID_POS模式.*/
		/* 经试验，PTP输出参考位置pos_ref，则速度跟随性要差一些，但是动态刚度较好;
				   PTP输出参考速度vel_ref，则速度跟随性要更好一些，但是动态刚度较差.*/
		PTP_struct->pending_flag = 1;				//防止PTP到零位置后继续使用PTP模式，可防过冲，强制让PTP切换到PID_P模式，使得PTP静态刚度提高.
		PTP_struct->o_vel_ref = 0;
		PTP_struct->o_pos_ref = PTP_struct->i_pos_end;
	}
}


