#include "MotionPathProcess.h"
#include "JointControl.h"
#include <stdlib.h>
#include "chipHAL_GPIO.h"


MotionIterm_t TestBuff[MAX_PVT_BUFFER];
float fCycle = 0.0;
uint32_t IntCount = 0;
///*Note!! 路径记录的三元素分别是：角度/速度/时间，在做PVT路径恢复时，需要将角度转换成脉冲位置(Position)*/

static void RecordandAssignment(PVT_Table_Typedef * targetBuff,
                                PVT_Table_Typedef *sourceBuff,
                                uint16_t targetIndex,
                                uint16_t sourceIndex)
{
    targetBuff[targetIndex].position    = sourceBuff[sourceIndex].position;
    targetBuff[targetIndex].velocity    = sourceBuff[sourceIndex].velocity;
    targetBuff[targetIndex].time        = sourceBuff[sourceIndex].time;
}

void MotionPathRecorder(void)
{
    float fTime = 0.000;
    uint16_t u16index = 0;
    
    if((JointCtrl.MotionPath.u32TimeCount <= (RECORDTIME*100)) &&
        (JointCtrl.MotionPath.u16TotalPoint < MAX_PVT_OBJECT) && /*缓冲区满*/
        (eRecordStop != JointCtrl.MotionPath.u16CtrlWowrds))  /*主控命令停止记录*/
    {
        /*计时，开始记录后持续计时*/
        JointCtrl.MotionPath.u32TimeCount += 1;

        /*间隔10mS，将行程的每个点都记录下来*/
        JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].position = JointCtrl.Motion.i32JointActualPosition;
        
        JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].velocity = EposCtrl.i32ServoActualVelocity;
        
        fTime = (float)JointCtrl.MotionPath.u32TimeCount;
        JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].time = fTime/100;
        
        TestBuff[JointCtrl.PVT_struct.u16P_index].i32Angle = JointCtrl.Motion.i32JointActualAngle;
        TestBuff[JointCtrl.PVT_struct.u16P_index].fTime = fTime/100;
#if(PRINTF_EN)
        printf("%d\t%d\t%d\t%f\t%d\r\n",  JointCtrl.PVT_struct.u16P_index,
                JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].position,
                JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].velocity,
                JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index].time,
                TestBuff[JointCtrl.PVT_struct.u16P_index].i32Angle);
        #endif
        JointCtrl.PVT_struct.u16P_index += 1;
    }

    
    /*记满一个buff 256个点数据，筛选一次*/
    if(JointCtrl.PVT_struct.u16P_index >= MAX_PVT_BUFFER)
    {
        for(u16index = 0; u16index < JointCtrl.PVT_struct.u16P_index; u16index++)
        {
            /*速度低于1Hz时，认为是速度为0，记录特殊点*/
            if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index].velocity) < THRESHOLD_V)
            {
                /*第一个符合要求的点，直接存入PVT表*/
                if(JointCtrl.MotionPath.bFirstPoint)
                {
                    JointCtrl.MotionPath.bFirstPoint = FALSE;
                    JointCtrl.MotionPath.u16PathIndex = 0;
                    RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                    JointCtrl.MotionPath.u16PathIndex += 1;
                    JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                }
                else
                {
                    if((u16index > 0) && (u16index < (MAX_PVT_BUFFER - 1)))
                    {
                        if(((abs(JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V) &&
                            (abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V)) ||
                            (abs(JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V) ||
                            (abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V))
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
#if(PRINTF_EN)
                            printf("index:%d\r\n",u16index);
                            #endif
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                    else if((MAX_PVT_BUFFER - 1) == u16index)
                    {
                        if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V)
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                    else if(0 == u16index)
                    {
                        if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V)
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                }
            }
        }
        /*Temp_Buff的0元素等于上一次Temp_Buff的最后一个元素*/
        JointCtrl.PVT_struct.u16P_index = 0;
        JointCtrl.PVT_struct.Temp_Buff[0].position    = JointCtrl.PVT_struct.Temp_Buff[MAX_PVT_BUFFER-1].position;
        JointCtrl.PVT_struct.Temp_Buff[0].velocity    = JointCtrl.PVT_struct.Temp_Buff[MAX_PVT_BUFFER-1].velocity;
        JointCtrl.PVT_struct.Temp_Buff[0].time        = JointCtrl.PVT_struct.Temp_Buff[MAX_PVT_BUFFER-1].time;
        
    }
    
    /*轨迹角度缓冲区满时/主控发送停止命令时/记录超时时，程序停止记录轨迹，并对记录的轨迹数据处理*/
    if((JointCtrl.MotionPath.u32TimeCount > (RECORDTIME*100))
        ||(JointCtrl.MotionPath.u16TotalPoint >= (MAX_PVT_OBJECT - 1)) /*缓冲区满*/
        ||(eRecordStop == JointCtrl.MotionPath.u16CtrlWowrds))  /*主控命令停止记录*/
    {
        /*停止记录时，处理缓冲区剩下的数据*/
        if(JointCtrl.PVT_struct.u16P_index >= 1)
        {
            for(u16index = 0; u16index < JointCtrl.PVT_struct.u16P_index; u16index++)
            {
                /*速度低于1Hz时，认为是速度为0，记录特殊点*/
                if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index].velocity) < THRESHOLD_V)
                {
                    if(0 == u16index)
                    {
                        if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V)
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                    else if((MAX_PVT_BUFFER - 1) == u16index)
                    {
                        if(abs(JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V)
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                    else if((u16index > 0) && (u16index < (MAX_PVT_BUFFER - 1)))
                    {
                        if((abs((JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V) &&\
                           (abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V)) ||\
                            (abs((JointCtrl.PVT_struct.Temp_Buff[u16index - 1].velocity) > THRESHOLD_V))||\
                            (abs(JointCtrl.PVT_struct.Temp_Buff[u16index + 1].velocity) > THRESHOLD_V))
                        {
                            RecordandAssignment(JointCtrl.PVT_struct.pvt_table_struct,
                                        JointCtrl.PVT_struct.Temp_Buff,
                                        JointCtrl.MotionPath.u16PathIndex,
                                        u16index);
                            JointCtrl.MotionPath.u16PathIndex += 1;
                            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex;
                        }
                    }
                }
            }
            
            /*第一个点的时间设置为0*/
            JointCtrl.PVT_struct.pvt_table_struct[0].time = 0.00;
            
            /*将示教行程的最后一个点的位置和时间记录下来，速度设置为0*/
            JointCtrl.PVT_struct.pvt_table_struct[JointCtrl.MotionPath.u16PathIndex].position = \
                        JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index - 1].position;
            JointCtrl.PVT_struct.pvt_table_struct[JointCtrl.MotionPath.u16PathIndex].velocity = 0;
            JointCtrl.PVT_struct.pvt_table_struct[JointCtrl.MotionPath.u16PathIndex].time = \
                        JointCtrl.PVT_struct.Temp_Buff[JointCtrl.PVT_struct.u16P_index - 1].time;
            JointCtrl.MotionPath.u16TotalPoint = JointCtrl.MotionPath.u16PathIndex + 1;
        }

#if(PRINTF_EN)
        printf("pvt table Init finished\r\n");
        #endif
        for(uint16_t index = 0; index < JointCtrl.MotionPath.u16TotalPoint; index++)
        {
            JointCtrl.MotionPath.PathBuff[index].i32Angle = encoderPulseConvertToAngle(JointCtrl.Motion.i32JointStanbyPosi,
                                                                JointCtrl.PVT_struct.pvt_table_struct[index].position,
                                                                ENCODER_BIT ,JOINT_GEAR_REDUCTION);
            JointCtrl.MotionPath.PathBuff[index].i32Velocity = JointCtrl.PVT_struct.pvt_table_struct[index].velocity;
            JointCtrl.MotionPath.PathBuff[index].fTime = JointCtrl.PVT_struct.pvt_table_struct[index].time;
#if(PRINTF_EN)
            printf("%d\t", index);
            printf("%d\t", JointCtrl.PVT_struct.pvt_table_struct[index].position);
            printf("%d\t", JointCtrl.PVT_struct.pvt_table_struct[index].velocity);
            printf("%f\t", JointCtrl.PVT_struct.pvt_table_struct[index].time);
            printf("%d\r\n", JointCtrl.MotionPath.PathBuff[index].i32Angle);
            #endif
        }

        JointCtrl.MotionPath.bPathRecordEn = FALSE;
        if(JointCtrl.MotionPath.u16TotalPoint >= 2)
        {
            /*示教路径记录完成标志置位*/
            JointCtrl.MotionPath.bPathRecordFinish = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b1_RecordFinish = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty = FALSE;
        }
        else
        {
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b1_RecordFinish = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b2_PathPointError = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b4_BufferEmpty = TRUE;
        }
        
        JointCtrl.MotionPath.u16StatusWords.Status_bit.b0_RecordStart = FALSE;
        
        /*时间戳清零*/
        JointCtrl.MotionPath.u32TimeCount = 0;
        JointCtrl.MotionPath.u16TimeDelay = 0;
        JointCtrl.MotionPath.u16PathIndex = 0;
    }
}


PVT_Table_Typedef pvttable_hip_debug[65] ={{0,0,0.0}};

void PVT_MotionPathInit(PVT_Typedef *pPVT_struct, float pos_Ts,uint8_t mp, uint16_t startIndex, uint16_t endIndex)
{
	pPVT_struct->MC_period_time = pos_Ts;
	pPVT_struct->MP[3] = mp;
	pPVT_struct->p_start = &pPVT_struct->pvt_table_struct[startIndex];
	pPVT_struct->p_end   = &pPVT_struct->pvt_table_struct[endIndex];
}



int pvt_o_vel_ref=0;
int delta_T_Error=0;

uint8_t PVT_Generator(PVT_Typedef *pvt_struct)
{
	static int P0, V0;
    static float T0;
	static int P1, V1;
    static float T1;
	static float a,b,c,d;
	static float delta_T;

	if(pvt_struct->start)
	{
		if(!pvt_struct->now_inner_point)
		{
			P0 = pvt_struct->p_start->position;
			V0 = pvt_struct->p_start->velocity;
			T0 = pvt_struct->p_start->time;
			
			pvt_struct->p_next = pvt_struct->p_start + 1;
			
			if(pvt_struct->p_start == pvt_struct->p_end)
			{
				if(pvt_struct->MP[3] == 0)
				{
					pvt_struct->start = 0;
                    pvt_struct->now_inner_point = 0;
					
                    pvt_struct->p_start = &pvt_struct->pvt_table_struct[1];
                    pvt_struct->p_next = &pvt_struct->pvt_table_struct[1];
                    pvt_struct->p_end   = &pvt_struct->pvt_table_struct[1];

					pvt_struct->t=0;
					pvt_struct->a=0;
					pvt_struct->b=0;
					pvt_struct->c=0;
					pvt_struct->d=0;

                    return 0x01;
				}
			}
			
			pvt_struct->t = T0;
            
			P1 = pvt_struct->p_next->position;
			V1 = pvt_struct->p_next->velocity;
			T1 = pvt_struct->p_next->time;
            
            pvt_struct->i32PosTarget = P1;
			
			delta_T=T1-T0;
            if(delta_T<=0)/* actual time is not impossibal */
            {
                delta_T_Error++;
                pvt_struct->start = 0;/* severe fault, must stop pvt motion immediately */
                pvt_struct->now_inner_point = 0;
                pvt_struct->number_of_interpolation = 0;
                return 0xFF;
            }
			
			pvt_struct->number_of_interpolation = delta_T/pvt_struct->MC_period_time;
			pvt_struct->a = (-2*(P1) + V1*delta_T +   V0*delta_T + 2*(P0)) / (delta_T*delta_T*delta_T);
			pvt_struct->b = ( 3*(P1) - V1*delta_T - 2*V0*delta_T - 3*(P0)) / (delta_T*delta_T);
			pvt_struct->c = V0;
			pvt_struct->d = P0;
			
			a = pvt_struct->a;
			b = pvt_struct->b;
			c = pvt_struct->c;
			d = pvt_struct->d;
		}
		
		pvt_struct->t = T0 + pvt_struct->now_inner_point * pvt_struct->MC_period_time;
		
		pvt_struct->s3 = a*(pvt_struct->t-T0)*(pvt_struct->t-T0)*(pvt_struct->t-T0);
		pvt_struct->s2 = b*(pvt_struct->t-T0)*(pvt_struct->t-T0);
		pvt_struct->s1 = c*(pvt_struct->t-T0);
		pvt_struct->s0 = d;
		
		pvt_struct->v2 = 3*a*(pvt_struct->t-T0)*(pvt_struct->t-T0);
		pvt_struct->v1 = 2*b*(pvt_struct->t-T0);
		pvt_struct->v0 = c;
		
		pvt_struct->o_pos_ref = pvt_struct->s3 + pvt_struct->s2 + pvt_struct->s1 + pvt_struct->s0;
		pvt_struct->o_vel_ref = pvt_struct->v2 + pvt_struct->v1 + pvt_struct->v0;
		
        /*output data*/
        pvt_struct->i32PVT_P_Target = (int32_t)(pvt_struct->o_pos_ref);
        pvt_struct->i32PVT_V_Ref = (int32_t)(pvt_struct->o_vel_ref);
        JointCtrl.Motion.i32JointTargetPosition = pvt_struct->i32PVT_P_Target;

		pvt_struct->now_inner_point++;

		if(pvt_struct->now_inner_point == pvt_struct->number_of_interpolation)
		{
            if(!pvt_struct->pauseNext)  
            {
                pvt_struct->p_start++;
                pvt_struct->now_inner_point = 0;
            }
            else    /*走完一个点就停止*/
            {
                pvt_struct->start = 0;
                pvt_struct->now_inner_point = 0;
                
                pvt_struct->p_start = &pvt_struct->pvt_table_struct[1];
                pvt_struct->p_next = &pvt_struct->pvt_table_struct[1];
                pvt_struct->p_end   = &pvt_struct->pvt_table_struct[1];

                pvt_struct->t=0;
                pvt_struct->a=0;
                pvt_struct->b=0;
                pvt_struct->c=0;
                pvt_struct->d=0;

                JointCtrl.PVT_struct.start = FALSE;
                JointCtrl.PVT_Back_struct.start = FALSE;
                JointCtrl.MotionPath.bReappearStart = FALSE;
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = FALSE;
                JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = FALSE;
                return 0x01;
            }
		}
	}
	return 0;
}

uint16_t StartFlag = 0;

_Bool InitPathReturn(void)
{
    int32_t i32StartPoint = 0;
    int32_t i32EndPoint = 0;

    /*起点为当前关节所停止的位置*/
    i32StartPoint = JointCtrl.Motion.i32JointActualPosition;
    i32EndPoint = JointCtrl.PVT_struct.pvt_table_struct[0].position;    /*目标位置是示教路线的起点*/
    
    JointCtrl.PVT_Back_struct.pvt_table_struct[0].position = i32StartPoint;
    JointCtrl.PVT_Back_struct.pvt_table_struct[0].velocity = 0;
    JointCtrl.PVT_Back_struct.pvt_table_struct[0].time = 0.0;
    
    JointCtrl.PVT_Back_struct.pvt_table_struct[1].position = i32EndPoint;
    JointCtrl.PVT_Back_struct.pvt_table_struct[1].velocity = 0;
    JointCtrl.PVT_Back_struct.pvt_table_struct[1].time = ENDTOSTARTTIME;
    
    JointCtrl.PVT_Back_struct.p_start   = &JointCtrl.PVT_Back_struct.pvt_table_struct[0];
    JointCtrl.PVT_Back_struct.p_end     = &JointCtrl.PVT_Back_struct.pvt_table_struct[1];

#if(PRINTF_EN)
    printf("path back:\r\n");
    printf("start:%d\r\n",JointCtrl.PVT_Back_struct.pvt_table_struct[0].position);
    printf("end:%d\r\n",JointCtrl.PVT_Back_struct.pvt_table_struct[1].position);
#endif
    /*PVT 数据初始化*/
    fCycle = (float)JointCtrl.u16JontControlCycle;
    PVT_MotionPathInit(&JointCtrl.PVT_Back_struct, 0.01 ,0,0,1);
    
    return TRUE;
}

/*电机运动路线重现*/
void MotionPathRestore(void)
{
    int32_t i32PosiMax = 0;
    int32_t i32PosiMin = 0;

    /*从示教结束点返回到起始点*/
    if(JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus)
    {
        if(0 != JointCtrl.MotionPath.u32HaltDelay)
        {
            JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointHaltPosi;
            JointCtrl.MotionPath.u32HaltDelay--;
            return;
        }
        if(PVT_Generator(&JointCtrl.PVT_Back_struct))
        {
            /*关节返回到示教路线的起点，等待‘HALTDELAY’时间后，重复示教路线*/
            JointCtrl.MotionPath.u32HaltDelay = HALTDELAY/JointCtrl.u16JontControlCycle;

            /*归零路线结束后，初始化示教路线*/
            PVT_MotionPathInit(&JointCtrl.PVT_struct, 0.01 ,0,0,JointCtrl.MotionPath.u16TotalPoint);
            
            JointCtrl.PVT_struct.start = TRUE;
            JointCtrl.PVT_struct.now_inner_point = 0;
            JointCtrl.Motion.i32JointHaltPosi = JointCtrl.Motion.i32JointActualPosition;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = FALSE;
        }
    }
    else if(JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus)     /*正在重现示教路径*/
    {
        if(0 != JointCtrl.MotionPath.u32HaltDelay)
        {
            JointCtrl.Motion.i32JointTargetPosition = JointCtrl.Motion.i32JointHaltPosi;
            JointCtrl.MotionPath.u32HaltDelay--;
            return;
        }

        if(PVT_Generator(&JointCtrl.PVT_struct))
        {
            JointCtrl.MotionPath.u32HaltDelay = HALTDELAY/JointCtrl.u16JontControlCycle;
            
            /*每个示教动作结束时，需要重新初始化归零路线*/
            JointCtrl.Motion.i32JointHaltPosi = JointCtrl.Motion.i32JointActualPosition;
            JointCtrl.MotionPath.bReturnPathInitFlag = InitPathReturn();
            JointCtrl.PVT_Back_struct.start = TRUE;
            JointCtrl.PVT_Back_struct.now_inner_point = 0;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = TRUE;
        }
    }
#if ('D' == JOINTBOARD_TYPE)
    i32PosiMax = JointCtrl.Motion.i32JointAbsPositionMin;
    i32PosiMin = JointCtrl.Motion.i32JointAbsPositionMax;
#else
    i32PosiMax = JointCtrl.Motion.i32JointAbsPositionMax;
    i32PosiMin = JointCtrl.Motion.i32JointAbsPositionMin;
#endif
    PositionLimit(&JointCtrl.Motion.i32JointTargetPosition, i32PosiMax, i32PosiMin);
}



void MotionPathSwitch(void)
{
    /*判断电机停止的位置是否在限位内*/
    if((JointCtrl.Motion.i32JointActualAngle < JointCtrl.Motion.i32JointAngleMax) &&
        (JointCtrl.Motion.i32JointActualAngle > JointCtrl.Motion.i32JointAngleMin))
    {
        /*如果电机当前停止的位置离示教路线的起点非常近，则跳过回归路线，直接开始恢复示教路线*/
        if((JointCtrl.Motion.i32JointActualPosition < (JointCtrl.PVT_struct.pvt_table_struct[0].position + 200)) && 
            (JointCtrl.Motion.i32JointActualPosition > (JointCtrl.PVT_struct.pvt_table_struct[0].position - 200)))
        {
            PVT_MotionPathInit(&JointCtrl.PVT_struct, 0.01 ,0,0,JointCtrl.MotionPath.u16TotalPoint);
            /*归零环节禁止*/
            JointCtrl.PVT_Back_struct.start = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = FALSE;
            
            /*路径恢复使能*/
            JointCtrl.PVT_struct.start = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = TRUE;
        }
        else
        {
            /*回到初始位置路径初始化标志*/
            if(!JointCtrl.MotionPath.bReturnPathInitFlag)
            {
                JointCtrl.MotionPath.bReturnPathInitFlag = InitPathReturn();
                /*回到初始位置路径初始化标志置位*/
            }
            /*归零环节使能*/
            JointCtrl.PVT_Back_struct.start = TRUE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b5_ReurnStatus = TRUE;
            
            /*路径恢复禁止*/
            JointCtrl.PVT_struct.start = FALSE;
            JointCtrl.MotionPath.u16StatusWords.Status_bit.b3_RestoreStatus = FALSE;
        }
        
        JointCtrl.MotionPath.u32HaltDelay = HALTDELAY/JointCtrl.u16JontControlCycle;
        JointCtrl.Motion.i32JointHaltPosi = JointCtrl.Motion.i32JointActualPosition;
        JointCtrl.MotionPath.bReappearStart = TRUE;
    }
}


//_Bool PositionDataVerification(PVT_Table_Typedef *PVT_Data, uint16_t PointNum, uint16_t )
//{
//    
//}

