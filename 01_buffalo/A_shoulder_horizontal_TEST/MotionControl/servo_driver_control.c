
#include "servo_driver_control.h"
#include "stdlib.h"
#include "math.h"
#include "read_write_SDO.h"
#include "CANopenMaster.h"
#include "global.h"
#include "can.h"

#define SERVO_ID      0x1    // servo driver ID
#define BYTES_SIZE_1  1
#define BYTES_SIZE_2  2
#define BYTES_SIZE_4  4
#define TOLERANCE     1000


volatile int8_t mode_to_driver = 0;
int8_t mode_to_driver_previous = 0;

volatile uint16_t controlword_to_driver = 0;
uint16_t controlword_to_driver_previous = 0;

volatile int32_t target_position_to_driver = 0;
int32_t target_position_to_driver_previous = 0;

volatile uint32_t profile_vel_to_driver = 0;
uint32_t profile_vel_to_driver_previous = 0;

//volatile uint32_t following_error_window_to_driver = 2000;
//uint32_t following_error_window_to_driver_previous = 2000;

volatile uint32_t profile_accel_to_driver = 0;
uint32_t profile_accel_to_driver_previous = 0;

volatile uint32_t profile_decel_to_driver = 0;
uint32_t profile_decel_to_driver_previous = 0;

//volatile uint32_t quick_stop_decel_to_driver  = 0;
//uint32_t quick_stop_decel_to_driver_previous  = 0;

//volatile uint32_t max_motor_speed_to_driver = 0;
//uint32_t max_motor_speed_to_driver_previous = 0;

int8_t mode_to_display = 0;
volatile int ppm_before_pvt_flag = 0;
volatile int ppm_before_pvt_flag_previous = 0;

int controlword_temp = 0;
extern uint8_t canSend(CO_Data* d, Message *m);

//
int supervise_par_from_driver(void)
{
    // To Do

    return 0;
}


///////////////////////////////////////////////////////////////////////
volatile int sync_motion_flag = 0;

// use the NMT signal to sync the 4 jointboard to start motion
int syncMotionStart(void)
{
    sync_motion_flag = 1;

	switch (gMC.modes_of_operation)
    {
	    case POS_PTP_MODE:
		
		    break;
		
	    case POS_PVT_MODE:
			
			// 
            if(abs(gMC.PVT_struct.p_start->position - gMC.i_pos_fdbk) > TOLERANCE)
            {
				// called by CAN1_RX0_IRQHandler, which prior to general timer, can protect the next two operations not be interupt
				ppm_before_pvt_flag = 1;
                controlword_temp = ABS_POS;		
            }
//			else
//			{
//			    MC_Start_PVT(&gMC);
//			}
//            statusword |= 1<<12; 			

		    break;
		
	    default:
		    break;
    }

    return 0;
}


volatile int sync_motion_stop_flag = 0;
volatile int sync_motion_stop_flag_previous = 0;
// use the NMT signal to sync the 4 jointboard to stop motion
int syncMotionStop(void)
{
    sync_motion_stop_flag = 1;
	//controlword_stop = 0x10;
	
    return 0;
}


uint16_t controlword_debug_preset = 0;
int servoShutdownToReady(void)
{
	controlword_debug_preset = 0x06;
 //   sdoWriteNetworkDictCallBackAI(&CANopenMaster_Data, SERVO_ID, 0x6040, 0x00, BYTES_SIZE_2, OD_DATATYPE_U16,  &controlword_debug_preset, NULL, 0, 0);	// test code
	Message can_frame;
	can_frame.cob_id = 0x601;
	can_frame.rtr = 0;
	can_frame.len = 8;
    can_frame.data[0] = 0x2b;
    can_frame.data[1] = 0x40;	
    can_frame.data[2] = 0x60;	
    can_frame.data[3] = 0x00;	
    can_frame.data[4] = 0x06;
    can_frame.data[5] = 0x00;		
    can_frame.data[6] = 0x00;			
    can_frame.data[7] = 0x00;		
	canSend(&CANopenMaster_Data, &can_frame);	
	
    return 0;
}


int16_t controlword_debug = 0;
int servoControlwordPresetDebug2(void)
{
    controlword_debug = 0x0F;	// test code 
		
	Message can_frame;
	can_frame.cob_id = 0x601;
	can_frame.rtr = 0;
	can_frame.len = 8;
    can_frame.data[0] = 0x2b;
    can_frame.data[1] = 0x40;	
    can_frame.data[2] = 0x60;	
    can_frame.data[3] = 0x00;	
    can_frame.data[4] = 0x0F;
    can_frame.data[5] = 0x00;		
    can_frame.data[6] = 0x00;			
    can_frame.data[7] = 0x00;		
	canSend(&CANopenMaster_Data, &can_frame);
	
    //sdoWriteNetworkDictCallBackAI(&CANopenMaster_Data, SERVO_ID, 0x6040, 0x00, BYTES_SIZE_2, OD_DATATYPE_U16,  &controlword_debug, NULL, 0, 0);	// test code
	return 0;
}

