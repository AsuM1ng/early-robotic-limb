#ifndef _MOTOR_PARAM_EC60_H__
#define _MOTOR_PARAM_EC60_H__

#define MOTOR_BRAND   "MAXON"
#define MOTOR_TYPE    "EC60"


#define MOTOR_NO_LOAD_CURRENT  419   // unit: mA
#define MOTOR_NORMINAL_TORQUE  289   // unit: mNm
#define MOTOR_NORMINAL_CURRENT 5470  // unit: mA
#define MOTOR_EFFICIENCY       0.86  
#define TORQUE_CONSTANT        53.4  // unit: mNm/A

#define MOTOR_CURRENT_NOMIANL_LIMIT  15000    // unit: mA
#define MOTOR_CURRENT_LIMIT_USE      7500     // unit: mA

#define ENCODER_LINE           1024
#define ENCODER_INTERPOLATION  4
#define ENCODER_RESOLUTION     4096  // ENCODER_LINE * ENCODER_INTERPOLATION



#endif

