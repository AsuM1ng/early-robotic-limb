#ifndef _EQUIPMENT_PARAM_H__
#define _EQUIPMENT_PARAM_H__

// robot joint angle limit
// encoder resolution: 4096
// gear_ratio: 120
#define LEFT_HIP_POS_LIMIT   40960               // 30бу
#define LEFT_HIP_NEG_LIMIT   -163840             // -120бу
#define LEFT_KNEE_POS_LIMIT  0                    // 0бу
#define LEFT_KNEE_NEG_LIMIT  -163840             // 120бу
#define RIGHT_HIP_POS_LIMIT  163840              // 120бу
#define RIGHT_HIP_NEG_LIMIT  -40960              // -30бу
#define RIGHT_KNEE_POS_LIMIT 163840              // 120бу
#define RIGHT_KNEE_NEG_LIMIT 0                    // 0бу


//unit: kg
typedef struct 
{
    float hip_joint_weight;    
	float knee_joint_weight;
	float shoe_weight;
	float thigh_weight;
	float calf_weight;
}RobotLegWeight;

//unit: m
typedef struct
{
    float thigh_length;
	float calf_length;
}RobotLegLength;


typedef struct
{
    RobotLegWeight weight;
	RobotLegLength length;
}RobotLeg;

typedef struct
{
    RobotLeg left_leg;
    RobotLeg right_leg;	
}DoubleRobotLeg;


typedef enum 
{
    LEFT_HIP = 1,
	LEFT_KNEE,
	RIGHT_HIP,
	RIGHT_KNEE
}JointID;


#endif

