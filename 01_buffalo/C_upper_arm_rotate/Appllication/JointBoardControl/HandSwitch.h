#ifndef HANDSWITCH_H_INCLUDED
#define HANDSWITCH_H_INCLUDED


#define RIGHTHAND                   1
#define LEFTHAND                    0

#define JOINT_CTRL_CYCLE            10          /*关节控制函数周期，单位mS*/
#define HANDTYPE                    RIGHTHAND   //LEFTHAND  //



typedef enum{
    RightHand   = 0xAC,     /*右手模式*/
    LeftHand    = 0x32      /*左手模式*/
}HandType_en;





#endif // HANDSWITCH_H_INCLUDED
