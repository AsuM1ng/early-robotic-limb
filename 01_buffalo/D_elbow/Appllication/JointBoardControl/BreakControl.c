#include "BreakControl.h"
#include "JointControl.h"

#if ('B' == JOINTBOARD_TYPE)

/********************************************************************
 * @brief	Initialize the GPIO of Break.
 *			Break control  -- PA1;
 *			Break feedback -- PA2;
 *
 * @param      void
 *
 * @return     none
 */

void GPIO_BreakInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(BREAK_CONTROL_GPIO_CLOCK | BREAK_FEEDBACK_GPIO_CLOCK, ENABLE);    //enable GPIOC clock
    
    GPIO_InitStructure.GPIO_Pin = BREAK_CONTROL_GPIO_PIN;       /*刹车控制引脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BREAK_CONTROL_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = BREAK_FEEDBACK_GPIO_PIN;      /*刹车状态反馈引脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(BREAK_FEEDBACK_GPIO_PORT, &GPIO_InitStructure);
    
    /*初始化时，刹车抱死*/
    GPIO_SetBits(BREAK_CONTROL_GPIO_PORT,BREAK_CONTROL_GPIO_PIN);
}

/********************************************************************
 * @brief	刹车释放函数
 * @param   void
 *
 * @return  void
 */
void BreakFreeCtrl(void)
{
    /*GPIO拉低，刹车线圈通电，释放刹车*/
    GPIO_ResetBits(BREAK_CONTROL_GPIO_PORT,BREAK_CONTROL_GPIO_PIN);
}

/********************************************************************
 * @brief	刹车抱死函数
 * @param   void
 *
 * @return  void
 */
void BreakEnableCtrl(void)
{
    /*GPIO拉高，刹车线圈不通电，刹车抱死*/
    GPIO_SetBits(BREAK_CONTROL_GPIO_PORT,BREAK_CONTROL_GPIO_PIN);
}

/********************************************************************
 * @brief	获取刹车状态
 * @param   void
 *
 * @return BreakStatus 刹车状态， 正在刹车/刹车释放
 */
BreakStatus GetBreakStatus(void)
{
    if(GPIO_ReadInputDataBit(BREAK_FEEDBACK_GPIO_PORT,BREAK_FEEDBACK_GPIO_PIN))
        return Breaking;
    else
        return Releaseing;
}


void BreakControlProcess(BreakControl_en breakCtrl)
{
    if(Release == breakCtrl)
    {
        BreakFreeCtrl();    /*释放刹车*/
    }
    else
    {
        BreakEnableCtrl();  /*刹车抱死*/
    }
}

#endif


