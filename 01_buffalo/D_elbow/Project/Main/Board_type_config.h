#ifndef _JOINTBOARD_TYPE_H__
#define _JOINTBOARD_TYPE_H__



// jointboard selection
// type A : 肩关节水平自由度节点,   NODE ID = 3;
// type B : 肩关节垂直自由度节点,   NODE ID = 4;
// type C : 上臂旋转节点,           NODE ID = 5;
// type D : 肘关节节点,             NODE ID = 6;
// type E : 前臂旋转节点,           NODE ID = 7;
// type F : 腕关节旋转节点,         NODE ID = 8;
// type G : 力传感器节点,           NODE ID = 9;


#define JOINTBOARD_TYPE         'D'


#if ('A' == JOINTBOARD_TYPE)      /*肩关节水平节点*/
    #define     SELF_NODE_ID    3
    /*节点特征：
    1. PCB: V10SZPCB003A
    2. Kit多圈编码器，SSI 数据接口；
    3. CAN master RX -- PB5, TX -- PB6
    */
    
#elif ('B' == JOINTBOARD_TYPE)    /*肩关节垂直节点*/
    #define     SELF_NODE_ID    4
    /*节点特征：
    1. PCB: V10SZPCB004A
    2. TDK单圈编码器，数据接口SPI；
    3. 包含刹车功能；
    4. 电机驱动板是EPOS4-50/15；
    5. CAN master RX -- PB12, TX -- PB13
    */
    
#elif ('C' == JOINTBOARD_TYPE)    /*上臂旋转节点*/
    #define     SELF_NODE_ID    5
    /*节点特征：
    1. PCB: V10SZPCB003A
    2. Kit多圈编码器，SSI 数据接口；
    3. CAN master RX -- PB5, TX -- PB6
    4. 无增量编码器，使用SSI的多圈编码器替代增量编码器实现电机控制闭环
    5. MCU有两种方式读取关节角度数据，如果从多圈编码器直接读取，则需要将U2(数据方向)配置为输入，
        如果从电机驱动器中读取则不关心U2的数据方向
    */
    
#elif ('D' == JOINTBOARD_TYPE)    /*肘关节节点*/
    #define     SELF_NODE_ID    6
    /*节点特征：
    1. PCB: V10SZPCB002A
    2. TDK单圈编码器，SPI接口；
    3. CAN master RX -- PB12, TX -- PB13
    */
    
#elif ('E' == JOINTBOARD_TYPE)    /*前臂旋转节点*/
    #define     SELF_NODE_ID    7
    /*节点特征：
    1. V10SZPCB003A
    2. Kit多圈编码器，SSI 数据接口;
    3. CAN master RX -- PB5, TX -- PB6
    */
    
#elif ('F' == JOINTBOARD_TYPE)    /*腕关节节点*/
    #define     SELF_NODE_ID    8
    /*节点特征：
    1. PCB: V10SZPCB002A
    2. TDK单圈编码器，SPI接口；
    3. CAN master RX -- PB12, TX -- PB13
    */
    
#elif ('G' == JOINTBOARD_TYPE)    /*力传感器节点*/
    #define     SELF_NODE_ID    9
#endif


#define TEST_BOARD              0       /*test board 是下肢的旧版本硬件*/

#define PRINTF_EN               0

#define EPOSEMONITOR            0
#define CANOPENMONITOR          0


#endif
