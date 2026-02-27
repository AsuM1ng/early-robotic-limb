#include "chipHAL_GPIO.h"



/********************************************************************
 * @brief	Initialize the GPIO of CANID Switch.
 *			switch0 -- GPIO -- PB2
 *          switch1 -- GPIO -- PD2
 *			switch2 -- GPIO -- PC11
 *          switch3 -- GPIO -- PA15
 *          each switch connect to GND when switched on, 
 *          which means GPIO_PuPd should be set to GPIO_PuPd_UP
 * @param      void
 *
 * @return     none
 */
void GPIO_CanIDInit(void)
{    	 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(CANID0_GPIO_CLK | CANID1_GPIO_CLK
                                            	| CANID2_GPIO_CLK | CANID3_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    GPIO_InitStructure.GPIO_Pin = CANID0_GPIO_PIN;
	GPIO_Init(CANID0_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CANID1_GPIO_PIN;
	GPIO_Init(CANID1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CANID2_GPIO_PIN;
	GPIO_Init(CANID2_GPIO_PORT, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = CANID3_GPIO_PIN;
	GPIO_Init(CANID3_GPIO_PORT, &GPIO_InitStructure);
}


/********************************************************************
 * @brief	Initialize the GPIO of LED.
 *			LED green   -- GPIO -- PC6
 *          LED blue    -- GPIO -- PC7
 *          LED red     -- GPIO -- PC8
 *          LED white   -- GPIO -- PC9
 *          each LED connect to GND, so the MCU need to provide the source to light the LED,
 *          which means GPIO_PuPd should be set to GPIO_PuPd_UP
 *
 * @param      void
 *
 * @return     none
 */
void GPIO_LedInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_LED_GPIO_CLOCK, ENABLE);    //enable GPIOC clock
    #if (TEST_BOARD)
        GPIO_InitStructure.GPIO_Pin = LED_G | LED_B | LED_R | LED_W;
    #else
        GPIO_InitStructure.GPIO_Pin = LED_R | LED_G | LED_B;
    #endif

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
    
    #if (TEST_BOARD)
    GPIO_ResetBits(LED_GPIO_PORT, LED_R | LED_G | LED_B | LED_W);   //set high, turn on LED
    #else
    GPIO_ResetBits(LED_GPIO_PORT, LED_R | LED_G | LED_B);           //set high, turn on LED
    #endif
}


/********************************************************************
 * @brief	Initialize the GPIO of USART.
 *			USART6_TX -- PC6;
 *			USART6_RX -- PC7;
 *
 * @param      void
 *
 * @return     none
 */
void GPIO_UsartInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);     //enable GPIOC clock
	
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART1);  //GPIOC6 use AF as USART1
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART1); //GPIOC7 use AF as USART1

	//USART1 port config
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(GPIOC,&GPIO_InitStructure);
}

#if (!PRINTF_EN)
void TestGpio_USART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(TEST_GPIO_CLOCK, ENABLE);    //enable GPIOB clock

    GPIO_InitStructure.GPIO_Pin = TEST_GPIO_PINTX | TEST_GPIO_PINRX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             
    GPIO_Init(TEST_GPIO_PORT, &GPIO_InitStructure);

    
    GPIO_ResetBits(TEST_GPIO_PORT, TEST_GPIO_PINTX | TEST_GPIO_PINRX);   //set high, turn on LED
}
#endif

