
/* 本文件定义了stm32F4的位带区(bit-band)到位带别名区(bit-band alias region)的映射关系，方便用位带别名区去操作位带区的位！ 	*/
/* 用途：用于操作某一个IO口的某一个引脚。eg:读PA10 --> tmp = PAI(10); 写PB9 --> PBO(9) = 1, PBO(9) = 0. 					*/

#ifndef __BITBAND_H
#define __BITBAND_H

/* 位带区-->位带别名区地址映射 */
#define BITBAND_ALIAS(addr, bitnum)	(((addr)&0xf0000000) + 0x02000000 + (((addr)&0x000fffff)<<5) + ((bitnum)<<2))	//位带区地址addr --> 位带别名区地址BITBAND_ALIAS
#define MEM_ADDR(addr)				( *((volatile uint32_t*)(addr)) )				//数字强制转换为 uint32_t* 的地址并取地址上的内容
#define BIT_ADDR(addr, bitnum)		( MEM_ADDR(BITBAND_ALIAS(addr, bitnum)) )		//将地址转换和取地址上的内容合 用BIT_ADDR代替

/* stm32f4 IO口输入输出寄存器地址映射 */
#define GPIOA_IDR_ADDR    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_ADDR    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_ADDR    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_ADDR    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_ADDR    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_ADDR    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_ADDR    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_ADDR    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_ADDR    (GPIOI_BASE+16) //0x40022010 

#define GPIOA_ODR_ADDR    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_ADDR    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_ADDR    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_ADDR    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_ADDR    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_ADDR    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_ADDR    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_ADDR    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_ADDR    (GPIOI_BASE+20) //0x40022014     

/* IO读取写入操作宏 */
#define PAO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PAI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PBO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PBI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PCO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PCI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PDO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PDI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PEO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PEI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PFO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PFI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PGO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PGI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PHO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PHI(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取

#define PIO(n)		BIT_ADDR(GPIOA_ODR_ADDR, n)		//写入
#define PII(n)		BIT_ADDR(GPIOA_IDR_ADDR, n)		//读取


#endif


