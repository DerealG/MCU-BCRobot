#include "delay.h"

static u8  fac_us = 0;							//us延时倍乘数			   
static u16 fac_ms = 0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	u32 reload;
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us = SystemCoreClock / 8000000;				//为系统时钟的1/8  
#if SYSTEM_SUPPORT_OS  							//如果需要支持OS.
	reload = SystemCoreClock / 8000000;				//每秒钟的计数次数 单位为M  
	reload *= 1000000 / delay_ostickspersec;		//根据delay_ostickspersec设定溢出时间
												//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	fac_ms = 1000 / delay_ostickspersec;			//代表OS可以延时的最少单位	   

	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD = reload; 						//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    

#else
	fac_ms = (u16)fac_us * 1000;					//非OS下,代表每个ms需要的systick时钟数   
#endif
}

//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus*fac_us; 					//时间加载	  		 
	SysTick->VAL = 0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开始倒数	  
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)));		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0X00;      					 //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;							//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开始倒数  
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)));		//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0X00;       					//清空计数器	  	    
}

