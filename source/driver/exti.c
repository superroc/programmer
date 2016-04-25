#include "exti.h"
#include "delay.h"
#include "usart.h"

#define KEY0 PFin(3)
#define KEY1 PFin(4)
#define KEY2 PFin(5)

u8 key1_pressed = 0;
u8 key2_pressed = 0;
u8 key3_pressed = 0;

void EXTI3_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY0==0)
	{
	    //uart_tx("start.\r\n", strlen("start.\r\n"));
        key1_pressed = 1;
	}
	EXTI->PR=1<<3;
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY1==0)
	{
	    //uart_tx("error.\r\n", strlen("error.\r\n"));
        key2_pressed = 1;
	}		 
	EXTI->PR=1<<4;
}

void EXTI9_5_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY2==0)
	{
	    //uart_tx("handler.\r\n", strlen("handler.\r\n"));
        key3_pressed = 1;
	}
	EXTI->PR=1<<5;
}

void EXTIX_Init(void)
{
	RCC->APB2ENR|=1<<7;     //使能PORTF时钟	  
	GPIOF->CRL&=0XFF000FFF;	//PF3~4设置成输入	  
	GPIOF->CRL|=0X00888000; 				   
	GPIOF->ODR|=7<<3;	   	//PF3~4 上拉

	Ex_NVIC_Config(GPIO_F, 3, FTIR); 			//下降沿触发
	Ex_NVIC_Config(GPIO_F, 4, FTIR); 			//下降沿触发
	Ex_NVIC_Config(GPIO_F, 5, FTIR); 			//下降沿触发

	MY_NVIC_Init(2, 3, EXTI3_IRQChannel, 2);	//抢占2，子优先级3，组2
	MY_NVIC_Init(2, 2, EXTI4_IRQChannel, 2);	//抢占2，子优先级2，组2
	MY_NVIC_Init(2, 2, EXTI9_5_IRQChannel, 2);	//抢占2，子优先级2，组2
}

