#include "main.h"

void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  					//使能TIM2时钟
	
	TIM_TimeBaseInitStructure.TIM_Period 		= 	1000 - 1; 				//自动重装载值	1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler		= 	84 - 1;  				//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode	=	TIM_CounterMode_Up; 	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);						//初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 								//允许定时器4更新中断
	TIM_Cmd(TIM2,ENABLE); 													//使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM2_IRQn; 	//定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1; 		//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1; 		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int timer2_flag = 0;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		timer2_flag++;
//		printf ("已进入定时器中断\r\n");
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	}
}
