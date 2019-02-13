#include "stm32f4xx.h"
#include "usart3.h"
#include "tim4.h"
#include "can1.h"
#include "motor.h"
#include "up_down.h"
#include "chassis.h"
#include "trailer.h"

char Buff[7] = {'9','8','0','8','0','6',' '};
void TIM4_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  					//使能TIM4时钟
	
	TIM_TimeBaseInitStructure.TIM_Period 		= 	2000 - 1; 				//自动重装载值	1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler		= 	84 - 1;  				//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode	=	TIM_CounterMode_Up; 	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);						//初始化TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 								//允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); 													//使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM4_IRQn; 	//定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1; 		//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1; 		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//void NRF2401_Send(char *packet)
//{
//	char i;
//	for(i=0;i<7;i++)
//	{
//		USART3_SendChar(packet[i]);
////		USART_SendData(USART3,packet[i]);
//	}
//}


int ReadValue1,ReadValue2,ReadValue3,ReadValue4,ReadValue5,ReadValue6,ReadValue7,ReadValue8;
int time = 0; 


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
	{

		motor_perform ();
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	}
}

