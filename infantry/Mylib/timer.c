#include "main.h"

void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef 			NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  					//ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period 		= 	1000 - 1; 				//�Զ���װ��ֵ	1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler		= 	84 - 1;  				//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode	=	TIM_CounterMode_Up; 	//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);						//��ʼ��TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 								//����ʱ��4�����ж�
	TIM_Cmd(TIM2,ENABLE); 													//ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM2_IRQn; 	//��ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1; 		//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	1; 		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int timer2_flag = 0;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
	{
		timer2_flag++;
//		printf ("�ѽ��붨ʱ���ж�\r\n");
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	}
}
