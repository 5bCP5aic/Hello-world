#include "bsp.h"

void BSP_Init(void)	
{
	CAN1_Configuration();            //��ʼ��CAN1
	CAN2_Configuration(); 
//	USART3_Configuration();          //����3��ʼ��    ���ڵ���	 
	UART4_Configuration();
	USART2_Configuration();
	KEY_Configuration();
//	TIM4_Configuration();
	air_cylinder_Configuration();
}


