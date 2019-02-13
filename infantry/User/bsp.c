#include "bsp.h"

void BSP_Init(void)	
{
	CAN1_Configuration();            //初始化CAN1
	CAN2_Configuration(); 
//	USART3_Configuration();          //串口3初始化    串口调试	 
	UART4_Configuration();
	USART2_Configuration();
	KEY_Configuration();
//	TIM4_Configuration();
	air_cylinder_Configuration();
}


