#include "air_cylinder.h"
#include "stm32f4xx.h"
#include "uart4.h"
#include "up_down.h"

//Æø¸×ioÅäÖÃ   B4~B7    E2~E5

void air_cylinder_Configuration(void)
{
		GPIO_InitTypeDef  			GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE,ENABLE);

		GPIO_InitStructure.GPIO_Pin 	= 	GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType 	= 	GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed 	= 	GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd 	= 	GPIO_PuPd_DOWN;
		GPIO_Init(GPIOB,&GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin 	= 	GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType 	= 	GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed 	= 	GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd 	= 	GPIO_PuPd_DOWN;
		GPIO_Init(GPIOE,&GPIO_InitStructure);

		GPIO_ResetBits(GPIOE,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 );
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 );
		
}

void aircylinder_on_off( void )
{
	

}
