//#include "main.h"

//void KEY_Configuration(void)
//{
//    GPIO_InitTypeDef  gpio;
//    EXTI_InitTypeDef  exti;
//    NVIC_InitTypeDef  nvic;
//    
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
//   
//    gpio.GPIO_Pin = GPIO_Pin_8;   
//    gpio.GPIO_Mode = GPIO_Mode_IN;
//    gpio.GPIO_OType = GPIO_OType_PP;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;
//    gpio.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(GPIOC, &gpio);
//    
//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource4);
//    
//    exti.EXTI_Line = EXTI_Line4;
//    exti.EXTI_Mode = EXTI_Mode_Interrupt;
//    exti.EXTI_Trigger = EXTI_Trigger_Falling;
//    exti.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&exti);
//    
//    nvic.NVIC_IRQChannel = EXTI4_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 2;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//}

//void EXTI4_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line4))
//    {
//        EXTI_ClearITPendingBit(EXTI_Line4);
//    }
//}


#include "main.h"
#include "key.h"

void KEY_Configuration(void)
{
   GPIO_InitTypeDef         GPIO_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//´¥Åö
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//¹âµç
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE,&GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
}


