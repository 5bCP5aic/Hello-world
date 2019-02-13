#include "stm32f4xx.h"

//TIM3 TIM4 connect to encoder,a-b
//TIM3_CH3 ----- PC8
//TIM3_CH4 ----- PC9

//TIM4_CH1 ----- PD12
//TIM4_CH2 ----- PD13

void Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4 ,ENABLE);
    
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC,&gpio);
	
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	  gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD,&gpio);
	
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12,  GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13,  GPIO_AF_TIM4);
    
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}
