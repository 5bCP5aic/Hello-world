#include "stm32f4xx.h"
#include "usart2.h"
#include "usart3.h"
#include "motor.h"

/**
  * @brief  USART2Ïà¹ØÅäÖÃ
  * @param  void
  * @retval void
  * @note		USART2_TX-----PD5   USART2_RX-----PD6
  */

void USART2_Configuration(void)
{
  USART_InitTypeDef usart2;
	GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD ,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5 ,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);
	
	gpio.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD,&gpio);
    
  USART_DeInit(USART2);
	usart2.USART_BaudRate = 115200;   
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_No;
	usart2.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
  usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	
	USART_Cmd(USART2,ENABLE);
  
	 nvic.NVIC_IRQChannel 						= 	USART2_IRQn;
   nvic.NVIC_IRQChannelPreemptionPriority 	= 	1;
   nvic.NVIC_IRQChannelSubPriority 			= 	2;
   nvic.NVIC_IRQChannelCmd 					= 	ENABLE;
   NVIC_Init(&nvic);
}
void USART2_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);
		USART_SendData(USART2,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);
    USART_SendData(USART2, (uint8_t)ch);
    return ch;
}
char Rx_data_2;
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
        Rx_data_2  = USART2->DR;
		
		if ( Rx_data_2 == '0' )
		{
			
			pid_trailer.Sv[0] = 0;
			
		}
		if ( Rx_data_2 == '1' )
		{
			
			pid_trailer.Sv[0] = 78000;
		
		}
		if ( Rx_data_2 == '2' )
		{
			
			pid_trailer.Sv[0] += 2000;
			
		}
		if ( Rx_data_2 == '3' )
		{
			
			pid_trailer.Sv[0] -= 2000;
			
		}
		if ( Rx_data_2 == '4' )
		{
			
			pid_trailer.Kp[0] += 0.01f;//
		
		}
		if ( Rx_data_2 == '5' )
		{
			
			pid_trailer.Kp[0] -= 0.01f;//
			
			
		}
		if ( Rx_data_2 == '6' )
		{
			
			pid_trailer.Ki[1] += 0.01f;//
			
		}
		if ( Rx_data_2 == '7' )
		{
			
			pid_trailer.Ki[1] -= 0.01f;//
			
		}
		if ( Rx_data_2 == '8' )
		{
			
			pid_trailer.location[0] = 0;//
			
		}
		
    }
}



