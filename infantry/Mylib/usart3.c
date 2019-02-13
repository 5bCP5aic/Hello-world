#include "stm32f4xx.h"
#include <stdio.h>
#include "usart3.h"
#include "chassis.h"
#include "up_down.h"

/**
  * @brief  USART3相关配置
  * @param  void
  * @retval void
  * @note	USART3_TX-----PB10		USART3_RX-----PB11
  */

//char vision_buff[1] = {'s'};
void USART3_Configuration(void)
{
  USART_InitTypeDef 				USART_InitStructure;
  GPIO_InitTypeDef  				GPIO_InitStructure;
  NVIC_InitTypeDef  				NVIC_InitStructure;
//  DMA_InitTypeDef   				DMA_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10 ,GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

  GPIO_InitStructure.GPIO_Pin 	= 	GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType 	= 	GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed 	= 	GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd 	= 	GPIO_PuPd_UP;//之前为GPIO_PuPd_NOPULL
  GPIO_Init(GPIOB,&GPIO_InitStructure);

	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate 				= 	115200;
	USART_InitStructure.USART_WordLength 			= 	USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 				= 	USART_StopBits_1;
	USART_InitStructure.USART_Parity 				= 	USART_Parity_No;
	USART_InitStructure.USART_Mode 					= 	USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl 	= 	USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure);

//	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

	USART_Cmd(USART3,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel 						= 	USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   	= 	0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 	2;
    NVIC_InitStructure.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);	

//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//    //USART3_RX
//    DMA_DeInit(DMA1_Stream6);
//    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
//    DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART3->DR);
//    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(vision_buff);
//    DMA_InitStructure.DMA_DIR               =   DMA_DIR_MemoryToPeripheral;
//    DMA_InitStructure.DMA_BufferSize        =   1;
//    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
//    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
//    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
//    DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
//    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Circular;
//    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
//    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
//    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
//    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
//    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
//    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
//    DMA_Cmd(DMA1_Stream3, ENABLE);
}

//void DMA1_Stream3_IRQHandler(void)
//{
//	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
//  {
//		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
//		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
//	}
//}


void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
		USART_SendData(USART3,b);
}

//int fputc(int ch, FILE *f)
//{
//    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
//    USART_SendData(USART3, (uint8_t)ch);
//    return ch;
//}
char Rx_data_3;
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
        Rx_data_3  = USART3->DR;
		
		if ( Rx_data_3 == '0' )
		{
			
			pid_up_down.Sv[0] = 0;
			
		}
		if ( Rx_data_3 == '1' )
		{
			
			pid_up_down.Sv[0] = 790000;
		
		}
		if ( Rx_data_3 == '2' )
		{
			
			pid_up_down.Sv[0] += 2000;
			
		}
		if ( Rx_data_3 == '3' )
		{
			
			pid_up_down.Sv[0] -= 2000;
			
		}
		if ( Rx_data_3 == '4' )
		{
			
			pid_up_down.Kp[0] += 0.01f;//
		
		}
		if ( Rx_data_3 == '5' )
		{
			
			pid_up_down.Kp[0] -= 0.01f;//
			
			
		}
		if ( Rx_data_3 == '6' )
		{
			
			pid_up_down.Ki[1] += 0.01f;//
			
		}
		if ( Rx_data_3 == '7' )
		{
			
			pid_up_down.Ki[1] -= 0.01f;//
			
		}
		
		
    }
}

