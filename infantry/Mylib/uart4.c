#include "stm32f4xx.h"
#include "uart4.h"
#include "usart3.h"
#include "can2.h"

//for D-BUS

RC_Ctl_t RC_Ctl;

volatile unsigned char sbus_rx_buffer[ DBUSLength + DBUSBackLength];

void Decode(void);
/**
  * @brief  UART4相关配置
  * @param  void
  * @retval void
  * @note	for Dbus		UART4_RX-----PC11
  */

void UART4_Configuration(void)
{
    USART_InitTypeDef 				USART_InitStructure;
		GPIO_InitTypeDef  				GPIO_InitStructure;
    NVIC_InitTypeDef  				NVIC_InitStructure;
		DMA_InitTypeDef   				DMA_InitStructure;
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
    
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin 	= 	GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType 	= 	GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed 	= 	GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd 	= 	GPIO_PuPd_UP;//之前为GPIO_PuPd_NOPULL
    GPIO_Init(GPIOC,&GPIO_InitStructure);

		USART_InitStructure.USART_BaudRate 				= 	100000;
		USART_InitStructure.USART_WordLength 			= 	USART_WordLength_8b;
		USART_InitStructure.USART_StopBits 				= 	USART_StopBits_1;
		USART_InitStructure.USART_Parity 				= 	USART_Parity_No;
		USART_InitStructure.USART_Mode 					= 	USART_Mode_Rx ;
		USART_InitStructure.USART_HardwareFlowControl 	= 	USART_HardwareFlowControl_None;
		USART_Init(UART4,&USART_InitStructure);
		    
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
//		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);                //测试时可以关掉
		USART_Cmd(UART4,ENABLE);
    USART_ClearFlag(UART4, USART_FLAG_CTS);
		
    NVIC_InitStructure.NVIC_IRQChannel 						= 	UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 	3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 	2;
    NVIC_InitStructure.NVIC_IRQChannelCmd					= 	ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(UART4,USART_IT_IDLE ,ENABLE);
		

		/* 配置 DMA Stream */    
    //UART4_RX
		DMA_DeInit(DMA1_Stream2);
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&UART4->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(sbus_rx_buffer);
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   DBUSLength + DBUSBackLength;
    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;    
    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream2, ENABLE);
}


char UART4temp;
void UART4_IRQHandler(void)
{	
	UART4temp = UART4->DR;
  UART4temp = UART4->SR;

	DMA_Cmd(DMA1_Stream2, DISABLE);				//关闭DMA传输 
	
	if(DMA1_Stream2->NDTR == DBUSBackLength)
	{
		Decode();                          //解码
	}
		
		CAN2_Send_D_BUS_Msg( RC_Ctl.key.v, RC_Ctl.mouse.y, RC_Ctl.mouse.press_l, RC_Ctl.mouse.press_r );
	
 //重启DMA
	DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
	while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
	DMA_SetCurrDataCounter(DMA1_Stream2, DBUSLength + DBUSBackLength);
	DMA_Cmd(DMA1_Stream2, ENABLE);
}

void Decode(void)
{
	  RC_Ctl.rc.ch0  = (sbus_rx_buffer[0]			| (sbus_rx_buffer[1] << 8)) & 0x07ff;
		RC_Ctl.rc.ch1  = ((sbus_rx_buffer[1] >> 3) 	| (sbus_rx_buffer[2] << 5)) & 0x07ff;
		RC_Ctl.rc.ch2  = ((sbus_rx_buffer[2] >> 6) 	| (sbus_rx_buffer[3] << 2) |
						  (sbus_rx_buffer[4] << 10)) & 0x07ff;
		RC_Ctl.rc.ch3  = ((sbus_rx_buffer[4] >> 1) 	| (sbus_rx_buffer[5] << 7)) & 0x07ff;
		RC_Ctl.rc.s1   = ((sbus_rx_buffer[5] >> 4)	& 0x000C) >> 2;
		RC_Ctl.rc.s2   = ((sbus_rx_buffer[5] >> 4)	& 0x0003);
		
		RC_Ctl.mouse.x =   sbus_rx_buffer[6] |(sbus_rx_buffer[7] << 8);
		RC_Ctl.mouse.y =   sbus_rx_buffer[8] |(sbus_rx_buffer[9] << 8);
		RC_Ctl.mouse.z =   sbus_rx_buffer[10] |(sbus_rx_buffer[11] << 8);
		RC_Ctl.mouse.press_l = sbus_rx_buffer[12];
		RC_Ctl.mouse.press_r = sbus_rx_buffer[13];
		
		RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);		
}




void telecontrol_Init( void )
{
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.mouse.x = 0;
	RC_Ctl.mouse.y = 0;
	RC_Ctl.key.v = 0;
}

