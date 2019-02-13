#include "stm32f4xx.h"
#include "usart3.h"
#include "can2.h"

/**
  * @brief  CAN2相关配置
  * @param  void
  * @retval void
  * @note	CAN2_TX-----PB12		CAN1_RX-----PB13
  */

void CAN2_Configuration(void)
{
    CAN_InitTypeDef        			CAN_InitStructure;
    CAN_FilterInitTypeDef  			CAN_FilterStructure;
    GPIO_InitTypeDef       			GPIO_InitStructure;
    NVIC_InitTypeDef       			NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel 						= 	CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 	0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 	2;
    NVIC_InitStructure.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM 		= 	DISABLE;
    CAN_InitStructure.CAN_ABOM 		= 	DISABLE;    
    CAN_InitStructure.CAN_AWUM 		= 	DISABLE;    
    CAN_InitStructure.CAN_NART 		= 	DISABLE;    
    CAN_InitStructure.CAN_RFLM 		= 	DISABLE;    
    CAN_InitStructure.CAN_TXFP 		= 	ENABLE;     
    CAN_InitStructure.CAN_Mode 		= 	CAN_Mode_Normal; 
    CAN_InitStructure.CAN_SJW  		= 	CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 		= 	CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2 		= 	CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = 	3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &CAN_InitStructure);
    
    CAN_FilterStructure.CAN_FilterNumber		=	14;
    CAN_FilterStructure.CAN_FilterMode			=	CAN_FilterMode_IdMask;
    CAN_FilterStructure.CAN_FilterScale			=	CAN_FilterScale_32bit;
    CAN_FilterStructure.CAN_FilterIdHigh		=	0x0000;
    CAN_FilterStructure.CAN_FilterIdLow			=	0x0000;
    CAN_FilterStructure.CAN_FilterMaskIdHigh	=	0x0000;
    CAN_FilterStructure.CAN_FilterMaskIdLow		=	0x0000;
    CAN_FilterStructure.CAN_FilterFIFOAssignment=	0;//the message which pass the filter save in fifo0
    CAN_FilterStructure.CAN_FilterActivation	=	ENABLE;
    CAN_FilterInit(&CAN_FilterStructure);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
}

int8_t photoswitch1 = 0;
int8_t photoswitch2 = 0;
int8_t take = 0;


void CAN2_Send_D_BUS_Msg( int16_t Vel1, int16_t Vel2, int8_t Vel3, int8_t Vel4 )
{
	CanTxMsg tx_message;			//发送缓冲区
	
	tx_message.StdId = 0x11;		//ID;0x11
	tx_message.ExtId = 0x00;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	tx_message.Data[0] = (unsigned char)(Vel1 >> 8);
	tx_message.Data[1] = (unsigned char) Vel1;
	
	tx_message.Data[2] = (unsigned char)(Vel2 >> 8);
	tx_message.Data[3] = (unsigned char) Vel2;
	
	tx_message.Data[4] = (unsigned char) Vel3;
	
	tx_message.Data[5] = (unsigned char) Vel4;
	
	CAN_Transmit(CAN2,&tx_message);
}

void CAN2_Send_Msg( int8_t Cylinder1, int8_t Cylinder2, int8_t dbus_key ) //
{
	CanTxMsg tx_message;			//发送缓冲区
	tx_message.StdId = 0x12;		//ID:0x12
	tx_message.ExtId = 0x00;
	tx_message.IDE = CAN_ID_STD;     //标准帧
	tx_message.RTR = CAN_RTR_DATA;   //数据帧
    tx_message.DLC = 0x08;           //帧长度为2
    
	tx_message.Data[0] = (uint8_t) Cylinder1;
	tx_message.Data[1] = (uint8_t) Cylinder2;
	tx_message.Data[2] = (uint8_t) dbus_key;
	
	CAN_Transmit(CAN2,&tx_message);
}

void CAN2_RX0_IRQHandler( void )
{
    CanRxMsg rx_message;
	 
	
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
//							 printf("321\r\n");

       switch(rx_message.StdId)
			 {

				  case 0x11 : 
						photoswitch1 = (int8_t)rx_message.Data[0];
						photoswitch2 = (int8_t)rx_message.Data[1];
					break;
				  case 0x12 : 
						take = (int8_t)rx_message.Data[0];
					break;
				  
				  default:
					break;
				  
				 
			 }
     CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	
		}
}

//void chassis_photoswitch( void )
//{
//	if ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 0 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0 )
//	{
//		CAN2_Send_Msg( 0, 1 );
//	}
//	if ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0 )
//	{
//		CAN2_Send_Msg( 0, 2 );
//	}
//	if ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 0 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 )
//	{
//		CAN2_Send_Msg( 0, 3 );
//	}
//	if ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 )
//	{
//		CAN2_Send_Msg( 0, 4 );
//	}
//}
