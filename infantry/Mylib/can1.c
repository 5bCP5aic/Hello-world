#include "can1.h"
#include "usart3.h"

/**
  * @brief  CAN1相关配置
  * @param  void
  * @retval void
  * @note	CAN1_TX-----PA12		CAN1_RX-----PA11
  */

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        			CAN_InitStructure;
    CAN_FilterInitTypeDef  			CAN_FilterInitStructure;
    GPIO_InitTypeDef       			GPIO_InitStructure;
    NVIC_InitTypeDef       			NVIC_InitStructure;
	
	//使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	//引脚复用映射配置
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);//PA12复用位CAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);//PA11复用位CAN1

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
    
    NVIC_InitStructure.NVIC_IRQChannel 						= 	CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 	0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 	1;
    NVIC_InitStructure.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    
    CAN_InitStructure.CAN_TTCM 		= 	DISABLE;	//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM 		= 	DISABLE;	//软件自动离线管理
    CAN_InitStructure.CAN_AWUM 		= 	DISABLE;	//睡眠模式通过软件唤醒
    CAN_InitStructure.CAN_NART 		= 	DISABLE;	//禁止报文自动传送
    CAN_InitStructure.CAN_RFLM 		= 	DISABLE;	//报文不锁定，新的覆盖旧的
    CAN_InitStructure.CAN_TXFP 		= 	ENABLE;	//发送FIFO优先级由报文的标识符来决定
    CAN_InitStructure.CAN_Mode 		= 	CAN_Mode_Normal;	//CAN硬件工作在正常模式
    CAN_InitStructure.CAN_SJW  		= 	CAN_SJW_1tq;	//重新同步跳跃宽度为1个时间单位
    CAN_InitStructure.CAN_BS1 		= 	CAN_BS1_9tq;	//时间段1占用9个时间单位
    CAN_InitStructure.CAN_BS2 		= 	CAN_BS2_4tq;	//时间段2占用4个时间单位
    CAN_InitStructure.CAN_Prescaler = 	3;   // CAN BaudRate 42/(1+9+4)/3=1Mbps	分频系数（Fdiv）
    CAN_Init(CAN1, &CAN_InitStructure);	//初始化CAN1

	CAN_FilterInitStructure.CAN_FilterNumber		=	0;	//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode			=	CAN_FilterMode_IdMask;	//指定了过滤器将被初始化到的模式标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale			=	CAN_FilterScale_32bit;	//给出了过滤器位宽1个32位过滤器
	CAN_FilterInitStructure.CAN_FilterIdHigh		=	0x0000;	//用来设定过滤器标识符（32位位宽时为其高段位，16位位宽时为其第一个）
	CAN_FilterInitStructure.CAN_FilterIdLow			=	0x0000;	//用来设定过滤器标识符（32位位宽时为其低段位，16位位宽时为其第二个）
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh	=	0x0000;	//用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其高段位，16位位宽时为第一个）
	CAN_FilterInitStructure.CAN_FilterMaskIdLow		=	0x0000;	////用来设定过滤器屏蔽标识符或者过滤器标识符（32位位宽时为其低段位，16位位宽时为第二个）
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=	0;	//设定了指向过滤器的FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation	=	ENABLE;	//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	//使能指定的CAN中断
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}


int16_t motor_speed[4] = {0.0f};
int16_t motor_encoder[4] = {0.0f};		/* 底盘电机编码器值 */
int16_t upper_speed = {0.0f};
int16_t upper_encoder = {0.0f};
int16_t trailer_speed = {0.0f};
int16_t trailer_encoder = {0.0f};

/*
 * 函数：CAN1_Send_Msg
 * 功能：can1发送
 * 参数：exdid 地址, msg 数据, len 数据长度
*/

void CAN1_Send_flex_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4)

{
	CanTxMsg tx_message;			//发送缓冲区
	
	tx_message.StdId = 0x1FF;		//ID;0x200
	tx_message.ExtId = 0x00;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	
	tx_message.Data[0] = (unsigned char)(Vel1 >> 8);
	tx_message.Data[1] = (unsigned char) Vel1;
	
	tx_message.Data[2] = (unsigned char)(Vel2 >> 8);
	tx_message.Data[3] = (unsigned char) Vel2;
	
	tx_message.Data[4] = (unsigned char)(Vel3 >> 8);
	tx_message.Data[5] = (unsigned char) Vel3;
	
	tx_message.Data[6] = (unsigned char)(Vel4 >> 8);
	tx_message.Data[7] = (unsigned char) Vel4;

	CAN_Transmit(CAN1,&tx_message);
}

void CAN1_Send_Chassis_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4)		//底盘发送

{
	CanTxMsg tx_message;			//发送缓冲区
	
	tx_message.StdId = 0x200;		//ID;0x200
	tx_message.ExtId = 0x00;
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	tx_message.Data[0] = (unsigned char)(Vel1 >> 8);
	tx_message.Data[1] = (unsigned char) Vel1;
	
	tx_message.Data[2] = (unsigned char)(Vel2 >> 8);
	tx_message.Data[3] = (unsigned char) Vel2;
	
	tx_message.Data[4] = (unsigned char)(Vel3 >> 8);
	tx_message.Data[5] = (unsigned char) Vel3;
	
	tx_message.Data[6] = (unsigned char)(Vel4 >> 8);
	tx_message.Data[7] = (unsigned char) Vel4;

	CAN_Transmit(CAN1,&tx_message);
}



void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		
    }
}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		switch(rx_message.StdId)
		{
			case 0x201 : 
				motor_speed[0] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//底盘左前
				motor_encoder[0] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x202 : 
				motor_speed[1] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//底盘右前
				motor_encoder[1] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x203 : 
				motor_speed[2] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//底盘左后
				motor_encoder[2] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x204 : 
				motor_speed[3] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//底盘右后
				motor_encoder[3] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x205 : 
				upper_speed = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];			//升降
				upper_encoder = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x206 : 
				trailer_speed = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//拖车
				trailer_encoder = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			default:
				break;
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

