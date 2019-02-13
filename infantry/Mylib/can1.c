#include "can1.h"
#include "usart3.h"

/**
  * @brief  CAN1�������
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
	
	//ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
	//���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);//PA12����λCAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);//PA11����λCAN1

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
    
    NVIC_InitStructure.NVIC_IRQChannel 						= 	CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 	0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 	1;
    NVIC_InitStructure.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    
    CAN_InitStructure.CAN_TTCM 		= 	DISABLE;	//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM 		= 	DISABLE;	//����Զ����߹���
    CAN_InitStructure.CAN_AWUM 		= 	DISABLE;	//˯��ģʽͨ���������
    CAN_InitStructure.CAN_NART 		= 	DISABLE;	//��ֹ�����Զ�����
    CAN_InitStructure.CAN_RFLM 		= 	DISABLE;	//���Ĳ��������µĸ��Ǿɵ�
    CAN_InitStructure.CAN_TXFP 		= 	ENABLE;	//����FIFO���ȼ��ɱ��ĵı�ʶ��������
    CAN_InitStructure.CAN_Mode 		= 	CAN_Mode_Normal;	//CANӲ������������ģʽ
    CAN_InitStructure.CAN_SJW  		= 	CAN_SJW_1tq;	//����ͬ����Ծ���Ϊ1��ʱ�䵥λ
    CAN_InitStructure.CAN_BS1 		= 	CAN_BS1_9tq;	//ʱ���1ռ��9��ʱ�䵥λ
    CAN_InitStructure.CAN_BS2 		= 	CAN_BS2_4tq;	//ʱ���2ռ��4��ʱ�䵥λ
    CAN_InitStructure.CAN_Prescaler = 	3;   // CAN BaudRate 42/(1+9+4)/3=1Mbps	��Ƶϵ����Fdiv��
    CAN_Init(CAN1, &CAN_InitStructure);	//��ʼ��CAN1

	CAN_FilterInitStructure.CAN_FilterNumber		=	0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode			=	CAN_FilterMode_IdMask;	//ָ���˹�����������ʼ������ģʽ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale			=	CAN_FilterScale_32bit;	//�����˹�����λ��1��32λ������
	CAN_FilterInitStructure.CAN_FilterIdHigh		=	0x0000;	//�����趨��������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ���һ����
	CAN_FilterInitStructure.CAN_FilterIdLow			=	0x0000;	//�����趨��������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ��ڶ�����
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh	=	0x0000;	//�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��߶�λ��16λλ��ʱΪ��һ����
	CAN_FilterInitStructure.CAN_FilterMaskIdLow		=	0x0000;	////�����趨���������α�ʶ�����߹�������ʶ����32λλ��ʱΪ��Ͷ�λ��16λλ��ʱΪ�ڶ�����
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=	0;	//�趨��ָ���������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation	=	ENABLE;	//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	//ʹ��ָ����CAN�ж�
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}


int16_t motor_speed[4] = {0.0f};
int16_t motor_encoder[4] = {0.0f};		/* ���̵��������ֵ */
int16_t upper_speed = {0.0f};
int16_t upper_encoder = {0.0f};
int16_t trailer_speed = {0.0f};
int16_t trailer_encoder = {0.0f};

/*
 * ������CAN1_Send_Msg
 * ���ܣ�can1����
 * ������exdid ��ַ, msg ����, len ���ݳ���
*/

void CAN1_Send_flex_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4)

{
	CanTxMsg tx_message;			//���ͻ�����
	
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

void CAN1_Send_Chassis_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4)		//���̷���

{
	CanTxMsg tx_message;			//���ͻ�����
	
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
				motor_speed[0] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//������ǰ
				motor_encoder[0] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x202 : 
				motor_speed[1] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//������ǰ
				motor_encoder[1] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x203 : 
				motor_speed[2] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//�������
				motor_encoder[2] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x204 : 
				motor_speed[3] = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//�����Һ�
				motor_encoder[3] = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x205 : 
				upper_speed = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];			//����
				upper_encoder = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			case 0x206 : 
				trailer_speed = (int16_t)(rx_message.Data[2] << 8) | rx_message.Data[3];		//�ϳ�
				trailer_encoder = (int16_t)(rx_message.Data[0] << 8) | rx_message.Data[1];
				break;
			
			default:
				break;
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

