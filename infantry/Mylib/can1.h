#ifndef __CAN1_H__
#define __CAN1_H__
#include "stm32f4xx.h"

void CAN1_Configuration(void);
void CAN1_Send_Chassis_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4);
void CAN1_Send_flex_Msg(int16_t Vel1,int16_t Vel2,int16_t Vel3,int16_t Vel4);

extern int16_t motor_speed[4];
extern int16_t motor_encoder[4];
extern int16_t upper_speed;
extern int16_t upper_encoder;
extern int16_t trailer_speed;
extern int16_t trailer_encoder;

#endif 
