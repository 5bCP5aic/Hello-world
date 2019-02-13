#ifndef __CAN2_H__
#define __CAN2_H__
#include <stm32f4xx.h>

void CAN2_Configuration(void);
void CAN2_Send_D_BUS_Msg( int16_t Vel1, int16_t Vel2, int8_t Vel3, int8_t Vel4 );
void CAN2_Send_Msg( int8_t Cylinder1, int8_t Cylinder2, int8_t dbus_key );

extern int8_t photoswitch1;
extern int8_t photoswitch2;
extern int8_t take;

#endif 
