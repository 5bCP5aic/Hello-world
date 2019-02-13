#ifndef __UART4_H__
#define __UART4_H__

#include "stdint.h"

#include "stm32f4xx.h"

#define DBUSLength		18				//DBUS数据帧长
#define DBUSBackLength	1				//增加一个字节保持稳定

void UART4_Configuration(void);
void telecontrol_Init( void );
	
/*********基本键值*********/
#define KEY_NONE		0x0000
#define KEY_W			0x0001
#define KEY_S			0x0002
#define KEY_A			0x0004
#define KEY_D			0x0008
#define KEY_SHIFT		0x0010
#define KEY_CTRL		0x0020
#define KEY_Q			0x0040
#define KEY_E			0x0080
#define KEY_R			0x0100
#define KEY_F			0x0200
#define KEY_G			0x0400
#define KEY_Z			0x0800
#define KEY_X			0x1000
#define KEY_C			0x2000
#define KEY_V			0x4000
#define KEY_B			0x8000
#define KEY_WA			0x0005
#define KEY_WD			0x0009
#define KEY_SA			0x0006
#define KEY_SD			0x000A
#define KEY_WQ			0x0041		//65
#define KEY_WE			0x0081		//129
#define KEY_WC			0x2001
#define KEY_SHIFT_W		0x0011
#define KEY_SHIFT_S		0x0012
#define KEY_SHIFT_D		0x0018
#define KEY_SHIFT_A		0x0014
#define KEY_SHIFT_Q		0x0050
#define KEY_SHIFT_G		0x0410
#define KEY_SHIFT_C		0x2010		//8208   
#define KEY_SHIFT_X		0x1010		//4112
#define KEY_SHIFT_Z		0x0810		//2064
#define KEY_CTRL_W		0x0021		//33
#define KEY_CTRL_S		0x0022		//34


typedef struct 
{
    struct     
	{         
		uint16_t ch0;         
		uint16_t ch1;         
		uint16_t ch2;         
		uint16_t ch3;         
		uint8_t  s1;         
		uint8_t  s2;     
	}rc;  
    struct     
	{         
		int16_t x;         
		int16_t y;         
		int16_t z;         
		uint8_t press_l;         
		uint8_t press_r;     
	}mouse;  
	struct     
	{         
	uint16_t v;     
	}key; 
}RC_Ctl_t; 


extern RC_Ctl_t  RC_Ctl;

#endif


