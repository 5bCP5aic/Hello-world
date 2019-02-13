#ifndef __UP_DOWN_H__
#define __UP_DOWN_H__

#include "main.h"

#define box_height		160
#define island_pillar	60
#define pillar_top		800000


//#define		Cylinder1_on		GPIO_SetBits(GPIOB,GPIO_Pin_4)
void PID_up_down_site ( void );
void PID_up_down_velocity ( void );
void up_down_pidinit( void );
void up_down_motor( void );
float impulse_mm( float height );
void up_island ( void );
	
extern int flag_upper_prop;
/*******************************  升降结构体  **********************************************/
typedef struct
{
	float Sv[2];			//set value  设定值 /给定值
	float Pv[2];			//present value	当前值
	
	float Kp[2];
	float Ki[2];
	float Kd[2];
	
	float Ek[2];			//当前误差
	float Ek_1[2];			//上次误差
	float sum_Ek[2];			//上上次误差
	
	float out[2];			//pid计算结果
	
	float location[2];
	
} PID_up_down;

extern PID_up_down pid_up_down;
extern int up_down_locked_rotor;
extern int flag_take;

#endif
