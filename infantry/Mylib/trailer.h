#ifndef __TRAILER_H__
#define __TRAILER_H__

#include "main.h"

void PID_trailer_site ( void );
void PID_trailer_velocity ( void );
void trailer_pidinit( void );
void trailer_motor( void );

/*******************************  拖车结构体  **********************************************/
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
	
} PID_trailer;

extern PID_trailer pid_trailer;

#endif
