#ifndef __LINE_PATROL_H__
#define __LINE_PATROL_H__

#include "main.h"

void line_patrol_pidinit( void );
void PID_line_patrol_x_site ( void );
void PID_line_patrol_rotate_site ( void );
void line_patrol_correct ( void );

/*******************************  巡线结构体  **********************************************/
typedef struct
{
	float Sv;			//set value  设定值 /给定值
	float Pv;			//present value	当前值
	
	float Kp;
	float Ki;
	float Kd;
	
	float Ek;			//当前误差
	float Ek_1;			//上次误差
	float sum_Ek;			//上上次误差
	
	float out;			//pid计算结果
	
	float location;
	
} PID_line_patrol_x;

extern PID_line_patrol_x pid_line_patrol_x;

typedef struct
{
	float Sv;			//set value  设定值 /给定值
	float Pv;			//present value	当前值
	
	float Kp;
	float Ki;
	float Kd;
	
	float Ek;			//当前误差
	float Ek_1;			//上次误差
	float sum_Ek;			//上上次误差
	
	float out;			//pid计算结果
	
	float location;
	
} PID_line_patrol_rotate;

extern PID_line_patrol_rotate pid_line_patrol_rotate;

#endif
