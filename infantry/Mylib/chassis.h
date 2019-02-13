#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"

#define chassis_max_speed				8000	//6500
#define chassis_max_rotate_speed		4000	//3000
#define chassis_slow_speed				5500	//5000
#define chassis_slow_rotate_speed		3000	//2000
#define chassis_normal_low_speed		3400	//1800
#define chassis_normal_low_rotate_speed	1700	//1000
#define chassis_low_speed				1000
#define chassis_low_rotate_speed		1000
#define rotate_180_time					570
#define auto_move_x						800

void IncPIDInit_200(void);
void PID_count_201 (void);
void PID_count_202 (void);
void PID_count_203 (void);
void PID_count_204 (void);

void chassis_alg( void );
void chassis_motor( void );

typedef struct
{
	float Sv[4];			//set value  设定值 /给定值
	float Pv[4];			//present value	当前值
	
	
	float A[4];
	float B[4];
	float C[4];
	
	float Ek[4];			//当前误差
	float Ek_1[4];			//上次误差
	float Ek_2[4];			//上上次误差
	
	float out[4];			//pid计算结果
	
	float location[4];
	
} PID_200;

extern PID_200 pid_200;

extern float left_front;
extern float right_front;
extern float left_back;
extern float right_back;
extern float speed_x;
extern float speed_y;
extern float rock_direction;

extern int back;

extern unsigned char auto_control_flag;     /* 自动控制标志位 */


#endif

