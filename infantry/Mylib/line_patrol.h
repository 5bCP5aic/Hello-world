#ifndef __LINE_PATROL_H__
#define __LINE_PATROL_H__

#include "main.h"

void line_patrol_pidinit( void );
void PID_line_patrol_x_site ( void );
void PID_line_patrol_rotate_site ( void );
void line_patrol_correct ( void );

/*******************************  Ѳ�߽ṹ��  **********************************************/
typedef struct
{
	float Sv;			//set value  �趨ֵ /����ֵ
	float Pv;			//present value	��ǰֵ
	
	float Kp;
	float Ki;
	float Kd;
	
	float Ek;			//��ǰ���
	float Ek_1;			//�ϴ����
	float sum_Ek;			//���ϴ����
	
	float out;			//pid������
	
	float location;
	
} PID_line_patrol_x;

extern PID_line_patrol_x pid_line_patrol_x;

typedef struct
{
	float Sv;			//set value  �趨ֵ /����ֵ
	float Pv;			//present value	��ǰֵ
	
	float Kp;
	float Ki;
	float Kd;
	
	float Ek;			//��ǰ���
	float Ek_1;			//�ϴ����
	float sum_Ek;			//���ϴ����
	
	float out;			//pid������
	
	float location;
	
} PID_line_patrol_rotate;

extern PID_line_patrol_rotate pid_line_patrol_rotate;

#endif
