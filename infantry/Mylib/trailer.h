#ifndef __TRAILER_H__
#define __TRAILER_H__

#include "main.h"

void PID_trailer_site ( void );
void PID_trailer_velocity ( void );
void trailer_pidinit( void );
void trailer_motor( void );

/*******************************  �ϳ��ṹ��  **********************************************/
typedef struct
{
	float Sv[2];			//set value  �趨ֵ /����ֵ
	float Pv[2];			//present value	��ǰֵ
	
	float Kp[2];
	float Ki[2];
	float Kd[2];
	
	float Ek[2];			//��ǰ���
	float Ek_1[2];			//�ϴ����
	float sum_Ek[2];			//���ϴ����
	
	float out[2];			//pid������
	
	float location[2];
	
} PID_trailer;

extern PID_trailer pid_trailer;

#endif
