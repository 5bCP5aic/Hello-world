#include "main.h"
#include "line_patrol.h"
#include "motor.h"


PID_line_patrol_x pid_line_patrol_x;
PID_line_patrol_rotate pid_line_patrol_rotate;

void line_patrol_pidinit(void)				//巡线初始化
{
/***************************** 巡线 左右 初始化 *************************/
	pid_line_patrol_x.Sv = 0;
	pid_line_patrol_x.Pv = 0;
	
	pid_line_patrol_x.Kp = 1.5;
	pid_line_patrol_x.Ki = 0;
	pid_line_patrol_x.Kd = 0;
	
	pid_line_patrol_x.Ek = 0;
	pid_line_patrol_x.Ek_1 = 0;
	pid_line_patrol_x.sum_Ek = 0;
	
	pid_line_patrol_x.location = 0;
	
	pid_line_patrol_x.out = 0;
	
/***************************** 巡线 旋转 初始化 *************************/
	
	pid_line_patrol_rotate.Sv = 0;
	pid_line_patrol_rotate.Pv = 0;
	
	pid_line_patrol_rotate.Kp = 8;
	pid_line_patrol_rotate.Ki = 0;
	pid_line_patrol_rotate.Kd = 0;
	
	pid_line_patrol_rotate.Ek = 0;
	pid_line_patrol_rotate.Ek_1 = 0;
	pid_line_patrol_rotate.sum_Ek = 0;
	
	pid_line_patrol_rotate.location = 0;
	
	pid_line_patrol_rotate.out = 0;
	
}
/************************************ 巡线 x ********************************/
int flag_line_patrol_x = 0;


void PID_line_patrol_x_site ( void )
{
	
	pid_line_patrol_x.Ek = pid_line_patrol_x.Sv - pid_line_patrol_x.Pv;
	
	pid_line_patrol_x.sum_Ek += pid_line_patrol_x.Ek;
	
//	pid_line_patrol_x.sum_Ek = limit ( pid_line_patrol_x.sum_Ek, 1000, -1000 );
	
	pid_line_patrol_x.out = pid_line_patrol_x.Kp * pid_line_patrol_x.Ek 
						  + pid_line_patrol_x.Ki * pid_line_patrol_x.sum_Ek 
						  + pid_line_patrol_x.Kd * ( pid_line_patrol_x.Ek - pid_line_patrol_x.Ek_1 );
	
//	pid_line_patrol_x.out = limit ( pid_line_patrol_x.out, 2000, -2000 );
	
	pid_line_patrol_x.Ek_1 = pid_line_patrol_x.Ek;
}



/************************************ 巡线 旋转 ********************************/

void PID_line_patrol_rotate_site ( void )
{
	
	pid_line_patrol_rotate.Ek = pid_line_patrol_rotate.Sv - pid_line_patrol_rotate.Pv;
	
	pid_line_patrol_rotate.sum_Ek += pid_line_patrol_rotate.Ek;
	
//	pid_line_patrol_rotate.sum_Ek = limit ( pid_line_patrol_rotate.sum_Ek, 1000, -1000 );
	
	pid_line_patrol_rotate.out = pid_line_patrol_rotate.Kp * pid_line_patrol_rotate.Ek 
							   + pid_line_patrol_rotate.Ki * pid_line_patrol_rotate.sum_Ek 
							   + pid_line_patrol_rotate.Kd * ( pid_line_patrol_rotate.Ek - pid_line_patrol_rotate.Ek_1 );
	
//	pid_line_patrol_rotate.out = limit ( pid_line_patrol_rotate.out, 2000, -2000 );
	
	pid_line_patrol_rotate.Ek_1 = pid_line_patrol_rotate.Ek;
}


void line_patrol_correct ( void )
{
	pid_line_patrol_x.Sv = 0;
	pid_line_patrol_x.Pv = ( float ) E_horizontal;
	PID_line_patrol_x_site ();
	pid_line_patrol_rotate.Sv = 0;
	pid_line_patrol_rotate.Pv = E_angular;
	PID_line_patrol_rotate_site ();
	
}
