#include "main.h"
#include "trailer.h"
#include "motor.h"

PID_trailer pid_trailer;


void trailer_pidinit(void)				//拖车电机初始化
{
	pid_trailer.Sv[0] = 0;
	pid_trailer.Pv[0] = 0;
	
	pid_trailer.Kp[0] = 0.29;
	pid_trailer.Ki[0] = 0;
	pid_trailer.Kd[0] = 0;
	
	pid_trailer.Ek[0] = 0;
	pid_trailer.Ek_1[0] = 0;
	pid_trailer.sum_Ek[0] = 0;
	
	pid_trailer.location[0] = 0;
	
	pid_trailer.out[0] = 0;
	
	
	pid_trailer.Sv[1] = 0;
	pid_trailer.Pv[1] = 0;
	
	pid_trailer.Kp[1] = 2.4;
	pid_trailer.Ki[1] = 0;
	pid_trailer.Kd[1] = 0;
	
	pid_trailer.Ek[1] = 0;
	pid_trailer.Ek_1[1] = 0;
	pid_trailer.sum_Ek[1] = 0;
	
	pid_trailer.location[1] = 0;
	
	pid_trailer.out[1] = 0;
}



/************************************ 拖车 ********************************/


float trailer_encoder_now = 0.0f;
float trailer_encoder_last = 0.0f;

unsigned char flag_trailer_encoder = 0;
//unsigned char flag_trailer_touch = 0;
unsigned char flag_trailer_motor = 0;
unsigned char flag_trailer = 0;
unsigned char flag_trailer_cancel = 0;
unsigned char trailer_locked_rotor = 0;
unsigned char trailer_locked_rotor_1ms = 0;

void PID_trailer_site ( void )
{
	
	pid_trailer.Ek[0] = pid_trailer.Sv[0] - pid_trailer.Pv[0];
	
	pid_trailer.sum_Ek[0] += pid_trailer.Ek[0];
	
//	pid_trailer.sum_Ek[0] = limit ( pid_trailer.sum_Ek[0], 1000, -1000 );
	
	pid_trailer.out[0] = pid_trailer.Kp[0] * pid_trailer.Ek[0] 
					   + pid_trailer.Ki[0] * pid_trailer.sum_Ek[0] 
					   + pid_trailer.Kd[0] * ( pid_trailer.Ek[0] - pid_trailer.Ek_1[0] );
	
	pid_trailer.out[0] = limit ( pid_trailer.out[0], 2000, -2000 );
	
	pid_trailer.Ek_1[0] = pid_trailer.Ek[0];
}

void PID_trailer_velocity ( void )
{
	
	if ( abs(pid_trailer.Pv[1] ) <= 4 )			//消除编码器的抖动
	{
			pid_trailer.Pv[1] = 0;
	}
	
	pid_trailer.Ek[1] = pid_trailer.Sv[1] - pid_trailer.Pv[1];

	pid_trailer.sum_Ek[1] += pid_trailer.Ek[1];
	
	pid_trailer.sum_Ek[1] = limit ( pid_trailer.sum_Ek[1], 3000, -3000 );
	
	pid_trailer.out[1] = pid_trailer.Kp[1] * pid_trailer.Ek[1]
					   + pid_trailer.Ki[1] * pid_trailer.sum_Ek[1]
					   + pid_trailer.Kd[1] * ( pid_trailer.Ek[1] - pid_trailer.Ek_1[1] );
	
	pid_trailer.out[1] = limit ( pid_trailer.out[1], 8000, -8000 );
	
	pid_trailer.Ek_1[1] = pid_trailer.Ek[1];
	
	if ( trailer_speed == 0 && abs( pid_trailer.out[1] ) > 5000 )
	{
		trailer_locked_rotor ++;
	}
	if ( trailer_locked_rotor >= 150 )
	{
		trailer_locked_rotor = 150;
		pid_trailer.out[1] = limit ( pid_trailer.out[1], 5000, -5000 );
	}
	if ( abs( pid_trailer.out[1] ) < 5000 && trailer_locked_rotor != 0 )
	{
		trailer_locked_rotor = 0;
	}
}

void trailer_motor( void )
{
	
	if ( flag_trailer_motor == 0 )
	{
		
		pid_trailer.Sv[1] = -1000;
		pid_trailer.Pv[1] = trailer_speed;
		PID_trailer_velocity ();
		if ( trailer_speed == 0 && pid_trailer.out[1] < -2000 )
		{
			
			trailer_locked_rotor_1ms ++;
			if ( trailer_locked_rotor_1ms > 50 )
			{
				
				flag_trailer_motor = 1;
				trailer_locked_rotor_1ms = 0;
				
			}
			
		}
		
	}
	if ( flag_trailer_motor == 1 )
	{
		
		trailer_encoder_now = trailer_encoder;
		trailer_encoder_last = trailer_encoder;
		flag_trailer_motor = 2;
		
	}
	
	if ( flag_trailer_motor == 2 )
	{
		
		pid_trailer.Sv[0] = 78000;
		flag_trailer_motor = 3;

	}
	if ( flag_trailer_motor == 3 )
	{
		
		trailer_encoder_now = trailer_encoder;
		
		if ( RC_Ctl.key.v == KEY_Z && flag_trailer == 0 )
		{
			
			back = 1;
			flag_trailer = 1;
			pid_trailer.Sv[0] = 28000;
			flag_trailer_cancel = 0;
			
		}
		if ( RC_Ctl.key.v != KEY_Z && flag_trailer == 1 )
		{
			
			flag_trailer = 2;
			
		}
		if ( RC_Ctl.key.v == KEY_Z && flag_trailer == 2 )
		{
			
			pid_trailer.Ki[1] = 0;
			pid_trailer.Sv[0] = 0;
			flag_trailer = 3;
			
		}
		if ( RC_Ctl.key.v != KEY_Z && flag_trailer == 3 )
		{
			
			flag_trailer = 4;
			
		}
		if ( RC_Ctl.key.v == KEY_Z && flag_trailer == 4 )
		{
			
			flag_trailer = 5;
			
		}
		if ( RC_Ctl.key.v != KEY_Z && flag_trailer == 5 )
		{
			
			back = 0;
			CAN2_Send_Msg( 0, 0, 4 );
			flag_trailer = 0;
			
		}
		
		if ( RC_Ctl.key.v == KEY_X && flag_trailer_cancel == 0 )
		{
			
			back = 1;
			flag_trailer = 0;
			flag_trailer_cancel = 1;
			
		}
		if ( RC_Ctl.key.v != KEY_X && flag_trailer_cancel == 1 )
		{
			
			flag_trailer_cancel = 2;
			
		}
		if ( RC_Ctl.key.v == KEY_X && flag_trailer_cancel == 2 )
		{
			
			pid_trailer.Ki[1] = 0.1;
			pid_trailer.Sv[0] = 78000;
			flag_trailer_cancel = 3;
			
		}
		if ( RC_Ctl.key.v != KEY_X && flag_trailer_cancel == 3 )
		{
			
			flag_trailer_cancel = 4;
			
		}
		if ( RC_Ctl.key.v == KEY_X && flag_trailer_cancel == 4 )
		{
			
			back = 0;
			flag_trailer_cancel = 5;
			
		}
		if ( RC_Ctl.key.v != KEY_X && flag_trailer_cancel == 5 )
		{
			
			CAN2_Send_Msg( 0, 0, 4 );
			flag_trailer_cancel = 0;
			
		}
		
		
		
		pid_trailer.location[0] += PTZencoder ( trailer_encoder_last, trailer_encoder_now );		//78000
		
		pid_trailer.Pv[0] = pid_trailer.location[0];
		
		PID_trailer_site ();
		
		pid_trailer.Sv[1] = pid_trailer.out[0];
		
		pid_trailer.Pv[1] = trailer_speed;
		
		PID_trailer_velocity ();
		
		trailer_encoder_last = trailer_encoder_now;
	}
}
