#include "main.h"
#include "up_down.h"
#include "motor.h"

PID_up_down pid_up_down;


void up_down_pidinit(void)				//升降电机初始化
{
	pid_up_down.Sv[0] = 0;
	pid_up_down.Pv[0] = 0;
	
	pid_up_down.Kp[0] = 0.3;
	pid_up_down.Ki[0] = 0;
	pid_up_down.Kd[0] = 0;
	
	pid_up_down.Ek[0] = 0;
	pid_up_down.Ek_1[0] = 0;
	pid_up_down.sum_Ek[0] = 0;
	
	pid_up_down.location[0] = 0;
	
	pid_up_down.out[0] = 0;
	
	
	pid_up_down.Sv[1] = 0;
	pid_up_down.Pv[1] = 0;
	
	pid_up_down.Kp[1] = 3.9;
	pid_up_down.Ki[1] = 0.35;
	pid_up_down.Kd[1] = 0;
	
	pid_up_down.Ek[1] = 0;
	pid_up_down.Ek_1[1] = 0;
	pid_up_down.sum_Ek[1] = 0;
	
	pid_up_down.location[1] = 0;
	
	pid_up_down.out[1] = 0;
}

/************************************* mm转换为脉冲 **************************************/

float up_down_1mm = 0;

float impulse_mm( float height )
{
	up_down_1mm = pillar_top / 445;
	
	height = height * up_down_1mm;
	
	return height;
}



/************************************ 升降 ********************************/

int up_down_flag = 0;
int flag_upper_prop = 0;
int up_down_locked_rotor = 0;

void PID_up_down_site ( void )
{
	pid_up_down.Sv[0] = limit ( pid_up_down.Sv[0], 810000, -1000 );

	pid_up_down.Ek[0] = pid_up_down.Sv[0] - pid_up_down.Pv[0];
	
	pid_up_down.sum_Ek[0] += pid_up_down.Ek[0];
	
	pid_up_down.sum_Ek[0] = limit ( pid_up_down.sum_Ek[0], 1000, -1000 );
	
	pid_up_down.out[0] = pid_up_down.Kp[0] * pid_up_down.Ek[0] 
					   + pid_up_down.Ki[0] * pid_up_down.sum_Ek[0] 
					   + pid_up_down.Kd[0] * ( pid_up_down.Ek[0] - pid_up_down.Ek_1[0] );
	
//	pid_up_down.out[0] = limit ( pid_up_down.out[0], 2000, -2000 );
	
	pid_up_down.Ek_1[0] = pid_up_down.Ek[0];
}

void PID_up_down_velocity ( void )
{
	
	if ( up_down_flag == 1 )
	{
		
		if ( flag_upper_prop == 0 )		//平常上升
		{
			
			pid_up_down.Kp[0] = 0.3;
			pid_up_down.Ki[1] = 0.68;
			pid_up_down.Kd[1] = 5;
			pid_up_down.Sv[1] = limit ( pid_up_down.Sv[1], 6000, -6000 );
			
		}
		
		if ( flag_upper_prop == 2 )		//抱柱下降
		{
			
			pid_up_down.Ki[1] = 0.28;
			pid_up_down.Kd[1] = 5;
			pid_up_down.Sv[1] = limit ( pid_up_down.Sv[1], 5000, -5000 );
			
		}
		
	}
	if ( up_down_flag == 2 )
	{
		if ( flag_upper_prop == 0 )		//平常下降
		{
			pid_up_down.Kp[0] = 0.3;
			pid_up_down.Ki[1] = 0.34;		
			pid_up_down.Kd[1] = 5;
			pid_up_down.Sv[1] = limit ( pid_up_down.Sv[1], 2500, -2500 );
		}
		if ( flag_upper_prop == 1 )		//抱柱上升
		{
			
			pid_up_down.Kp[0] = 0.4;
			pid_up_down.Ki[1] = 0.64;
			pid_up_down.Kd[1] = 3;
			pid_up_down.Sv[1] = limit ( pid_up_down.Sv[1], 7000, -7000 );
			
		}
	}
	
	if ( up_down_flag == 0 )
	{
		
		pid_up_down.Kp[0] = 0.3;
		pid_up_down.Ki[1] = 0.45;		//停止
		
	}
	
	if ( abs( pid_up_down.Pv[1] ) <= 5 )
	{
		
			pid_up_down.Pv[1] = 0;
		
	}
	
	pid_up_down.Ek[1] = pid_up_down.Sv[1] - pid_up_down.Pv[1];

	pid_up_down.sum_Ek[1] += pid_up_down.Ek[1];
	
	pid_up_down.sum_Ek[1] = limit ( pid_up_down.sum_Ek[1], 8000, -8000 );
	
	pid_up_down.out[1] = pid_up_down.Kp[1] * pid_up_down.Ek[1] 
					   + pid_up_down.Ki[1] * pid_up_down.sum_Ek[1] 
					   + pid_up_down.Kd[1] * ( pid_up_down.Ek[1] - pid_up_down.Ek_1[1] );
	
	pid_up_down.out[1] = limit ( pid_up_down.out[1], 15000, -15000 );
	
	pid_up_down.Ek_1[1] = pid_up_down.Ek[1];
	
	if ( upper_speed == 0 && abs( pid_up_down.out[1] ) > 8000 )
	{
		up_down_locked_rotor ++;
	}
	if ( up_down_locked_rotor >= 200 )
	{
		up_down_locked_rotor = 200;
		pid_up_down.out[1] = limit ( pid_up_down.out[1], 8000, -8000 );
	}
	if ( abs( pid_up_down.out[1] ) < 8000 && up_down_locked_rotor != 0 )
	{
		up_down_locked_rotor = 0;
	}
}

float up_down_motor_encoder_now = 0;
float up_down_motor_encoder_last = 0;

int up_down_motor_flag = 0;
int up_down_touchness = 0;
int up_pillar_touchness = 0;
int up_pillar = 0;

int flag_Cylinder1 = 0;		//抱柱爪子开合标志位
int flag_Cylinder2 = 0;		//抱柱爪子伸缩标志位
int flag_Cylinder_time = 0;

int flag_take = 0;

int flag_repristination = 0; //恢复状态

int flag_up_pillar_claw = 0;

void up_island ( void )
{
	
	/************* 上岛 ********************/	//D3  左    	D2  右
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 0 && flag_Cylinder2 == 0 )
	{
		
		CAN2_Send_Msg( 0, 0, 3 );
		pid_up_down.Sv[0] = pillar_top;
		flag_upper_prop = 0;		//平常上升
		flag_Cylinder2 = 1;
		flag_Cylinder1 = 1;
		back = 1;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 1 && flag_Cylinder2 == 1 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0 )
	{
		
		CAN2_Send_Msg( 0, 1, 3 );
		flag_Cylinder2 = 2;
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 1 && flag_Cylinder2 == 2 )
	{
		
		CAN2_Send_Msg( 1, 1, 1 );
		flag_Cylinder1 = 2;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 2 && flag_Cylinder2 == 2 )
	{
		
		flag_Cylinder2 = 3;
		flag_Cylinder1 = 3;
		
	}
	if ( RC_Ctl.key.v == KEY_F || take == 2 )
	{
		if ( flag_Cylinder1 == 3 && flag_Cylinder2 == 3 )
		{
			flag_Cylinder_time ++;
		if ( flag_Cylinder_time >= 300 )
		{
			pid_up_down.Sv[0] = 0;
			flag_upper_prop = 1;		//抱柱上升
			flag_Cylinder2 = 4;
			flag_Cylinder1 = 4;
			flag_Cylinder_time = 0;
		}
			
		}
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 4 && flag_Cylinder2 == 4 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 )
	{
		
		flag_Cylinder2 = 5;
		flag_Cylinder1 = 4;
		
	}
	if ( flag_Cylinder1 == 4 && flag_Cylinder2 == 5 )
	{
		if ( ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 0 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 ) || ( GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0 ) )
		{
			
			flag_Cylinder2 = 5;
			flag_Cylinder1 = 5;
			
		}
		
	}
	if ( flag_Cylinder1 == 5 && flag_Cylinder2 == 5 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 )
	{
		
		pid_up_down.Sv[0] = impulse_mm( island_pillar );
		flag_upper_prop = 2;		//抱柱下降
		flag_Cylinder2 = 6;
		flag_Cylinder1 = 6;
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 6 && flag_Cylinder2 == 6 )
	{
		
		CAN2_Send_Msg( 0, 1, 3 );
		flag_Cylinder2 = 6;
		flag_Cylinder1 = 7;
		
	}
	
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 7 && flag_Cylinder2 == 6 )
	{
		
		flag_Cylinder_time ++;
		if ( flag_Cylinder_time >= 150 )
		{
			
			CAN2_Send_Msg( 0, 0, 4 );
			flag_upper_prop = 0;
			flag_Cylinder2 = 7;
			flag_Cylinder1 = 7;
			flag_Cylinder_time = 0;
			take = 0;
			
		}
		
		back = 0;
		
	}
	
	
	/************* 下岛 ********************/
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 7 && flag_Cylinder2 == 7 )
	{
//		printf("\r\n----------------下岛模式----------------------\r\n");
//		auto_control_flag = DISABLE;  /* 退出自动模式 */
		pid_up_down.Sv[0] = impulse_mm( island_pillar );
		flag_upper_prop = 0;		//平常
		CAN2_Send_Msg( 0, 1, 3 );
		flag_Cylinder2 = 8;
		back = 1;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 7 && flag_Cylinder2 == 8 )
	{
		
		flag_Cylinder2 = 9;
		flag_Cylinder1 = 8;
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 8 && flag_Cylinder2 == 9 )
	{
		
		CAN2_Send_Msg( 1, 1, 1 );
		flag_Cylinder2 = 9;
		flag_Cylinder1 = 9;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 9 && flag_Cylinder2 == 9 )
	{
		
		flag_Cylinder2 = 10;
		flag_Cylinder1 = 10;
		
	}
	if ( flag_Cylinder1 == 10 && flag_Cylinder2 == 10 )
	{
		if ( RC_Ctl.key.v == KEY_F || take == 2 )
		{
			
			pid_up_down.Sv[0] = 0;
			flag_upper_prop = 1;		//抱柱上升
			flag_Cylinder2 = 11;
			flag_Cylinder1 = 11;
			
		}
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 11 && flag_Cylinder2 == 11 )
	{
		
		flag_Cylinder2 = 12;
		flag_Cylinder1 = 12;
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 12 && flag_Cylinder2 == 12 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 1 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 1 )
	{
		
		pid_up_down.Sv[0] = pillar_top;
		flag_upper_prop = 2;		//抱柱下降
		flag_Cylinder2 = 13;
		flag_Cylinder1 = 13;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 13 && flag_Cylinder2 == 13 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) == 0 && GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0 )
	{
		
		flag_Cylinder2 = 14;
		flag_Cylinder1 = 14;
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 14 && flag_Cylinder2 == 14 )
	{
		
		CAN2_Send_Msg( 0, 1, 3 );
		flag_Cylinder2 = 14;
		flag_Cylinder1 = 15;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 15 && flag_Cylinder2 == 14 )
	{
		
		flag_Cylinder_time ++;
		if ( flag_Cylinder_time >= 150 )
		{
			
			CAN2_Send_Msg( 0, 0, 4 );
			flag_Cylinder2 = 15;
			flag_Cylinder1 = 15;
			flag_Cylinder_time = 0;
			take = 0;
			
		}
		
	}
	if ( RC_Ctl.key.v == KEY_F && flag_Cylinder1 == 15 && flag_Cylinder2 == 15 )
	{
		pid_up_down.Sv[0] = 0;
		flag_upper_prop = 0;		//平常下降
		flag_Cylinder2 = 16;
		flag_Cylinder1 = 16;
		
	}
	if ( RC_Ctl.key.v != KEY_F && flag_Cylinder1 == 16 && flag_Cylinder2 == 16 )
	{
		
		flag_Cylinder2 = 0;
		flag_Cylinder1 = 0;
		back = 0;
		
	}
	
/*************** 取弹 ***************/
	if ( RC_Ctl.key.v == KEY_G )
	{
		
		pid_up_down.Sv[0] = impulse_mm( box_height );
		flag_upper_prop = 0;
		flag_Cylinder2 = 7;
		flag_Cylinder1 = 7;
		flag_take = 1;
		take = 0;
		back = 0;
		
	}
	if ( RC_Ctl.key.v == KEY_SHIFT_G )
	{
		
		pid_up_down.Sv[0] = impulse_mm( box_height );
		flag_upper_prop = 0;
		flag_Cylinder2 = 7;
		flag_Cylinder1 = 7;
		flag_take = 0;
		take = 0;
		back = 0;
		
	}
	
/*********** 交接 ***************/
	if ( ( flag_Cylinder1 == 7 && flag_Cylinder2 == 7 ) || ( flag_Cylinder1 == 8 && flag_Cylinder2 == 9 ) )
	{
		if ( RC_Ctl.key.v == KEY_R )
		{
			
			pid_up_down.Sv[0] = impulse_mm( 230 );
			flag_upper_prop = 0;
			flag_Cylinder2 = 7;
			flag_Cylinder1 = 7;
			back = 0;
			
		}
		
		
	}
	
	if ( RC_Ctl.key.v == KEY_R && flag_Cylinder1 == 0 && flag_Cylinder2 == 0 )
	{
		
		pid_up_down.Sv[0] = pillar_top;
		flag_upper_prop = 0;
		flag_Cylinder2 = 0;
		flag_Cylinder1 = 0;
		back = 0;
		
	}
	
/**************** 最高 最低 *******************/
	if ( RC_Ctl.key.v == KEY_SHIFT_W )
	{
		
		pid_up_down.Sv[0] = pillar_top;
		flag_upper_prop = 0;
		flag_up_pillar_claw = 0;
		flag_Cylinder2 = 0;
		flag_Cylinder1 = 0;
		flag_take = 0;
		take = 0;
		back = 0;
		
	}
	if ( RC_Ctl.key.v == KEY_SHIFT_S )
	{
		
		flag_repristination = 1;
		flag_up_pillar_claw = 0;
		pid_up_down.Sv[0] = 0;
		flag_upper_prop = 0;
		flag_Cylinder2 = 0;
		flag_Cylinder1 = 0;
		flag_take = 0;
		take = 0;
		back = 0;
		
	}
	
/*********** 微调 *******************/
	if ( RC_Ctl.key.v == KEY_CTRL_W )
	{
		
		flag_upper_prop = 0;
		pid_up_down.Sv[0] += 300;
		
	}
	if ( RC_Ctl.key.v == KEY_CTRL_S )
	{
		
		flag_upper_prop = 0;
		pid_up_down.Sv[0] -= 300;
		
	}
	if ( RC_Ctl.rc.s1 == 3 )
	{
		
		pid_up_down.Sv[0] += ( float ) ( ( RC_Ctl.rc.ch1 - 1024 ) * 3 );
		
	}
	
/********* 上岛爪子状态 ************/
	
	if ( RC_Ctl.key.v == KEY_B && flag_up_pillar_claw == 0 )
	{
		
		CAN2_Send_Msg( 0, 0, 0 );
		flag_up_pillar_claw = 1;
		
	}
	if ( RC_Ctl.key.v != KEY_B && flag_up_pillar_claw == 1 )
	{
		
		flag_up_pillar_claw = 2;
		
	}
	if ( RC_Ctl.key.v == KEY_B && flag_up_pillar_claw == 2 )
	{
		
		CAN2_Send_Msg( 0, 1, 0 );
		flag_up_pillar_claw = 3;
		
	}
	if ( RC_Ctl.key.v != KEY_B && flag_up_pillar_claw == 3 )
	{
		
		flag_up_pillar_claw = 4;
		
	}
	if ( RC_Ctl.key.v == KEY_B && flag_up_pillar_claw == 4 )
	{
		
		CAN2_Send_Msg( 1, 1, 0 );
		flag_up_pillar_claw = 5;
		
	}
	if ( RC_Ctl.key.v != KEY_B && flag_up_pillar_claw == 5 )
	{
		
		flag_up_pillar_claw = 0;
		
	}
}



void up_down_motor( void )//shang  A6
{
	
	if ( up_down_motor_flag == 0 )
	{
		
		up_down_motor_encoder_now = upper_encoder;
		up_down_motor_encoder_last = upper_encoder;
		up_down_motor_flag = 1;
		
	}
	if ( up_down_motor_flag == 1 )
	{
		
		up_down_motor_encoder_now = upper_encoder;
		
		pid_up_down.location[0] += PTZencoder ( up_down_motor_encoder_last, up_down_motor_encoder_now );
		
		if ( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1 )
			{
				
				up_down_touchness = 0;
				
			}
			
			if ( flag_repristination == 1 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1 )
			{
				
				CAN2_Send_Msg( 0, 0, 4 );
				flag_repristination = 0;
				
			}
			if ( pid_up_down.location[0] < 70000 )
			{
				
				if ( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 1 && up_down_touchness != 2 )
				{
					
					pid_up_down.location[0] = 0;
					up_pillar_touchness = 2;
					up_down_touchness = 2;
					up_pillar = 0;
					
					
				}
				
			}
			
			if ( pid_up_down.location[0] > 770000 )
			{
				
				if ( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1 && up_down_touchness != 1 )
				{
					
					pid_up_down.location[0] = pillar_top;
					up_down_touchness = 1;
					up_pillar_touchness = 1;
				}
				
			}
			
		
		
		
		pid_up_down.Pv[0] = pid_up_down.location[0];
			
		if ( pid_up_down.Sv[0] - pid_up_down.Pv[0] > 100 )
		{
			
			up_down_flag = 1;							//上升
			
		}
		else
			if ( pid_up_down.Sv[0] - pid_up_down.Pv[0] < 100 )
			{
				
				up_down_flag = 2;							//下降
				
			}
			else
			{
				
				up_down_flag = 0;							//停止
				
			}
			
			up_island ();
			
			
				
//			if ( flag_Cylinder1 == 5 || flag_Cylinder1 == 12 )
//			{
//				
//				if ( up_pillar_touchness != 2 && pid_up_down.Sv[0] == 0 && pid_up_down.Ek[0] > -290000 && abs (pid_up_down.Pv[1] ) <= 50 )
//				{
//					
//					up_pillar = 1;
//					
//				}
//				
//			}
			
		PID_up_down_site ();
		
		pid_up_down.Sv[1] = pid_up_down.out[0];
		
//		if ( up_pillar == 1 )
//		{
//			
//			pid_up_down.Sv[1] = -3500;
//			flag_upper_prop = 1;
//			
//		}
		
		pid_up_down.Pv[1] = upper_speed;
		
		PID_up_down_velocity ();
		
		up_down_motor_encoder_last = up_down_motor_encoder_now;
		
	}
}
