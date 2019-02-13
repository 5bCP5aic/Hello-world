#include "main.h"
#include "chassis.h"
#include "motor.h"
#include "line_patrol.h"

PID_200 pid_200;				//存放pid算法所需的数据


void IncPIDInit_200(void)                            //底盘电机参数初始化
{
	int i;
	for ( i = 0; i < 4; i++ )
	{
		pid_200.Sv[i] = 0.0f ;
	}
	for ( i = 0; i < 4; i++ )
	{
		pid_200.Pv[i] = 0.0f;
	}		
	
		pid_200.A[0] = 3.9f;//
		pid_200.A[1] = 4.4f;//
		pid_200.A[2] = 4.4f;//
		pid_200.A[3] = 5.5f;//
	
		pid_200.B[0] = 0.05f;//.8
		pid_200.B[1] = 0.05f;//.8
		pid_200.B[2] = 0.05f;//.8
		pid_200.B[3] = 0.05f;//.8
	
		pid_200.C[0] = 0.0f;//.5
		pid_200.C[1] = 0.0f;//.5
		pid_200.C[2] = 0.0f;//.5
		pid_200.C[3] = 0.0f;//.5
	
	for ( i = 0; i < 4; i++ )
	{
		pid_200.Ek[i] = 0.0f;
	}
	for ( i = 0; i < 4; i++ )
	{
		pid_200.Ek_1[i] = 0.0f;
	}
	for ( i = 0; i < 4; i++ )
	{
		pid_200.Ek_2[i] = 0.0f;
	}
	for ( i = 0; i < 4; i++ )
	{
		pid_200.out[i] = 0.0f;
	}
	for ( i = 0; i < 4; i++ )
	{
		pid_200.location[i] = 0.0f;
	}
	
}


int pid_201_locked_rotor = 0;
int pid_202_locked_rotor = 0;
int pid_203_locked_rotor = 0;
int pid_204_locked_rotor = 0;

/*********************************    201_pid     **********************************/

void PID_count_201 (void)
{
	
	float Incpid_201;
	
	pid_200.Ek[0] = pid_200.Sv[0] - pid_200.Pv[0];
	
	
	Incpid_201 = pid_200.A[0] * ( pid_200.Ek[0] - pid_200.Ek_1[0] ) + pid_200.B[0] * pid_200.Ek[0] + pid_200.C[0] * ( pid_200.Ek[0] - 2 * pid_200.Ek_1[0] + pid_200.Ek_2[0] );
	
	
	pid_200.out[0] += Incpid_201;
	
	pid_200.out[0] = limit ( pid_200.out[0], 13000, -13000 );
	
	pid_200.Ek_2[0] = pid_200.Ek_1[0];
	pid_200.Ek_1[0] = pid_200.Ek[0];
	
	if ( motor_speed[0] == 0 && abs( pid_200.out[0] ) > 8000 )
	{
		pid_201_locked_rotor ++;
	}
	if ( pid_201_locked_rotor >= 100 )
	{
		pid_201_locked_rotor = 100;
		pid_200.out[0] = limit ( pid_200.out[0], 8000, -8000 );
	}
	if ( abs( pid_200.out[0] ) < 8000 && pid_201_locked_rotor != 0 )
	{
		pid_201_locked_rotor = 0;
	}
	
}

/*********************************    202_pid     **********************************/
void PID_count_202 (void)
{
	
	float Incpid_202;
	
	pid_200.Ek[1] = pid_200.Sv[1] - pid_200.Pv[1];
	
	
	Incpid_202 = pid_200.A[1] * ( pid_200.Ek[1] - pid_200.Ek_1[1] ) + pid_200.B[1] * pid_200.Ek[1] + pid_200.C[1] * ( pid_200.Ek[1] - 2 * pid_200.Ek_1[1] + pid_200.Ek_2[1] );
	
	
	pid_200.out[1] += Incpid_202;
	
	pid_200.out[1] = limit ( pid_200.out[1], 13000, -13000 );
	
	pid_200.Ek_2[1] = pid_200.Ek_1[1];
	pid_200.Ek_1[1] = pid_200.Ek[1];
	
	if ( motor_speed[1] == 0 && abs( pid_200.out[1] ) > 8000 )
	{
		pid_202_locked_rotor ++;
	}
	if ( pid_202_locked_rotor >= 100 )
	{
		pid_202_locked_rotor = 100;
		pid_200.out[1] = limit ( pid_200.out[1], 8000, -8000 );
	}
	if ( abs( pid_200.out[1] ) < 8000 && pid_202_locked_rotor != 0 )
	{
		pid_202_locked_rotor = 0;
	}
	
}

/*********************************    203_pid     **********************************/
void PID_count_203 (void)
{
	
	float Incpid_203;
	
	pid_200.Ek[2] = pid_200.Sv[2] - pid_200.Pv[2];
	
	
	Incpid_203 = pid_200.A[2] * ( pid_200.Ek[2] - pid_200.Ek_1[2] ) + pid_200.B[2] * pid_200.Ek[2] + pid_200.C[2] * ( pid_200.Ek[2] - 2 * pid_200.Ek_1[2] + pid_200.Ek_2[2] );
	
	
	pid_200.out[2] += Incpid_203;
	
	pid_200.out[2] = limit ( pid_200.out[2], 13000, -13000 );
	
	pid_200.Ek_2[2] = pid_200.Ek_1[2];
	pid_200.Ek_1[2] = pid_200.Ek[2];
	
	if ( motor_speed[2] == 0 && abs( pid_200.out[2] ) > 8000 )
	{
		pid_203_locked_rotor ++;
	}
	if ( pid_203_locked_rotor >= 100 )
	{
		pid_203_locked_rotor = 100;
		pid_200.out[2] = limit ( pid_200.out[2], 8000, -8000 );
	}
	if ( abs( pid_200.out[2] ) < 8000 && pid_203_locked_rotor != 0 )
	{
		pid_203_locked_rotor = 0;
	}
	
}

/*********************************    204_pid     **********************************/
void PID_count_204 (void)
{
	
	float Incpid_204;
	
	pid_200.Ek[3] = pid_200.Sv[3] - pid_200.Pv[3];
	
	
	Incpid_204 = pid_200.A[3] * ( pid_200.Ek[3] - pid_200.Ek_1[3] ) + pid_200.B[3] * pid_200.Ek[3] + pid_200.C[3] * ( pid_200.Ek[3] - 2 * pid_200.Ek_1[3] + pid_200.Ek_2[3] );
	
	pid_200.out[3] += Incpid_204;
	pid_200.out[3] = limit ( pid_200.out[3], 13000, -13000 );
	
	
	pid_200.Ek_2[3] = pid_200.Ek_1[3];
	pid_200.Ek_1[3] = pid_200.Ek[3];
	
	if ( motor_speed[3] == 0 && abs( pid_200.out[3] ) > 8000 )
	{
		pid_204_locked_rotor ++;
	}
	if ( pid_204_locked_rotor >= 100 )
	{
		pid_204_locked_rotor = 100;
		pid_200.out[3] = limit ( pid_200.out[3], 8000, -8000 );
	}
	if ( abs( pid_200.out[3] ) < 8000 && pid_204_locked_rotor != 0 )
	{
		pid_204_locked_rotor = 0;
	}
	
}


float left_front = 0;
float right_front = 0;
float left_back = 0;
float right_back = 0;
float speed_x = 0.0f;
float speed_y = 0.0f;
float rock_direction = 0.0f;
float flag_photoswitch = 0;

void chassis_alg()
{
	left_front = speed_y + speed_x + rock_direction;//1050
	left_front = limit ( left_front, chassis_max_speed, -chassis_max_speed );
	
	right_front = -(speed_y - speed_x) + rock_direction;
	right_front = limit ( right_front, chassis_max_speed, -chassis_max_speed );
	
	left_back = speed_y - speed_x + rock_direction;
	left_back = limit ( left_back, chassis_max_speed, -chassis_max_speed );
	
	right_back = -(speed_y + speed_x ) + rock_direction;
	right_back = limit ( right_back, chassis_max_speed, -chassis_max_speed );
	
}	


float speed_transition[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
int flag_KEY_W = 0;
int KEY_W_time = 0;

int back = 0;
int take_flag = 0;
int move_r_l_flag = 0;
int auto_move_flag = 0;

unsigned char auto_control_flag = 0;     /* 自动控制标志位 */

int flag_rotate_180 = 0;
int flag_rotate_1ms = 0;

int flag_low_speed = 0;

int flag_brake = 0;
int brake_1ms = 0;
int flag_brake_r_l = 0;

void chassis_motor( void )
{
	
/************** 方向 *******************/
	if( RC_Ctl.rc.s1 == 1  && auto_control_flag == DISABLE)				//键盘控制
	{
		
/**************** 旋转 ************************/
		
		if ( RC_Ctl.key.v == KEY_Q )	//Q 键
		{
			rock_direction = -chassis_max_rotate_speed;
		}
		else
			if ( RC_Ctl.key.v == KEY_E )//E 键
		{
			rock_direction = chassis_max_rotate_speed;
		}
		else
		{
			rock_direction = ( float )( RC_Ctl.mouse.x * 200 );
		}
		
		switch ( RC_Ctl.key.v )
		{
			case KEY_NONE :
				speed_x = 0;
				speed_y = 0;
			break;
			
			case KEY_W : 
				speed_x = 0;
				speed_y = chassis_max_speed;
			break;
			
			case KEY_V : 
				speed_x = 0;
				speed_y = chassis_normal_low_speed;
			break;
			
			case KEY_S :
				speed_x = 0;
				speed_y = -chassis_slow_speed;
			break;
			
			case KEY_D :
				speed_x = chassis_max_speed;
				speed_y = 0;
			break;
			
			case KEY_A :
				speed_x = -chassis_max_speed;
				speed_y = 0;
			break;
			
			case KEY_WD :
				speed_x = chassis_max_speed;
				speed_y = chassis_max_speed;
			break;
			
			case KEY_SD :
				speed_x = chassis_max_speed;
				speed_y = -chassis_max_speed;
			break;
			
			case KEY_SA :
				speed_x = -chassis_max_speed;
				speed_y = -chassis_max_speed;
			break;
			
			case KEY_WA :
				speed_x = -chassis_max_speed;
				speed_y = chassis_max_speed;
			break;
			
			case KEY_WQ : 
				speed_x = 0;
				speed_y = chassis_max_speed;
				rock_direction = -chassis_max_rotate_speed;
			break;
			
			case KEY_WE : 
				speed_x = 0;
				speed_y = chassis_max_speed;
				rock_direction = chassis_max_rotate_speed;
			break;
			
			default:
				break;
			
		}
		
		if ( RC_Ctl.key.v == KEY_W && flag_KEY_W == 0 )
		{
			
			flag_KEY_W = 1;
			
		}
		if ( flag_KEY_W == 1 || flag_KEY_W == 2 )
		{
			
			if ( RC_Ctl.key.v == KEY_W )
			{
				
				KEY_W_time ++;
				
				if ( KEY_W_time < 100 )
				{
					
					flag_KEY_W = 2;
					
				}
				
				if ( KEY_W_time > 100 )
				{
					
					flag_KEY_W = 0;
					KEY_W_time = 0;
					
				}
			}
				
		}
		if ( flag_KEY_W == 2 || flag_KEY_W == 3 )
		{
			
			if ( RC_Ctl.key.v == KEY_NONE )
			{
				
				KEY_W_time ++;
				
				if ( KEY_W_time < 200 )
				{
					
					flag_KEY_W = 3;
					
				}
				if ( KEY_W_time > 200 )
				{
					
					flag_KEY_W = 0;
					KEY_W_time = 0;
					
				}
				
			}
				
		}
		if ( flag_KEY_W == 3 && RC_Ctl.key.v == KEY_W )
		{
			
			flag_KEY_W = 4;
			KEY_W_time = 0;
			
		}
			
/**************** 恢复高速 *********************/
		
		if ( RC_Ctl.key.v == KEY_C || RC_Ctl.key.v == KEY_WC )
		{
			
			flag_KEY_W = 0;
			
		}


/**************** 旋转180 ********************/
		
		if ( RC_Ctl.key.v == KEY_SHIFT_Q && flag_rotate_180 == 0 )
		{
			
			flag_rotate_180 = 1;
			
		}
		if ( flag_rotate_180 == 1 )
		{
			
			flag_rotate_1ms ++;
			rock_direction = -chassis_max_rotate_speed;
			if ( flag_rotate_1ms > rotate_180_time )
			{
				
				rock_direction = 0;
				flag_rotate_1ms = 0;
				flag_rotate_180 = 0;
				
			}
			
		}
		
		if ( take == 2 )
		{
			
			rock_direction = -rock_direction;
			
		}
		
	}
	
	if ( RC_Ctl.rc.s1 == 3 )
	{
		
		speed_x = ( float ) ( ( RC_Ctl.rc.ch2 - 1024 ) * 5 );
		speed_y = ( float ) ( ( RC_Ctl.rc.ch3 - 1024 ) * 5 );
		rock_direction = ( float ) ( ( RC_Ctl.rc.ch0 - 1024 ) * 5 );
		
	}

		if ( back == 1 )
		{
			
			speed_x = -speed_x;
			speed_y = -speed_y;
			
		}
		
		
		if ( flag_KEY_W == 4 )
	{
		
		speed_x = limit ( speed_x, chassis_slow_speed, -chassis_slow_speed );
		speed_y = limit ( speed_y, chassis_slow_speed, -chassis_slow_speed );
		rock_direction = limit ( rock_direction, chassis_slow_rotate_speed, -chassis_slow_rotate_speed );
		
	}
	
	if ( pid_up_down.Sv[0] >= impulse_mm( box_height ) || back == 1 || pid_up_down.Sv[0] == impulse_mm( island_pillar ) )
	{
		
		speed_x = limit ( speed_x, chassis_low_speed, -chassis_low_speed );
		speed_y = limit ( speed_y, chassis_low_speed, -chassis_low_speed );
		rock_direction = limit ( rock_direction, chassis_low_rotate_speed, -chassis_low_rotate_speed );
		
	}
	
	if ( flag_rotate_180 == 1 )
	{
		
		rock_direction = -4000;
		
	}
	
	if ( RC_Ctl.key.v == KEY_F || RC_Ctl.key.v == KEY_R || RC_Ctl.key.v == KEY_SHIFT_S )
	{
		
		auto_move_flag = 0;
		move_r_l_flag = 0;
		flag_take = 0;
		
	}
		
/***************** 取弹自动对准 ****************/
	
	if ( flag_take == 1 )		//开启自动取弹
	{
		auto_control_flag = ENABLE;
	
		if ( RC_Ctl.key.v == KEY_A )
		{
			
			move_r_l_flag = 1;
			
		}
		else if ( RC_Ctl.key.v == KEY_D )
		{
			
			move_r_l_flag = 2;
			
		}
		
		if ( auto_move_flag == 0 && move_r_l_flag == 1 )
		{
			if ( photoswitch1 != 1 || photoswitch2 != 1 )
			{
				
				speed_x = -auto_move_x;
				speed_y = 70;
				rock_direction = 0;
				auto_move_flag = 1;
				flag_brake_r_l = 1;
				
			}
			
		}
		else if ( auto_move_flag == 0 && move_r_l_flag == 2 )
		{
			
			if ( photoswitch1 != 1 || photoswitch2 != 1 )
			{
				
				speed_x = auto_move_x;
				speed_y = 70;
				rock_direction = 0;
				auto_move_flag = 1;
				flag_brake_r_l = 2;
				
			}
			
			
		}
		if ( auto_move_flag == 0 && photoswitch1 == 1 && photoswitch2 == 1 )
		{
			
			auto_move_flag = 2;
			take_flag = 2;
			
		}
		if ( auto_move_flag == 1 && photoswitch1 == 1 && photoswitch2 == 1 )
		{
			
			speed_x = 0;
			speed_y = 0;
			rock_direction = 0;
			auto_move_flag = 2;
			take_flag = 2;
			
		}
		
		if ( take == 1 && auto_move_flag == 2 && move_r_l_flag != 0 )
		{
			
				if ( move_r_l_flag == 2 )
				{
					
					speed_x = auto_move_x;
					flag_brake_r_l = 2;
					
				}
				else if ( move_r_l_flag == 1 )
				{
					
					speed_x = -auto_move_x;
					flag_brake_r_l = 1;
					
				}
				
				speed_y = 70;
				rock_direction = 0;
				take = 0;
				take_flag = 1;
			
		}
		
		if ( take_flag == 1 && photoswitch1 == 0 && photoswitch2 == 0 )
		{
			
			take = 0;
			take_flag = 2;
			
		}
		
		if ( take == 0 && photoswitch1 == 1 && photoswitch2 == 1 && take_flag == 2 )
		{
			
			flag_brake = 1;
		}
			
			
			
		if ( flag_brake == 1 )
		{
			
				brake_1ms ++;
			if ( flag_brake_r_l == 1 )
			{
				
				speed_x = auto_move_x;
				
			}
			else if ( flag_brake_r_l == 2 )
			{
				
				speed_x = -auto_move_x;
				
			}
			
			if ( brake_1ms > 150 )
			{
				
				speed_x = 0;
				speed_y = 0;
				rock_direction = 0;
				CAN2_Send_Msg( 0, 0, 2 );
				take = 0;
				take_flag = 0;
				brake_1ms = 0;
				flag_brake = 0;
				flag_brake_r_l = 0;
			
			}
		
//		printf ("flag_brake:%d\r\n",flag_brake);
		}
	}
	else 
	{
		
		auto_control_flag = DISABLE;  /* 退出自动模式 */
		
	}
	
		chassis_alg();  //底盘运动函数

		if ( auto_control_flag == DISABLE )
		{
			
			speed_transition[0] = speed_change_limit( speed_transition[0], left_front, 15, 35 );	//加速度
			speed_transition[1] = speed_change_limit( speed_transition[1], right_front, 15, 35 );
			speed_transition[2] = speed_change_limit( speed_transition[2], left_back, 15, 35 );
			speed_transition[3] = speed_change_limit( speed_transition[3], right_back, 15, 35 );
			
		}
		
		else if ( auto_control_flag == ENABLE )
		{
			
			speed_transition[0] = left_front;	//加速度
			speed_transition[1] = right_front;
			speed_transition[2] = left_back;
			speed_transition[3] = right_back;
			
		}
		
		
		
			pid_200.Sv[0] = speed_transition[0];
			pid_200.Sv[1] = speed_transition[1];
			pid_200.Sv[2] = speed_transition[2];
			pid_200.Sv[3] = speed_transition[3];
		
		
		pid_200.Pv[0] = motor_speed[0];
		pid_200.Pv[1] = motor_speed[1];
		pid_200.Pv[2] = motor_speed[2];
		pid_200.Pv[3] = motor_speed[3];
		
		PID_count_201 ();
		PID_count_202 ();
		PID_count_203 ();
		PID_count_204 ();

		CAN1_Send_Chassis_Msg( pid_200.out[0], pid_200.out[1], pid_200.out[2], pid_200.out[3] );
}

