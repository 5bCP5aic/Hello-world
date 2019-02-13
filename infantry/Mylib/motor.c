#include "main.h"
#include "motor.h"

/**
  * @brief  编码器速度
  * @param  void
  * @retval void
  * @note	void
  */
float PTZencoder ( float encoder_1, float encoder )		//转子转一圈有8192个脉冲
{
	float Pv;
	if ( abs( encoder - encoder_1 ) > 5500 )	//过0点判断
	{
		if ( encoder > encoder_1 )				//反转过0点
		{
			Pv = encoder - 8191 - encoder_1;
		}
		else 									//正转过0点
		{
			Pv = 8191 - encoder_1 + encoder;
		}
	}
	else 
	{
		Pv = encoder - encoder_1;
	}
//	if ( Pv > -4 && Pv < 4 )
//	{
//		Pv = 0;
//	}
	return Pv;
}



/**
  * @brief  加速度改变量
  * @param  void
  * @retval void
  * @note	void
  */
float speed_change_limit(float now_speed, float tar_speed, float acc, float dec)
{
	float return_speed = 0;
	
	if ((now_speed > 0) || (now_speed == 0 && tar_speed >= 0))
	{
		return_speed = change_limit(now_speed, tar_speed, acc, dec);
		if (return_speed < 0) return_speed = 0;
	}
	else
	{
		return_speed = change_limit(now_speed, tar_speed, dec, acc);
		if (return_speed > 0) return_speed = 0;
	}
	return (return_speed);
}

/**
  * @brief  加速度
  * @param  void
  * @retval void
  * @note	void
  */

float change_limit(float now_speed, float tar_speed, float acc, float dec)
{
	
	if ((now_speed < tar_speed) && (tar_speed - now_speed) > acc)
	{
		return (now_speed + acc);
	}
	else if ((now_speed > tar_speed) && (now_speed - tar_speed) > dec)
	{
		return (now_speed - dec);
	}
	else 
	{
		return (tar_speed);
	}
}

/**
  * @brief  限幅
  * @param  void
  * @retval void
  * @note	void
  */
int32_t limit ( int32_t limit, int32_t max, int32_t min )
{
	if ( limit > max )
	{
		return max;
	}
	else
		if ( limit < min )
		{
			return min;
		}
	else
	{
		return limit;
	}
		
}
/**
  * @brief  电机堵转
  * @param  void
  * @retval void
  * @note	void
  */
int locked_rotor_1ms = 0;
float Sv_1 = 0;

float locked_rotor ( float speed, float pid_out, float Sv)
{
	if ( speed == 0 && abs( pid_out ) > 8000 )
	{
		locked_rotor_1ms ++;
	}
	if ( locked_rotor_1ms >= 200 )
	{
		locked_rotor_1ms = 200;
		pid_out = limit ( pid_out, 8000, -8000 );
	}
	if ( abs( pid_out ) < 8000 && locked_rotor_1ms != 0 )
	{
		locked_rotor_1ms = 0;
	}
	if ( Sv_1 != Sv )
	{
		locked_rotor_1ms = 0;
	}
	Sv_1 = Sv;
	return pid_out;
}

	
/**
  * @brief  电机运行
  * @param  void
  * @retval void
  * @note	void
  */
void motor_perform ( void )
{
	trailer_motor();
	up_down_motor();
	CAN1_Send_flex_Msg( pid_up_down.out[1], pid_trailer.out[1], 0, 0 );//
	chassis_motor();
}
