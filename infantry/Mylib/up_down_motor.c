#include"main.h"
#include "up_down_motor.h"

float now_position1=0,last_position1=0,position1=0,position1_Ek=0;
float flag=0;

void up_motor()
{
	posit1();
	control();
	pid_1_cal(&pid_2,position1,pid_2.SV);
	pid_cal(&pid_3,motor_encoder[3],pid_2.OUT);   //速度环

	CAN1_Send_up_down(pid_3.OUT);
}

void posit1()
{
	last_position1 = now_position1;
	now_position1 = motor_encoder1[2];
	position1_Ek = now_position1-last_position1;
		if(position1_Ek>=5500)//反转过临界
	{
		position1_Ek=position1_Ek-8192;
	}
	else if(position1_Ek<-5500)//正传过临界
	{
		position1_Ek=8192+position1_Ek;
	}

	else
	{
		position1_Ek=position1_Ek;
	}
	if (flag==0)
	{
		position1=0;
	}
	else
	{
		position1=position1+position1_Ek;
	}		
	flag++;
}
void control()
{
	if(RC_Ctl.rc.s1==1&&RC_Ctl.rc.s2==1)//左上、、右上 升降模式
	{
			if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)==0)//触碰检测
		{
			delay_ms(10);
			if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)==0)
			{
				
				pid_2.SV = position1;
				
			
			}
		}
		else //遥控上升
		{
			if(RC_Ctl.rc.ch1-1024.f==1684)
			{
				pid_2.SV=829000;
				
			}
			else
			{
				if(RC_Ctl.rc.ch3-1024.f>0)
				{
					
					pid_2.SV=(RC_Ctl.rc.ch3-1024.f)*(29000/660.f)+position1;
			
				}
				else
				{
					pid_2.SV=(RC_Ctl.rc.ch3-1024.f)*(19000/660.f)+position1;
					
				}
			}
		
	}	
	if(pid_2.Ek==0)
		{
			if(pid_3.OUT>8000){pid_3.OUT=8000;}
			if(pid_3.OUT<-8000){pid_3.OUT=-8000;}
		}
}


