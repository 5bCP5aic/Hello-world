#include "main.h"
#include "bsp.h"
#include "line_patrol.h"
#include "up_down.h"
#include "chassis.h"
#include "trailer.h"

int16_t intial_encoder[6]= {0};
extern int timer2_flag;
extern int usart4_flag;
extern unsigned char sbus_rx_buffer[18];
int main(void)
{   
	delay_ms(4000);
	BSP_Init();
	up_down_pidinit();
	IncPIDInit_200();
	telecontrol_Init();
	trailer_pidinit();
	printf("TITR - RM\r\n");
	
	intial_encoder[0] = motor_encoder[0];
	intial_encoder[1] = motor_encoder[1];
	intial_encoder[2] = motor_encoder[2];
	intial_encoder[3] = motor_encoder[3];
	intial_encoder[4] = upper_encoder;
	intial_encoder[5] = trailer_encoder;
	
	printf ("intial_encoder[0]:%d  intial_encoder[1]:%d  intial_encoder[2]:%d  intial_encoder[3]:%d  intial_encoder[4]:%d  intial_encoder[5]:%d \r\n",
	intial_encoder[0],
	intial_encoder[1],
	intial_encoder[2],
	intial_encoder[3],
	intial_encoder[4],
	intial_encoder[5]);
	TIM4_Configuration();

  while( 1 )
	{		
//		printf("%d      %d\r\n",timer2_flag,   usart4_flag);
//		printf("pid_up_down.location[0]:%f  upper_encoder:%d\r\n",pid_up_down.location[0],upper_encoder);
//		printf("Sv0:%.2f  Pv0:%.2f  Ek0:%.2f  out0:%.2f  Kp0:%.2f   Ki0:%.2f  Sv[1]:%.2f  Pv[1]:%.2f  Ek[1]:%.2f  out[1]:%.2f  Kp1:%.2f   Ki:%.2f  \r\n",pid_up_down.Sv[0],pid_up_down.Pv[0],pid_up_down.Ek[0],pid_up_down.out[0],pid_up_down.Kp[0],pid_up_down.Ki[0],pid_up_down.Sv[1],pid_up_down.Pv[1],pid_up_down.Ek[1],pid_up_down.out[1],pid_up_down.Kp[1],pid_up_down.Ki[1]);
//		printf("Sv0:%.2f  Pv0:%.2f  Ek0:%.2f  out0:%.2f  Kp0:%.2f   Ki0:%.2f\r\nSv1:%.2f  Pv1:%.2f  Ek1:%.2f  out1:%.2f  Kp1:%.2f   Ki1:%.2f\r\nSv2:%.2f  Pv2:%.2f  Ek2:%.2f  out2:%.2f  Kp2:%.2f   Ki2:%.2f\r\nSv3:%.2f  Pv3:%.2f  Ek3:%.2f  out3:%.2f  Kp3:%.2f   Ki3:%.2f\r\n\r\n\r\n"
//				,pid_200.Sv[0],pid_200.Pv[0],pid_200.Ek[0],pid_200.out[0],pid_200.A[0],pid_200.B[0]
//				,pid_200.Sv[1],pid_200.Pv[1],pid_200.Ek[1],pid_200.out[1],pid_200.A[1],pid_200.B[1]
//				,pid_200.Sv[2],pid_200.Pv[2],pid_200.Ek[2],pid_200.out[2],pid_200.A[2],pid_200.B[2]
//				,pid_200.Sv[3],pid_200.Pv[3],pid_200.Ek[3],pid_200.out[3],pid_200.A[3],pid_200.B[3]);
		
//		printf("Sv0:%.2f  Pv0:%.2f  Ek0:%.2f  out0:%.2f  Kp0:%.2f   Ki0:%.2f\r\n",pid_200.Sv[3],pid_200.Pv[3],pid_200.Ek[3],pid_200.out[3],pid_200.A[3],pid_200.B[3]);
//		printf ("RC_Ctl.rc.ch0:%d\r\n",RC_Ctl.rc.ch0);
//		printf("trailer_encoder:%d  trailer_speed:%d  Pv0:%.2f\r\n",trailer_encoder,trailer_speed,pid_trailer.Pv[0]);
//		printf(" 2:%d   3:%d \r\n",GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5),GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7));
//		printf("Sv0:%.2f  Pv0:%.2f  Ek0:%.2f  out0:%.2f  Kp0:%.2f   Ki0:%.2f  Sv[1]:%.2f  Pv[1]:%.2f  Ek[1]:%.2f  out[1]:%.2f  Kp1:%.2f   Ki:%.2f  location[0]:%f\r\n",
//		pid_trailer.Sv[0],
//		pid_trailer.Pv[0],
//		pid_trailer.Ek[0],
//		pid_trailer.out[0],
//		pid_trailer.Kp[0],
//		pid_trailer.Ki[0],
//		pid_trailer.Sv[1],
//		pid_trailer.Pv[1],
//		pid_trailer.Ek[1],
//		pid_trailer.out[1],
//		pid_trailer.Kp[1],
//		pid_trailer.Ki[1],
//		pid_trailer.location[0]);
//		printf("Sv0:%.2f  Pv0:%.2f  Ek0:%.2f  out0:%.2f  Kp0:%.2f   Ki0:%.2f  Sv[1]:%.2f  Pv[1]:%.2f  Ek[1]:%.2f  out[1]:%.2f  Kp1:%.2f   Ki:%.2f\r\n",pid_line_patrol_x.Sv,pid_line_patrol_x.Pv,pid_line_patrol_x.Ek,pid_line_patrol_x.out,pid_line_patrol_x.Kp,pid_line_patrol_x.Ki,pid_line_patrol_rotate.Sv,pid_line_patrol_rotate.Pv,pid_line_patrol_rotate.Ek,pid_line_patrol_rotate.out,pid_line_patrol_rotate.Kp,pid_line_patrol_rotate.Ki);
//printf("ch0£º%d  ch1£º%d  ch2£º%d  ch3£º%d  s1:%d  s2:%d  x:%d  y:%d  z:%d  l:%d  r:%d  v:%d  \r\n",
//		RC_Ctl.rc.ch0,
//		RC_Ctl.rc.ch1,
//		RC_Ctl.rc.ch2,
//		RC_Ctl.rc.ch3,
//		RC_Ctl.rc.s1,
//		RC_Ctl.rc.s2,
//		RC_Ctl.mouse.x,
//		RC_Ctl.mouse.y,
//		RC_Ctl.mouse.z,
//		RC_Ctl.mouse.press_l,
//		RC_Ctl.mouse.press_r,
//		RC_Ctl.key.v);
//		printf("photoswitch1:%d  photoswitch2:%d \r\n",photoswitch1,photoswitch2);
//		printf("flag_upper_prop:%d\r\n",flag_upper_prop);
//		printf ("E_horizontal:%d  E_angular:%d \r\n",E_horizontal,E_angular);
//		printf("speed_y:%f\r\n",speed_y);
//		printf ("upper_speed:%d  upper_encoder:%d\r\n",upper_speed,upper_encoder);
//printf("[0]:%d  [1]:%d  [2]:%d  [3]:%d  [4]:%d  [5]:%d  6:%d  7:%d  8:%d  9:%d  10:%d  11:%d  12:%d  13:%d  14:%d  15:%d  16:%d 17:%d\r\n",
//sbus_rx_buffer[0],
//sbus_rx_buffer[1],
//sbus_rx_buffer[2],
//sbus_rx_buffer[3],
//sbus_rx_buffer[4],
//sbus_rx_buffer[5],
//sbus_rx_buffer[6],
//sbus_rx_buffer[7],
//sbus_rx_buffer[8],
//sbus_rx_buffer[9],
//sbus_rx_buffer[10],
//sbus_rx_buffer[11],
//sbus_rx_buffer[12],
//sbus_rx_buffer[13],
//sbus_rx_buffer[14],
//sbus_rx_buffer[15],
//sbus_rx_buffer[16],
//sbus_rx_buffer[17]
//);
	}
}
