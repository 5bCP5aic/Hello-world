#ifndef __BSP_H__
#define __BSP_H__

#include <stm32f4xx.h>
#include "can1.h"
#include "can2.h"
#include "led.h"
#include "usart3.h"
#include "uart4.h"
#include "usart2.h"
#include "usart3.h"
#include "delay.h"
#include "air_cylinder.h"
#include "pwm.h"
#include "key.h"

#include "tim4.h"
//#include "mpu6050_driver.h"
void BSP_Init(void);

#endif 


