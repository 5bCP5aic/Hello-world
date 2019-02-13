#ifndef __AIR_CYLINDER_H__
#define __AIR_CYLINDER_H__

#define		Cylinder1_on		GPIO_SetBits(GPIOB,GPIO_Pin_4)
#define		Cylinder1_off		GPIO_ResetBits(GPIOB,GPIO_Pin_4)
#define		Cylinder2_on 		GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define		Cylinder2_off 		GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define		Cylinder3_on 		GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define		Cylinder3_off 		GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define		Cylinder4_on 		GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define		Cylinder4_off 		GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define		Cylinder5_on 		GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define		Cylinder5_off 		GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define		Cylinder6_on 		GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define		Cylinder6_off 		GPIO_ResetBits(GPIOE,GPIO_Pin_3)
#define		Cylinder7_on 		GPIO_SetBits(GPIOE,GPIO_Pin_4)
#define		Cylinder7_off 		GPIO_ResetBits(GPIOE,GPIO_Pin_4)
#define		Cylinder8_on 		GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define		Cylinder8_off 		GPIO_ResetBits(GPIOE,GPIO_Pin_5)


void air_cylinder_Configuration(void);
	
#endif


