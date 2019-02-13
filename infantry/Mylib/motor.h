#ifndef _motor_
#define _motor_

#include "main.h"
#include "up_down.h"
#include "chassis.h"
#include "trailer.h"

void motor_perform ( void );
float PTZencoder ( float encoder_1, float encoder );
int32_t limit ( int32_t limit, int32_t max, int32_t min );
float change_limit(float now_speed, float tar_speed, float acc, float dec);
float speed_change_limit(float now_speed, float tar_speed, float acc, float dec);

#endif
