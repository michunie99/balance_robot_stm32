/*
 * balance_pid.c
 *
 *  Created on: Jul 27, 2022
 *      Author: michunie
 */
#include "balance_pid.h"

#define abs(x)  (x<0)?-x:x
#define dir(x)	(x<0)?0:1

void init_balance_regulator(balance_pid* cnt,float set_point, float k_p, float k_i, float k_d){

	/* Initalize gains */
	cnt->k_p = k_p;
	cnt->k_i = k_i;
	cnt->k_d = k_d;

	/* Set point */
	cnt->set_point = set_point;

	/* Anti wind up, scale and limits */
	cnt->anti_windup = 1200;

	cnt->motor1_scale = 1;
	cnt->motor2_scale = 1;

	cnt->up_lim = UINT16_MAX;
	cnt->low_lim = -UINT16_MAX;

	/* Initailize errors */
	cnt->total_error = 0;
	cnt->prev_error = 0;
}

void update_controler(balance_pid* cnt, float measured){

	float error = cnt->set_point-measured;

	cnt->total_error += error;

	/* Check anti windup */
	if(cnt->total_error > cnt->anti_windup)
		cnt->total_error = cnt->anti_windup;
	if(cnt->total_error < -cnt->anti_windup)
			cnt->total_error = -cnt->anti_windup;


	/* Calculate control input for both wheels */
	float control = 	error*cnt->k_p +
						cnt->total_error*cnt->k_i +
						(cnt->prev_error-error)*cnt->k_d;

	/* Set apropriate PWM values for both registers in controler */

	/* Motor 1 */
	float control1 = control * cnt->motor1_scale;

	if(control1 >= cnt->up_lim)
		control1 = cnt->up_lim;
	if(control1 <= cnt->low_lim)
		control1 = cnt->low_lim;

	max14870_set_speed(&cnt->cntr_1, abs(control1));
	max1470_set_direction(&cnt->cntr_1, dir(control1));

	/* Motor 2 */
	float control2 = control * cnt->motor2_scale;

	if(control2 >= cnt->up_lim)
		control2 = cnt->up_lim;
	if(control2 <= cnt->low_lim)
		control2 = cnt->low_lim;

	max14870_set_speed(&cnt->cntr_2, abs(control2));
	max1470_set_direction(&cnt->cntr_2, dir(control2));

	/* Save previous error */
	cnt->prev_error = error;
}

void change_set_point(balance_pid* cnt, float set_point){
	cnt->set_point = set_point;
}

void change_limits(balance_pid* cnt, float upper, float lower){
	if(upper > lower){
		cnt->up_lim = upper;
		cnt->low_lim = lower;
	}
}

void change_scales(balance_pid* cnt, float motor1_scale, float motor2_scale){
	cnt->motor1_scale = motor1_scale;
	cnt->motor2_scale = motor2_scale;
}

void reset_regulator(balance_pid* cnt){
	cnt->total_error = 0;
	cnt->prev_error = 0;
}
