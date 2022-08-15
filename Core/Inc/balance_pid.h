/*
 * balance_pid.h
 *
 *  Created on: Jul 27, 2022
 *      Author: michunie
 */

#ifndef INC_BALANCE_PID_H_
#define INC_BALANCE_PID_H_

#include "stm32f4xx_hal.h"
#include "max14870.h"

#define CONTROLER_FREQUENCY 10 // control loop frequency in Hz


typedef struct{
	/* Set point */
	float set_point;

	/* Regulator gains */
	float k_p;
	float k_i;
	float k_d;

	/* Errors storage */
	float total_error;
	float prev_error;

	/*Anti-windup */
	float anti_windup;

	/* Motor controller structures */
	max14870 cntr_1;
	max14870 cntr_2;

	/* Scale factors */
	float motor1_scale;
	float motor2_scale;

	/* Output limits */
	int up_lim;
	int low_lim;

}balance_pid;

/* Initialization function*/
void init_balance_regulator(balance_pid* cnt,float set_point, float k_p, float k_i, float k_d);

/* Calculate output and update PWM values*/
void update_controler(balance_pid* cnt, float measured);

/* General use functions */
void change_set_point(balance_pid* cnt, float set_point);
void change_limits(balance_pid* cnt, float upper, float lower);
void change_scales(balance_pid* cnt, float motor1_scale, float motor2_scale);
void reset_regulator(balance_pid* cnt);



#endif /* INC_BALANCE_PID_H_ */
