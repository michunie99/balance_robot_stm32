/*
 * comp_filter.c
 *
 *  Created on: Jul 19, 2022
 *      Author: michunie
 */

#include "comp_filter.h"

void init_filter( comp_filter* filter, float gain, float sample_time, float initial_value){
	filter->curr_value = initial_value;
	filter->filter_gain = gain;
	filter->sample_time = sample_time;
}

float update_filter( comp_filter* filter, float acc, float gyro){
	float gyro_angle = (gyro*filter->sample_time+filter->curr_value) * filter->filter_gain;
	float acc_angle = acc * (1-filter->filter_gain);

	filter->curr_value = gyro_angle + acc_angle;

	return filter->curr_value;
}


