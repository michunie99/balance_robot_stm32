/*
 * complementary_filter.h
 *
 *  Created on: Jul 19, 2022
 *      Author: michunie
 */

#ifndef INC_COMP_FILTER_H_
#define INC_COMP_FILTER_H_

typedef struct {
	float curr_value;
	float filter_gain;
	float sample_time;
}comp_filter;

// Simple one axis complementary filter
void init_filter( comp_filter* filter, float gain, float sample_time, float initial_value);

float update_filter( comp_filter* filter, float acc, float gyro);

#endif /* INC_COMP_FILTER_H_ */
