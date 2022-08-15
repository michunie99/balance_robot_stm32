/*
 * max14870.c
 *
 *  Created on: Feb 14, 2022
 *      Author: michu
 */

#include "max14870.h"

void max14870_init(max14870* max14870, TIM_HandleTypeDef* timer, uint16_t channel,
		uint16_t EN_pin, GPIO_TypeDef * EN_port,uint16_t DIR_pin, GPIO_TypeDef * DIR_port){

	max14870->PWN_timer = timer;
	max14870->PWM_Channel = channel;

	max14870->EN_pin = EN_pin;
	max14870->EN_port = EN_port;

	max14870->DIR_pin = DIR_pin;
	max14870->DIR_port = DIR_port;

	max14870_mode_control(max14870,ON);
	max14870_set_speed(max14870,0);
	max1470_set_direction(max14870,CW);

	HAL_TIM_PWM_Start_IT(max14870->PWN_timer,
			max14870->PWM_Channel);
}

void max14870_mode_control(max14870* max14870,MAX14870_mode mode){
	switch(mode){
	case SHUTDOWN:
		HAL_GPIO_WritePin(max14870->EN_port,
						max14870->EN_pin, SET);
		break;

	case ON:
		HAL_GPIO_WritePin(max14870->EN_port,
								max14870->EN_pin, RESET);
		break;
	}
}

void max14870_set_speed(max14870* max14870, uint16_t speed){
	if(speed >= max14870->PWN_timer->Instance->ARR)
		speed = max14870->PWN_timer->Instance->ARR;

	__HAL_TIM_SetCompare(max14870->PWN_timer,
			max14870->PWM_Channel,speed);

}

void max1470_set_direction(max14870* max14870, MAX14870_dir dir){
	switch(dir){
	case CW:
		HAL_GPIO_WritePin(max14870->DIR_port,
						max14870->DIR_pin, SET);
		break;

	case CCW:
		HAL_GPIO_WritePin(max14870->DIR_port,
								max14870->DIR_pin, RESET);
		break;
	}
}
