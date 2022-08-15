/*
 * max14870.h
 *
 *  Created on: Feb 14, 2022
 *      Author: michu
 */

#ifndef INC_MAX14870_H_
#define INC_MAX14870_H_

#include "stm32f4xx_hal.h"

// pin configuration for motor driver
typedef struct {
	// PWN timer configuration
	TIM_HandleTypeDef* PWN_timer;
	uint16_t PWM_Channel;

	// EN pin configuration
	uint16_t EN_pin;
	GPIO_TypeDef * EN_port;

	// DIR pin configuration
	uint16_t DIR_pin;
	GPIO_TypeDef * DIR_port;
}max14870;


// enumerate for motor direction
typedef enum{
	CW = 0,
	CCW = 1,
}MAX14870_dir;

// enumerate for motor modes
typedef enum{
	SHUTDOWN,
	ON
}MAX14870_mode;

void max14870_init( max14870*, TIM_HandleTypeDef*,uint16_t , uint16_t, GPIO_TypeDef *,
		uint16_t, GPIO_TypeDef *);
void max14870_mode_control( max14870* ,MAX14870_mode);
void max14870_set_speed( max14870*, uint16_t);
void max1470_set_direction( max14870*, MAX14870_dir);


#endif /* INC_MAX14870_H_ */
