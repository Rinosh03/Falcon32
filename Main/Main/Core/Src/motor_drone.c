/*
 * motor_drone.c
 *
 *  Created on: May 2, 2025
 *      Author: yerke
 */

#include "motor_drone.h"

#define IIR_FILTER 		0.95
#define ADC_2_VOLTAGE   (3 * 3.0) / (4095 * 2)

void set_motor_voltage(MotorDroneHandler *motor, uint8_t motor_number, float voltage)
{
	if(motor->vbus_voltage < voltage)
	{
		voltage = motor->vbus_voltage;
	}
	uint16_t compare_value = ((voltage * motor->arr_value) / 4.0);
	if(motor->vbus_voltage < MIN_VOLTAGE)
	{
		__HAL_TIM_SET_COMPARE(motor->timer, motor->motor1_ch, 0);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->motor2_ch, 0);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->motor3_ch, 0);
		__HAL_TIM_SET_COMPARE(motor->timer, motor->motor4_ch, 0);

	}
	else
	{
		switch(motor_number)
		{
		case MOTOR_1:
			__HAL_TIM_SET_COMPARE(motor->timer, motor->motor1_ch, compare_value);
		case MOTOR_2:
				__HAL_TIM_SET_COMPARE(motor->timer, motor->motor2_ch, compare_value);
		case MOTOR_3:
				__HAL_TIM_SET_COMPARE(motor->timer, motor->motor3_ch, compare_value);
		case MOTOR_4:
				__HAL_TIM_SET_COMPARE(motor->timer, motor->motor4_ch, compare_value);
		}

	}

}

void update_bat_voltage(MotorDroneHandler *motor, float bat_voltage)
{
	static uint8_t first_time = 1;
	if(first_time)
	{
		motor->vbus_voltage = bat_voltage * ADC_2_VOLTAGE;
		first_time = 0;
	}
	else
	{
		motor->vbus_voltage = motor->vbus_voltage * IIR_FILTER +
				bat_voltage * ADC_2_VOLTAGE * (1 - IIR_FILTER);

	}

}
