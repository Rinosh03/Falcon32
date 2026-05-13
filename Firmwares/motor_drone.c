

#include "motor_drone.h"


void set_motor_voltage(MotorDroneHandler *motor, uint8_t motor_number, float percentage)
{
	if(percentage > 100.0)
	{
		percentage = 100.0;
	}
	uint16_t compare_value = ((percentage * motor->timer->Instance->ARR) / 100.0);

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

