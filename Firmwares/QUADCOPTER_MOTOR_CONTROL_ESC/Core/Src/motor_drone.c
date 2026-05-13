
#include "motor_drone.h"
#define ONE_MS_DUTY  1000


void set_motor_voltage(MotorDroneHandler *motor, uint8_t motor_number, float percentage)
{
	uint16_t compare_value;
	if(percentage > 100.0)
	{
		percentage = 100.0;
	}
	compare_value= ((percentage * ONE_MS_DUTY) / (100.0));
	compare_value += ONE_MS_DUTY; // 1 ms
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

