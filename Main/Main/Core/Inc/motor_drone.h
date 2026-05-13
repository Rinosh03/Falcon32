/*
 * motor_drone.h
 *
 *  Created on: May 2, 2025
 *      Author: yerke
 */

#ifndef INC_MOTOR_DRONE_H_
#define INC_MOTOR_DRONE_H_

#include "main.h"

#define MIN_VOLTAGE  3.2

typedef struct
{
	TIM_HandleTypeDef *timer;
	uint32_t motor1_ch;
	uint32_t motor2_ch;
	uint32_t motor3_ch;
	uint32_t motor4_ch;
	uint16_t arr_value;
	float vbus_voltage;
}MotorDroneHandler;

typedef enum
{
	MOTOR_1,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
}MOTOR_NUMBER;

void set_motor_voltage(MotorDroneHandler *motor, uint8_t motor_number, float voltage);
void update_bat_voltage(MotorDroneHandler *motor, float bat_voltage);
#endif /* INC_MOTOR_DRONE_H_ */
