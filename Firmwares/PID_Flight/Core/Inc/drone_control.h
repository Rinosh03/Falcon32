/*
 * drone_control.h
 *
 *  Created on: May 5, 2025
 *      Author: yerke
 */

#ifndef INC_DRONE_CONTROL_H_
#define INC_DRONE_CONTROL_H_

#include "motor_drone.h"
#include "pid_control.h"
#include "euler_angles.h"

typedef struct
{
	pid_instance pid_pitch;
	pid_instance pid_roll;
	pid_instance pid_yaw;
	MotorDroneHandler motor_drone;
	euler_angles      angles_estimation;
	uint8_t start_flight;
}DroneTypeDef;

void drone_initialization(DroneTypeDef *drone);
void drone_apply_control(DroneTypeDef *drone, euler_angles reference);
void drone_update_angle(DroneTypeDef *drone, euler_angles angle);
#endif /* INC_DRONE_CONTROL_H_ */
