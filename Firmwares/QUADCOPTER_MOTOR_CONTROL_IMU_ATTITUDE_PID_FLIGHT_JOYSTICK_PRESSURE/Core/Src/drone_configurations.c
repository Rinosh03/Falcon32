/*
 * drone_configuration.c
 *
 *  Created on: May 5, 2025
 *      Author: yerke
 */


#include "drone_control.h"

extern TIM_HandleTypeDef htim4;
#define SAMLPE_RATE  500

DroneTypeDef drone = {
		.motor_drone = {
			.motor1_ch = TIM_CHANNEL_1,
			.motor2_ch = TIM_CHANNEL_2,
			.motor3_ch = TIM_CHANNEL_3,
			.motor4_ch = TIM_CHANNEL_4,
			.timer = &htim4,
						},
		.pid_pitch = {
			.d_gain = 10,
			.error_integral = 0,
			.i_gain = 10,
			.integral_max = 60 * SAMLPE_RATE / 10,
			.last_error = 0,
			.output = 0,
			.p_gain = 35,
			.pid_max = 90,
			.sam_rate = SAMLPE_RATE
		},
		.pid_roll = {
			.d_gain = 10,
			.error_integral = 0,
			.i_gain = 10,
			.integral_max = 60 * SAMLPE_RATE / 10,
			.last_error = 0,
			.output = 0,
			.p_gain = 35,
			.pid_max = 90,
			.sam_rate = SAMLPE_RATE
		},
		.pid_yaw = {
			.d_gain = 3,
			.error_integral = 0,
			.i_gain = 6,
			.integral_max = 40 * SAMLPE_RATE / 6,
			.last_error = 0,
			.output = 0,
			.p_gain = 20,
			.pid_max = 50,
			.sam_rate = SAMLPE_RATE
		},
		.pid_altitude = {
			.d_gain = 0.0,
			.error_integral = 0,
			.i_gain = 1,
			.integral_max = 40 * SAMLPE_RATE / 1,
			.last_error = 0,
			.output = 0,
			.p_gain = 10,
			.pid_max = 80,
			.sam_rate = SAMLPE_RATE
		},

};
