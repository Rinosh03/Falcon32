/*
 * drone_configuration.c
 *
 *  Created on: May 5, 2025
 *      Author: yerke
 */


#include "drone_control.h"

extern TIM_HandleTypeDef htim4;
#define SAMLPE_RATE  500
#define MAX_VOLTAGE  4.0
DroneTypeDef drone = {
		.motor_drone = {
			.arr_value = 999,
			.motor1_ch = TIM_CHANNEL_1,
			.motor2_ch = TIM_CHANNEL_2,
			.motor3_ch = TIM_CHANNEL_3,
			.motor4_ch = TIM_CHANNEL_4,
			.timer = &htim4,
						},
		.pid_pitch = {
			.d_gain = 0.15,
			.error_integral = 0,
			.i_gain = 0.8,
			.integral_max = MAX_VOLTAGE * 0.35 * SAMLPE_RATE / 0.7,
			.last_error = 0,
			.output = 0,
			.p_gain = 1.5,
			.pid_max = MAX_VOLTAGE * 0.5,
			.sam_rate = SAMLPE_RATE
		},
		.pid_roll = {
			.d_gain = 0.15,
			.error_integral = 0,
			.i_gain = 0.8,
			.integral_max = MAX_VOLTAGE * 0.35 * SAMLPE_RATE / 0.7,
			.last_error = 0,
			.output = 0,
			.p_gain = 1.5,
			.pid_max = MAX_VOLTAGE * 0.5,
			.sam_rate = SAMLPE_RATE
		},
		.pid_yaw = {
			.d_gain = 0.0,
			.error_integral = 0,
			.i_gain = 0.5,
			.integral_max = MAX_VOLTAGE * 0.25 * SAMLPE_RATE / 0.25,
			.last_error = 0,
			.output = 0,
			.p_gain = 1.0,
			.pid_max = MAX_VOLTAGE * 0.4,
			.sam_rate = SAMLPE_RATE
		},
		.pid_altitude = {
			.d_gain = 0.0,
			.error_integral = 0,
			.i_gain = 0.05,
			.integral_max = MAX_VOLTAGE * 0.4 * SAMLPE_RATE / 0.05,
			.last_error = 0,
			.output = 0,
			.p_gain = 0.4,
			.pid_max = MAX_VOLTAGE * 0.8,
			.sam_rate = SAMLPE_RATE
		},
		.voltage_default = HOVERING_VOLTAGE
};
