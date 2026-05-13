#include "drone_control.h"


float moto1_copy;
void drone_initialization(DroneTypeDef *drone)
{
	reset_pid(&drone->pid_pitch);
	reset_pid(&drone->pid_roll);
	reset_pid(&drone->pid_yaw);
}

void drone_update_angle(DroneTypeDef *drone, euler_angles angle)
{
	drone->angles_estimation = angle;
}

void drone_set_zero(DroneTypeDef *drone)
{
	set_motor_voltage(&drone->motor_drone, MOTOR_1, 0);
	set_motor_voltage(&drone->motor_drone, MOTOR_2, 0);
	set_motor_voltage(&drone->motor_drone, MOTOR_3, 0);
	set_motor_voltage(&drone->motor_drone, MOTOR_4, 0);
}

void drone_apply_control(DroneTypeDef *drone, euler_angles reference)
{
	float yaw_error;
	if(drone->angles_estimation.pitch > 0.8 || drone->angles_estimation.pitch < -0.8 ||
			drone->angles_estimation.roll > 0.8 || drone->angles_estimation.roll < -0.8)
	{
		set_motor_voltage(&drone->motor_drone, MOTOR_1, 0);
		set_motor_voltage(&drone->motor_drone, MOTOR_2, 0);
		set_motor_voltage(&drone->motor_drone, MOTOR_3, 0);
		set_motor_voltage(&drone->motor_drone, MOTOR_4, 0);
		drone->start_flight = 0;

	}
	else
	{
		yaw_error = reference.yaw - drone->angles_estimation.yaw;
		if(yaw_error > PI)
		{
			yaw_error -= 2 * PI;
		}
		if(yaw_error < -PI)
		{
			yaw_error += 2 * PI;
		}
	apply_pid(&drone->pid_pitch, reference.pitch - drone->angles_estimation.pitch);
	apply_pid(&drone->pid_roll, reference.roll - drone->angles_estimation.roll);
	apply_pid(&drone->pid_yaw, reference.yaw - drone->angles_estimation.yaw);

	float motor1_command = drone->voltage_default;
	float motor2_command = drone->voltage_default;
	float motor3_command = drone->voltage_default;
	float motor4_command = drone->voltage_default;


	 motor1_command += drone->pid_pitch.output ;
	 motor2_command += drone->pid_pitch.output ;
	 motor3_command -= drone->pid_pitch.output ;
	 motor4_command -= drone->pid_pitch.output ;

	motor1_command += drone->pid_roll.output;
	motor2_command -= drone->pid_roll.output;
	motor3_command -= drone->pid_roll.output;
	motor4_command += drone->pid_roll.output;

	motor1_command += drone->pid_yaw.output;
	motor2_command -= drone->pid_yaw.output;
	motor3_command += drone->pid_yaw.output;
	motor4_command -= drone->pid_yaw.output;
		moto1_copy = motor1_command - HOVERING_VOLTAGE;
	if(motor1_command < 0)
	{
		motor1_command = 0;
	}
	if(motor2_command < 0)
		{
			motor2_command = 0;
		}
	if(motor3_command < 0)
		{
			motor3_command = 0;
		}
	if(motor4_command < 0)
		{
			motor4_command = 0;
		}

	set_motor_voltage(&drone->motor_drone, MOTOR_1, motor1_command);
	set_motor_voltage(&drone->motor_drone, MOTOR_2, motor2_command);
	set_motor_voltage(&drone->motor_drone, MOTOR_3, motor3_command);
	set_motor_voltage(&drone->motor_drone, MOTOR_4, motor4_command);
	}
}

