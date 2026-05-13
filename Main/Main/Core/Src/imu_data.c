#include "imu_data.h"
#include "stdio.h"


void sensor2imu_lsm6dsr(lsm6dsr_handler imu_data, imu_norm* imu)
{
	float32_t norm_acc;
	arm_status arm_status_temp;

	arm_status_temp = arm_sqrt_f32((float32_t)imu_data.data.ax * imu_data.data.ax + (float32_t)imu_data.data.ay * imu_data.data.ay
			+ imu_data.data.az * imu_data.data.az, &norm_acc);

	if(arm_status_temp != ARM_MATH_SUCCESS)
	{
		printf("error sqrt! %d \n", arm_status_temp);
		while(1);
	}

	imu->ax = -((float32_t)imu_data.data.ax) / norm_acc;
	imu->ay = ((float32_t)imu_data.data.ay) / norm_acc;
	imu->az = ((float32_t)imu_data.data.az) / norm_acc;

	imu->gx = ((float32_t)imu_data.data.gx) * imu_data.gyro_scale;
	imu->gy = -((float32_t)imu_data.data.gy) * imu_data.gyro_scale;
	imu->gz = -((float32_t)imu_data.data.gz) * imu_data.gyro_scale;

	imu->mx = 0;
	imu->my = 0;
	imu->mz = 0;
}
