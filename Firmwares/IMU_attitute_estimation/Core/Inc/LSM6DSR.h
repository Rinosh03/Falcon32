/*
 * LSM6DSR.h
 *
 *  Created on: May 2, 2025
 *      Author: yerke
 */

#ifndef INC_LSM6DSR_H_
#define INC_LSM6DSR_H_

#include "main.h"

#define LSM6DSR_WHO_AM_I_REG   		0x0f
#define LSM6DSR_CTRL1_XL_REG   		0x10
#define LSM6DSR_CTRL2_G_REG    		0x11
#define LSM6DSR_CTRL3_XL_REG   		0x12
#define LSM6DSR_CTRL8_XL_REG   		0x17
#define LSM6DSR_OUTX_L_G_REG   		0x22
#define LSM6DSR_ALL_INT_SRC_REG 	0x1A
#define LSM6DSR_FREE_FALL_REG 		0x5d
#define LSM6DSR_WAKE_UP_DUR_REG     0x5c
#define LSM6DSR_TAP_CFG2_REG		0x58

#define GYRO_RANGE_VALUE        _gyro_2000dps
#define ACCEL_RANGE_VALUE       _accel_8g

#define LSM6DSR_ID   0x6b

typedef enum
{
	_gyro_250dps =  0x00,
	_gyro_500dps =  0x01,
	_gyro_1000dps = 0x02,
	_gyro_2000dps = 0x03
} gyro_range;

typedef enum
{
	NO_FREE_FALL =  0x00,
	FREE_FALL =  0x01
} FreeFallStatusTypeDef;


typedef enum
{
	_accel_2g =  0x00,
	_accel_16g =  0x01,
	_accel_4g =  0x02,
	_accel_8g = 0x03
} accel_range;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
}imu_data;
typedef struct
{
	SPI_HandleTypeDef 	*spi_imu;
	GPIO_TypeDef 		*gpio_cs_port;
	uint16_t 			gpio_cs_pin;
	imu_data            data;
	int16_t            gx_bias;
	int16_t            gy_bias;
	int16_t            gz_bias;
	float               gyro_scale;
}lsm6dsr_handler;

typedef enum
{
	LSM6DSR_OK,
	LSM6DSR_ERROR,
}LSM6DSR_StateTypeDef;
LSM6DSR_StateTypeDef lsm6dsr_init(lsm6dsr_handler* imu_sensor , uint16_t sample_rate);
void lsm6dsr_read_data(lsm6dsr_handler* imu_sensor);
FreeFallStatusTypeDef lsm6dsr_free_fall(lsm6dsr_handler* imu_sensor);
#endif /* INC_LSM6DSR_H_ */
