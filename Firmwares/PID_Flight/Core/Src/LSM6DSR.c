#include "LSM6DSR.h"
#include "math.h"

static void activate_imu(lsm6dsr_handler* imu_sensor)
{
	HAL_GPIO_WritePin(imu_sensor->gpio_cs_port, imu_sensor->gpio_cs_pin,
			GPIO_PIN_RESET);
}
static void deactivate_imu(lsm6dsr_handler* imu_sensor)
{
	HAL_GPIO_WritePin(imu_sensor->gpio_cs_port, imu_sensor->gpio_cs_pin,
				GPIO_PIN_SET);
}
static void lsm6dsr_read_reg(lsm6dsr_handler* imu_sensor, uint8_t address, uint8_t *data, uint8_t size)
{
	uint8_t temp_data = 0x80|address;
	activate_imu(imu_sensor);
	HAL_SPI_Transmit(imu_sensor->spi_imu, &temp_data , 1, 100);
	HAL_SPI_Receive(imu_sensor->spi_imu, data, size, 100);
	deactivate_imu(imu_sensor);
}

static void lsm6dsr_write_reg(lsm6dsr_handler* imu_sensor, uint8_t reg, uint8_t data)
{
	activate_imu(imu_sensor);
	HAL_SPI_Transmit(imu_sensor->spi_imu, &reg, 1, 100);
	HAL_SPI_Transmit(imu_sensor->spi_imu, &data, 1, 100);
	deactivate_imu(imu_sensor);
}

LSM6DSR_StateTypeDef lsm6dsr_init(lsm6dsr_handler* imu_sensor, uint16_t sample_rate)
{
	uint8_t temp_data;
	deactivate_imu(imu_sensor);
	HAL_Delay(100);
	// software reset
	temp_data = 0x01;
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_CTRL3_XL_REG, temp_data);
	HAL_Delay(100);

	// read who am i register
	lsm6dsr_read_reg(imu_sensor, LSM6DSR_WHO_AM_I_REG, &temp_data, 1);
	if(temp_data != LSM6DSR_ID)
	{
		return LSM6DSR_ERROR;
	}

	// block data update and automatic register increment
	temp_data = 0x44;
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_CTRL3_XL_REG, temp_data);

	// exit gyroscope power down mode and set the baud rate (833 hz, 1000 dps)
	temp_data = (0x07 << 4)|(GYRO_RANGE_VALUE << 2);
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_CTRL2_G_REG, temp_data);

	// exit accelerometer power down mode and set the baud rate(833 Hz, 4g range)
	// enable low pass filter
	temp_data = (0x07 << 4)|(ACCEL_RANGE_VALUE << 2|0x02);
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_CTRL1_XL_REG, temp_data);

	temp_data = (0x02 << 5);
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_CTRL8_XL_REG, temp_data);

	imu_sensor ->gyro_scale = (0.0001331621 * pow(2, GYRO_RANGE_VALUE))/ (sample_rate);

	// free-fall interrupt enable
	// duration 8 odr, 156 mg threshold
	temp_data = (0x08 << 3)|(0x00);
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_FREE_FALL_REG, temp_data);

	// enable basic interrupts
	temp_data = 0x80;
	lsm6dsr_write_reg(imu_sensor, LSM6DSR_TAP_CFG2_REG, temp_data);

	// gyro bias estimation
	int32_t temp_bias_x = 0, temp_bias_y = 0, temp_bias_z = 0;
	lsm6dsr_read_data(imu_sensor);
	HAL_Delay(30);
	lsm6dsr_read_data(imu_sensor);
	HAL_Delay(30);
	imu_sensor->gx_bias = 0;
	imu_sensor->gy_bias = 0;
	imu_sensor->gz_bias = 0;

	for(int i = 0 ; i < 100; i++)
	{
		lsm6dsr_read_data(imu_sensor);
		temp_bias_x += imu_sensor->data.gx;
		temp_bias_y += imu_sensor->data.gy;
		temp_bias_z += imu_sensor->data.gz;
		HAL_Delay(3);
	}
	temp_bias_x /= 100;
	temp_bias_y /= 100;
	temp_bias_z /= 100;

	imu_sensor->gx_bias = temp_bias_x;
	imu_sensor->gy_bias = temp_bias_y;
	imu_sensor->gz_bias = temp_bias_z;


	return LSM6DSR_OK;
}

void lsm6dsr_read_data(lsm6dsr_handler* imu_sensor)
{
	uint8_t temp_data[12];
	lsm6dsr_read_reg(imu_sensor, LSM6DSR_OUTX_L_G_REG, temp_data, 12);
	imu_sensor->data.gx = (((int16_t)temp_data[1]<<8)| (int16_t)temp_data[0]) - imu_sensor->gx_bias;
	imu_sensor->data.gy = (((int16_t)temp_data[3]<<8)| (int16_t)temp_data[2]) - imu_sensor->gy_bias;
	imu_sensor->data.gz = (((int16_t)temp_data[5]<<8)| (int16_t)temp_data[4]) - imu_sensor->gz_bias;
	imu_sensor->data.ax = ((int16_t)temp_data[7]<<8)| (int16_t)temp_data[6];
	imu_sensor->data.ay = ((int16_t)temp_data[9]<<8)| (int16_t)temp_data[8];
	imu_sensor->data.az = ((int16_t)temp_data[11]<<8)| (int16_t)temp_data[10];
}

FreeFallStatusTypeDef lsm6dsr_free_fall(lsm6dsr_handler* imu_sensor)
{
	uint8_t temp_data;
	lsm6dsr_read_reg(imu_sensor, LSM6DSR_ALL_INT_SRC_REG, &temp_data, 1);
	if(temp_data & 0x01)
	{
		return FREE_FALL;
	}
	else
	{
		return NO_FREE_FALL;
	}
}

