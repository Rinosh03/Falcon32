#include "LPS22HH.h"

#define ALPHA_PRESSUE  0.1f

static void lps22hh_activate(lps22hh_handler* pressure_sensor)
{
	HAL_GPIO_WritePin(pressure_sensor->gpio_cs_port, pressure_sensor->gpio_cs_pin,
			GPIO_PIN_RESET);
}
static void lps22hh_deactivate(lps22hh_handler* pressure_sensor)
{
	HAL_GPIO_WritePin(pressure_sensor->gpio_cs_port, pressure_sensor->gpio_cs_pin,
				GPIO_PIN_SET);
}
static void lps22hh_read_reg(lps22hh_handler* pressure_sensor, uint8_t address, uint8_t *data, uint8_t size)
{
	uint8_t temp_data = 0x80|address;
	lps22hh_activate(pressure_sensor);
	HAL_SPI_Transmit(pressure_sensor->spi_handler, &temp_data , 1, 100);
	HAL_SPI_Receive(pressure_sensor->spi_handler, data, size, 100);
	lps22hh_deactivate(pressure_sensor);
}

static void lps22hh_write_reg(lps22hh_handler *pressure_sensor, uint8_t reg, uint8_t data)
{
	lps22hh_activate(pressure_sensor);
	HAL_SPI_Transmit(pressure_sensor->spi_handler, &reg, 1, 100);
	HAL_SPI_Transmit(pressure_sensor->spi_handler, &data, 1, 100);
	lps22hh_deactivate(pressure_sensor);
}

void lps22hh_init(lps22hh_handler *pressure_sensor)
{
	uint8_t temp_data;
	int32_t pressure_base_sum = 0, temperature_base_sum = 0;
	// software reset, enable automatic increment
	temp_data = 0x4;
	lps22hh_write_reg(pressure_sensor, LPS22HH_CTRL_REG1_REG, temp_data);
	HAL_Delay(10);
	temp_data = 0x12;
	lps22hh_write_reg(pressure_sensor, LPS22HH_CTRL_REG1_REG, temp_data);
	HAL_Delay(10);

	// 200 Hz data rate  ODR/9 bandwidth, Block update
	temp_data = 0b01011110;
	lps22hh_write_reg(pressure_sensor, LPS22HH_CTRL_REG1_REG, temp_data);
	pressure_sensor ->pressure_base = 0;
	pressure_sensor->temperature_base = 0;
	// computing base pressure and temperature
	HAL_Delay(50);
	lps22hh_estimate_altitude(pressure_sensor);
	HAL_Delay(50);
	for(int i = 0; i < 16; i++)
	{
		lps22hh_estimate_altitude(pressure_sensor);
		pressure_base_sum += pressure_sensor->pressure;
		temperature_base_sum += pressure_sensor->temperature;
		HAL_Delay(20);
	}
	pressure_sensor ->pressure_base = pressure_base_sum  / 16;
	pressure_sensor->temperature_base = temperature_base_sum / 16;
}

void lps22hh_estimate_altitude(lps22hh_handler *pressure_sensor)
{
	uint8_t temp_data[5];
	float altitude_raw;
	lps22hh_read_reg(pressure_sensor, LPS22HH_PRESS_OUT_XL_REG, temp_data, 5);

	pressure_sensor->pressure = (int32_t)((temp_data[0])|(temp_data[1] << 8)|(temp_data[2]<<16));
	pressure_sensor->temperature = (int16_t)((temp_data[3])|(temp_data[4] << 8));
	pressure_sensor->pressure -= pressure_sensor ->pressure_base;

	// 8.5 / 4096 = 0.002001953125f
	if(pressure_sensor ->pressure_base)
	{
		altitude_raw = -((float)pressure_sensor->pressure) * 0.002001953125f;
		pressure_sensor->altitude += (altitude_raw - pressure_sensor->altitude) * ALPHA_PRESSUE;

	}
	else
	{
		pressure_sensor->altitude = 0;
	}

}
