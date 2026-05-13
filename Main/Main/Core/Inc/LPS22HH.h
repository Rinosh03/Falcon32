/*
 * LPS22HH.h
 *
 *  Created on: Aug 9, 2025
 *      Author: yerke
 */

#ifndef INC_LPS22HH_H_
#define INC_LPS22HH_H_

#include "stdint.h"
#include "main.h"

#define LPS22HH_WHO_AM_I_REG   					0x0f
#define LPS22HH_CTRL_REG1_REG   				0x10
#define LPS22HH_CTRL_REG2_REG   				0x11
#define LPS22HH_CTRL_REG3_REG   				0x12
#define LPS22HH_REF_P_L_REG   					0x15
#define LPS22HH_REF_P_H_REG   					0x16
#define LPS22HH_PRESS_OUT_XL_REG   				0x28
#define LPS22HH_PRESS_OUT_L_REG   				0x29
#define LPS22HH_PRESS_OUT_H_REG   				0x2a
#define LPS22HH_TEMP_OUT_L_REG   				0x2b
#define LPS22HH_TEMP_OUT_H_REG   				0x2c

typedef struct
{
	SPI_HandleTypeDef 	*spi_handler;
	GPIO_TypeDef 		*gpio_cs_port;
	uint16_t 			gpio_cs_pin;
	int32_t              	pressure;
	int32_t              	pressure_base;
	int16_t 				temperature;
	int16_t 				temperature_base;
	float               altitude;
}lps22hh_handler;


void lps22hh_init(lps22hh_handler *pressure_sensor);
void lps22hh_estimate_altitude(lps22hh_handler *pressure_sensor);

#endif /* INC_LPS22HH_H_ */
