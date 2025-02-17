/*
 * climate_data.h
 *
 *  Created on: Dec 18, 2024
 *      Author: frank
 */

#ifndef INC_EEPROM_CLIMATE_DATA_H_
#define INC_EEPROM_CLIMATE_DATA_H_

//#include "stm32f4xx_hal.h"
#include "main.h"



#define EEPROM_I2C_ADDRESS 0x50 << 1

#define EEPROM_MAX_TEMP_ADDR 0x1555
#define EEPROM_MIN_TEMP_ADDR 0x1559
#define EEPROM_MAX_HUM_ADDR 0x155D
#define EEPROM_MIN_HUM_ADDR 0x1561



typedef struct __climate_data
{
	float max_temp;
	float min_temp;
	float max_hum;
	float min_hum;

	float current_temp;
	float current_hum;

	// This assumes the sht41 and the eeprom are both hooked up to the same i2c bus
	I2C_HandleTypeDef *hal_i2c;
} climate_data;

/*
 * This function needs to initialize a new climate_data struct
 * 1) get current values from sht41
 * 2) get min/max values from eeprom
 */
HAL_StatusTypeDef init_climate_data(climate_data * cdata, I2C_HandleTypeDef * hi2c);

/*
 * Gets the current values from the sht41,
 * Compares the current values to the input climate_data pointer.
 * Update input climate_data values with new values where appropriate.
 * If anything changed, write climate_data to eeprom.
 */
HAL_StatusTypeDef update_climate_data_in_eeprom(climate_data * data);

HAL_StatusTypeDef read_eeprom_into_climate_data(climate_data * data);
HAL_StatusTypeDef write_eeprom_from_climate_data(climate_data * data);
HAL_StatusTypeDef reset_climate_data_eeprom(I2C_HandleTypeDef * hi2c);

#endif /* INC_EEPROM_CLIMATE_DATA_H_ */
