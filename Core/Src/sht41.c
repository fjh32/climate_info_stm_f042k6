/*
 * sht41.c
 *
 *  Created on: Dec 17, 2024
 *      Author: frank
 */

#include "sht41.h"


HAL_StatusTypeDef get_temperature_sht41(I2C_HandleTypeDef *i2ch, float* data_buffer)
{
	uint8_t inc_bytes[6] = {0};

	if(HAL_I2C_IsDeviceReady(i2ch, SHT41_I2C_ADDR, 10, 1000) != HAL_OK) {
	  return HAL_ERROR;
	}

	HAL_Delay(10);
	uint8_t command = 0xFD;
	if(HAL_I2C_Master_Transmit(i2ch, SHT41_I2C_ADDR, &command, 1, 1000) != HAL_OK) {
	  return HAL_ERROR;
	}

	HAL_Delay(10);
	if(HAL_I2C_Master_Receive(i2ch, SHT41_I2C_ADDR, &inc_bytes, 6, 1000) != HAL_OK) {
	  return HAL_ERROR;
	}

	uint16_t raw_temp = (inc_bytes[0] << 8) | inc_bytes[1];
	uint8_t temp_checksum = inc_bytes[2];
	uint16_t raw_humidity = (inc_bytes[3] << 8) | inc_bytes[4];
	uint8_t humidity_checksum = inc_bytes[5];

	float tempc = -45 + 175 * (raw_temp / 65535.0);
	float tempf = 32 + (tempc * (9.0/5));
	float humidity_percent = -6 + 125 * (raw_humidity /65535.0);

	HAL_Delay(10);

	data_buffer[0] = tempf;
	data_buffer[1] = humidity_percent;
	return HAL_OK;
}
