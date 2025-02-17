/*
 * eeprom.c
 *
 *  Created on: Dec 24, 2024
 *      Author: frank
 */

#include "eeprom.h"
/*
static HAL_StatusTypeDef eeprom_write_data(I2C_HandleTypeDef *i2c, uint16_t eeprom_mem_addr, const void * data, size_t size)
{
	HAL_StatusTypeDef retval = HAL_OK;

	const uint8_t *data_bytes = (const uint8_t *)&data;
	uint8_t send_buffer[3];
	for(uint8_t i = 0; i < size; i++)
	{
		uint16_t eeprom_store_addr = eeprom_mem_addr + i;
		send_buffer[0] = (uint8_t)((eeprom_store_addr >> 8) & 0xFF);  // High Byte
		send_buffer[1] = (uint8_t)(eeprom_store_addr & 0xFF);         // Low Byte
		send_buffer[2] = data_bytes[i];
		retval = HAL_I2C_Master_Transmit(i2c, EEPROM_I2C_ADDRESS, send_buffer, 3, HAL_MAX_DELAY);
		if(retval != HAL_OK)
		{
			return retval;
		}
		HAL_Delay(10);
	}
	return HAL_OK;
}

static HAL_StatusTypeDef eeprom_read_data(I2C_HandleTypeDef *i2c, uint16_t eeprom_mem_addr, void *value_to_read, size_t size)
{
	HAL_StatusTypeDef retval = HAL_OK;
	uint8_t send_buffer[2];
	send_buffer[0] = (uint8_t)((eeprom_store_addr >> 8) & 0xFF);  // High Byte
	send_buffer[1] = (uint8_t)(eeprom_store_addr & 0xFF);

	retval = HAL_I2C_Master_Transmit(i2c, EEPROM_I2C_ADDRESS, send_buffer, 2, HAL_MAX_DELAY);
	if(retval != HAL_OK)
	{
		return retval;
	}
	HAL_Delay(10);

	retval = HAL_I2C_Master_Receive(i2c, EEPROM_I2C_ADDRESS, (uint8_t *)value_bytes, size, HAL_MAX_DELAY);
	if(retval != HAL_OK)
	{
		return retval;
	}
	HAL_Delay(10);

	return HAL_OK;
}
*/

