/*
 * climate_data.c
 *
 *  Created on: Dec 18, 2024
 *      Author: frank
 */


#include "eeprom_climate_data.h"
#include "sht41.h"
#include <string.h>


////////////////////////////////////////////////////////////
static HAL_StatusTypeDef eeprom_wait_ready(I2C_HandleTypeDef *i2c) {
    uint8_t dummy = 0;
    while (HAL_I2C_Master_Transmit(i2c, EEPROM_I2C_ADDRESS, &dummy, 0, HAL_MAX_DELAY) != HAL_OK) {
        if (HAL_GetError() != HAL_BUSY) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

// #define EEPROM_I2C_ADDRESS 0x50 << 1 is defined in header file
static HAL_StatusTypeDef eeprom_write_float(I2C_HandleTypeDef *i2c, uint16_t eeprom_mem_addr, float value)
{
	HAL_StatusTypeDef retval = HAL_OK;

	uint8_t *value_bytes = (uint8_t *)&value;
	uint8_t send_buffer[3];
	for(uint8_t i = 0; i < 4; i++)
	{
		uint16_t eeprom_store_addr = eeprom_mem_addr + i;
		send_buffer[0] = (uint8_t)((eeprom_store_addr >> 8) & 0xFF);  // High Byte
		send_buffer[1] = (uint8_t)(eeprom_store_addr & 0xFF);         // Low Byte
		send_buffer[2] = value_bytes[i];
		retval = HAL_I2C_Master_Transmit(i2c, EEPROM_I2C_ADDRESS, send_buffer, 3, HAL_MAX_DELAY);
		if(retval != HAL_OK)
		{
			return retval;
		}
		HAL_Delay(10);
	}
	return HAL_OK;
}

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


static HAL_StatusTypeDef eeprom_read_float(I2C_HandleTypeDef *i2c, uint16_t eeprom_mem_addr, float *value_to_read)
{
	HAL_StatusTypeDef retval = HAL_OK;

	uint8_t value_bytes[4];
	uint8_t send_buffer[2];
	for(uint8_t i = 0; i < 4; i++)
	{
		uint16_t eeprom_store_addr = eeprom_mem_addr + i;
		send_buffer[0] = (uint8_t)((eeprom_store_addr >> 8) & 0xFF);  // High Byte
		send_buffer[1] = (uint8_t)(eeprom_store_addr & 0xFF);         // Low Byte

		retval = HAL_I2C_Master_Transmit(i2c, EEPROM_I2C_ADDRESS, send_buffer, 2, HAL_MAX_DELAY);
		if(retval != HAL_OK)
		{
			return retval;
		}
		HAL_Delay(10);

		retval = HAL_I2C_Master_Receive(i2c, EEPROM_I2C_ADDRESS, &(value_bytes[i]), 1, HAL_MAX_DELAY);
		if(retval != HAL_OK)
		{
			return retval;
		}
		HAL_Delay(10);
	}

	memcpy(value_to_read, value_bytes, sizeof(float));

	return HAL_OK;
}

/*

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

/*
 * This function needs to initialize a new climate_data struct
 * 1) get current values from sht41
 * 2) get min/max values from eeprom
 */
HAL_StatusTypeDef init_climate_data(climate_data * cdata, I2C_HandleTypeDef * hi2c)
{
	HAL_StatusTypeDef retval = HAL_OK;
	cdata->hal_i2c = hi2c;
	float current_climate_vals[2];
	if(get_temperature_sht41(cdata->hal_i2c, current_climate_vals) != HAL_OK)
	{
		return HAL_ERROR;
	}
	cdata->current_temp = current_climate_vals[0];
	cdata->current_hum  = current_climate_vals[1];

	if(read_eeprom_into_climate_data(cdata) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}


/*
 * Gets the current values from the sht41,
 * Compares the current values to the input climate_data pointer.
 * Update input climate_data values with new values where appropriate.
 * If anything changed, write climate_data to eeprom.
 */
HAL_StatusTypeDef update_climate_data_in_eeprom(climate_data * data)
{
	HAL_StatusTypeDef retval = HAL_OK;
	float current_climate_vals[2];
	retval |= get_temperature_sht41(data->hal_i2c, current_climate_vals);

	data->current_temp = current_climate_vals[0];
	data->current_hum = current_climate_vals[1];

	uint8_t updateEEPROMorNot = 0;
	if(current_climate_vals[0] > data->max_temp)
	{
		data->max_temp = current_climate_vals[0];
		updateEEPROMorNot = 1;
	}
	if(current_climate_vals[0] < data->min_temp)
	{
		data->min_temp = current_climate_vals[0];
		updateEEPROMorNot = 1;
	}
	if(current_climate_vals[1] > data->max_hum)
	{
		data->max_hum = current_climate_vals[1];
		updateEEPROMorNot = 1;
	}
	if(current_climate_vals[1] < data->min_hum)
	{
		data->min_hum = current_climate_vals[1];
		updateEEPROMorNot = 1;
	}

	if(updateEEPROMorNot)
	{
		retval|= write_eeprom_from_climate_data(data);
	}


	return retval;
}

HAL_StatusTypeDef read_eeprom_into_climate_data(climate_data * data)
{
	if( eeprom_read_float(data->hal_i2c, EEPROM_MAX_TEMP_ADDR, &(data->max_temp)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_read_float(data->hal_i2c, EEPROM_MIN_TEMP_ADDR, &(data->min_temp)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_read_float(data->hal_i2c, EEPROM_MAX_HUM_ADDR, &(data->max_hum)) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_read_float(data->hal_i2c, EEPROM_MIN_HUM_ADDR, &(data->min_hum)) != HAL_OK)
	{
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef write_eeprom_from_climate_data(climate_data * data)
{
	if( eeprom_write_float(data->hal_i2c, EEPROM_MAX_TEMP_ADDR, data->max_temp) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_write_float(data->hal_i2c, EEPROM_MIN_TEMP_ADDR, data->min_temp) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_write_float(data->hal_i2c, EEPROM_MAX_HUM_ADDR, data->max_hum) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if( eeprom_write_float(data->hal_i2c, EEPROM_MIN_HUM_ADDR, data->min_hum) != HAL_OK)
	{
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef reset_climate_data_eeprom(I2C_HandleTypeDef * hi2c)
{
	HAL_StatusTypeDef retval = HAL_OK;
	float current_climate_vals[2];
	retval |= get_temperature_sht41(hi2c, current_climate_vals);

	climate_data cdata;
	cdata.current_temp = current_climate_vals[0];
	cdata.current_hum  = current_climate_vals[1];
	cdata.max_temp = current_climate_vals[0];
	cdata.min_temp = current_climate_vals[0];
	cdata.max_hum  = current_climate_vals[1];
	cdata.min_hum  = current_climate_vals[1];
	cdata.hal_i2c = hi2c;

	retval |= write_eeprom_from_climate_data(&cdata);

	return retval;
}


/*
 * HAL_StatusTypeDef EEPROM_WriteByte(I2C_HandleTypeDef *hi2c1, uint16_t memAddress, uint8_t byte_to_write)
{
    uint8_t buffer[3];

    // Split 16-bit address into high and low bytes
    buffer[0] = (uint8_t)((memAddress >> 8) & 0xFF);  // High Byte
    buffer[1] = (uint8_t)(memAddress & 0xFF);         // Low Byte
    buffer[2] = byte_to_write;                                // Data to Write

    // Send data to EEPROM
    HAL_StatusTypeDef retval =  HAL_I2C_Master_Transmit(hi2c1, EEPROM_ADDRESS, buffer, 3, HAL_MAX_DELAY);
    HAL_Delay(10);
    return retval;

}

HAL_StatusTypeDef EEPROM_ReadByte(I2C_HandleTypeDef *hi2c1, uint16_t memAddress, uint8_t *data_buffer)
{
    uint8_t buffer[2];

    // Split 16-bit address into high and low bytes
    buffer[0] = (uint8_t)((memAddress >> 8) & 0xFF);  // High Byte
    buffer[1] = (uint8_t)(memAddress & 0xFF);         // Low Byte

    // Send memory address to EEPROM
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c1, EEPROM_ADDRESS, buffer, 2, HAL_MAX_DELAY);
    HAL_Delay(10);
    if (status != HAL_OK)
        return status;

    // Read data from EEPROM
    status = HAL_I2C_Master_Receive(hi2c1, EEPROM_ADDRESS, data_buffer, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    return status;
}

HAL_StatusTypeDef read_min_max_climate_vals(I2C_HandleTypeDef * hi2c, uint16_t store_address, int8_t * data_buffer)
{

	HAL_StatusTypeDef retval = HAL_OK;
	for(uint8_t i = 0; i < 4; i++)
	{
		uint16_t current_addr = store_address + i;
		retval |= EEPROM_ReadByte(hi2c, current_addr, (uint8_t *)&(data_buffer[i]));
	}

	return retval;
}

HAL_StatusTypeDef write_min_max_climate_vals(I2C_HandleTypeDef * hi2c, uint16_t store_address, uint8_t * climate_data)
{
	HAL_StatusTypeDef retval = HAL_OK;
	for(uint8_t i = 0; i < 4; i++)
	{
		uint16_t current_addr = store_address + i;
		retval |= EEPROM_WriteByte(hi2c, current_addr, (uint8_t) climate_data[i]);
	}
	return retval;
}
HAL_StatusTypeDef reset_eeprom(I2C_HandleTypeDef * hi2c, uint16_t store_address)
{
	int8_t climate_data_buffer[4] = {-127, 127, -127, 127};
	HAL_StatusTypeDef retval = write_min_max_climate_vals(&hi2c1, store_address, climate_data_buffer);
	return retval;
}
 *
 */
