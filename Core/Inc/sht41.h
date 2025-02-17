/*
 * sht41.h
 *
 *  Created on: Dec 17, 2024
 *      Author: frank
 */

#ifndef INC_SHT41_H_
#define INC_SHT41_H_

//#include "stm32f4xx_hal.h"
#include "main.h"


#define SHT41_I2C_ADDR (0x44 << 1)


HAL_StatusTypeDef get_temperature_sht41(I2C_HandleTypeDef *i2ch, float* data_buffer);

#endif /* INC_SHT41_H_ */
