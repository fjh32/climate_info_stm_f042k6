/*
 * lcd.h
 *
 *  Created on: Dec 17, 2024
 *      Author: frank
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

//#include "stm32f4xx_hal.h"
#include "main.h"


#define LCD_I2C_ADDR 0x4E

void lcd_clear_display(I2C_HandleTypeDef *hi2c);
void lcd_print_2_lines(I2C_HandleTypeDef *hi2c, char*, char*);


void lcd_print_string (I2C_HandleTypeDef *hi2c, char *str);
void lcd_place_cursor(I2C_HandleTypeDef *hi2c, int row, int col);

/* hi2c needs these settings
 * static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}
 */

#endif /* INC_LCD_H_ */
