/*
 * lcd.c
 *
 *  Created on: Dec 17, 2024
 *      Author: frank
 */
#include "lcd.h"



static HAL_StatusTypeDef lcd_send_command(I2C_HandleTypeDef *hi2c, char cmd)
{
	char data_upper, data_lower;
	data_upper = (cmd & 0xf0);
	data_lower = ((cmd << 4) & 0xf0);
	uint8_t data[4];
	data[0] = data_upper | 0x0C; //en=1, rs=0 -> bxxxx1100
	data[1] = data_upper | 0x08; //en=0, rs=0 -> bxxxx1000
	data[2] = data_lower | 0x0C;
	data[3] = data_lower | 0x08;
	HAL_StatusTypeDef retval;
	retval = HAL_I2C_Master_Transmit (hi2c, LCD_I2C_ADDR, data, 4, 100);
	return retval;
}

static HAL_StatusTypeDef lcd_send_data(I2C_HandleTypeDef *hi2c, char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u | 0x09;  //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l | 0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l | 0x09;  //en=0, rs=1 -> bxxxx1001
	HAL_StatusTypeDef retval;
	retval = HAL_I2C_Master_Transmit (hi2c, LCD_I2C_ADDR, data_t, 4, 100);
	return retval;
}

static HAL_StatusTypeDef lcd_init_settings(I2C_HandleTypeDef *hi2c)
{
	static uint8_t initialized = 0;

	if(!initialized)
	{
		if(HAL_I2C_IsDeviceReady(hi2c, LCD_I2C_ADDR, 10, 1000) != HAL_OK) {
			return HAL_ERROR;
		}
		// 4 bit initialisation
		HAL_Delay(50);  // wait for >40ms
		lcd_send_command (hi2c, 0x30);
		HAL_Delay(5);  // wait for >4.1ms
		lcd_send_command (hi2c, 0x30);
		HAL_Delay(1);  // wait for >100us
		lcd_send_command (hi2c, 0x30);
		HAL_Delay(10);
		lcd_send_command (hi2c, 0x20);  // 4bit mode
		HAL_Delay(10);

		// display initialisation
		lcd_send_command (hi2c, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
		HAL_Delay(1);
		lcd_send_command (hi2c, 0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
		HAL_Delay(1);
		lcd_send_command (hi2c, 0x01);  // clear display
		HAL_Delay(2);
		lcd_send_command (hi2c, 0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
		HAL_Delay(1);
		lcd_send_command (hi2c, 0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
		HAL_Delay(100);

		initialized = 1;
	}
	return HAL_OK;
}


void lcd_print_string (I2C_HandleTypeDef *hi2c, char *str)
{
	lcd_init_settings(hi2c);
	while (*str) lcd_send_data (hi2c, *str++);
}

void lcd_place_cursor(I2C_HandleTypeDef *hi2c, int row, int col)
{
	lcd_init_settings(hi2c);
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_command (hi2c, col);
}

void lcd_clear_display(I2C_HandleTypeDef *hi2c)
{
	lcd_send_command (hi2c, 0x01);
	HAL_Delay(10);
}

void lcd_print_2_lines(I2C_HandleTypeDef *hi2c, char* top_line, char* bot_line)
{
	lcd_clear_display(hi2c);
	lcd_place_cursor(hi2c, 0, 0);
	lcd_print_string(hi2c, top_line);
	lcd_place_cursor(hi2c, 1, 0);
	lcd_print_string(hi2c, bot_line);
}
