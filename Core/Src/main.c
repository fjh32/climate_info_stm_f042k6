/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "sht41.h"
#include "eeprom_climate_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONSECUTIVE_BUTTON_PRESS_THRESHOLD 20
#define RESET_EEPROM_BUTTON_PRESS_THRESHOLD 1000 // 10 seconds at 100Hz
#define NUMBER_LCD_SCREENS 4
#define EEPROM_ADDRESS 0x50 << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
typedef void (* lcd_screen_func)(climate_data *clim_data);
lcd_screen_func lcd_screens[NUMBER_LCD_SCREENS];
uint8_t current_lcd_screen = 0;



volatile int16_t consecutive_button_presses = 0;
volatile int button_handled = 0;
volatile uint8_t lcd_update_pending = 0;
volatile uint8_t sample_pending = 0;  // Flag for temperature sampling
volatile uint8_t reset_eeprom_flag = 0;
volatile uint8_t _1_second_elapsed = 0;

volatile uint8_t lcd_refresh_clock = 0;

climate_data cdata;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void init_lcd_screens();
void write_temps_lcd(I2C_HandleTypeDef * hi2c, char * temp_buffer, char * hum_buffer);
void lcd_print_current_climate(climate_data *clim_data);
void lcd_print_minmax_temp(climate_data *clim_data);
void lcd_print_minmax_hum(climate_data *clim_data);
void lcd_print_time(climate_data *clim_data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Button class could expose a callback function to be put into HAL_TIM_PeriodElapsedCallback
 * It interally read the pin, check previous and current states, etc.
 * Button handler struct could allow a user to provide a callback function that runs when button is pressed
 * How does arduino handle button press API?
 * 	- blocking call to WaitForButtonPress()?
 * 	- Callback when a button is pressed?
 * 	-
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t tick_counter = 0;
	static uint16_t button_down_counter = 0;
	static uint16_t button_up_counter = 0;
	static GPIO_PinState last_button_state = GPIO_PIN_RESET;  // Assume button is normally HIGH
	static uint8_t button_pressed = 0;

	if (htim->Instance == TIM2)  // 100 Hz timer
	{
		GPIO_PinState buttonState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
		if (buttonState == GPIO_PIN_SET && last_button_state == GPIO_PIN_RESET)
		{
			button_pressed = 1;
			last_button_state = GPIO_PIN_SET;
		}
		else if(buttonState == GPIO_PIN_RESET)
		{
			button_pressed = 0;
			last_button_state = GPIO_PIN_RESET;
		}
		else
		{

		}

		if(button_pressed)
		{
			button_pressed = 0;
			current_lcd_screen = (current_lcd_screen + 1) % NUMBER_LCD_SCREENS;
			lcd_update_pending = 1;  // Signal LCD update
		}

		// Temperature sampling
		if (++tick_counter >= 300)  // 3 seconds at 100 Hz
		{
			tick_counter = 0;
			sample_pending = 1;
		}
		else if(tick_counter >= 100) // 1 second
		{
			_1_second_elapsed = 1;
		}


	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	if(current_lcd_screen == 3)
	{
		lcd_refresh_clock = 1;
	}
}

void set_time (uint8_t hr, uint8_t min, uint8_t sec)
{
	RTC_TimeTypeDef sTime = {0};

	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day)  // monday = 1
{

	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = day;
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
}

void get_time_date(char *time, char *date)
{
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  /* Display time Format: hh:mm:ss */
  sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);

  /* Display date Format: dd-mm-yyyy */
  sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
}


void init_lcd_screens()
{
	//uint8_t amount_screens = sizeof(lcd_screens) / sizeof(lcd_screens[0]); // way to calculate
	lcd_screens[0] = lcd_print_current_climate;
	lcd_screens[1] = lcd_print_minmax_temp;
	lcd_screens[2] = lcd_print_minmax_hum;
	lcd_screens[3] = lcd_print_time;
	lcd_screens[0](&cdata);
}

void write_temps_lcd(I2C_HandleTypeDef * hi2c, char * temp_buffer, char * hum_buffer)
{
	//uart_print(huart, temp_buffer);
	//uart_print(huart, hum_buffer);

	lcd_clear_display(hi2c);
	lcd_place_cursor(hi2c, 0, 0);
	lcd_print_string(hi2c, temp_buffer);
	lcd_place_cursor(hi2c, 1, 0);
	lcd_print_string(hi2c, hum_buffer);
}

void lcd_print_current_climate(climate_data *clim_data)
{
	char temp_str_buffer[32] = {0};
	char hum_str_buffer[32] = {0};

	sprintf(temp_str_buffer, "Cur Temp: %dF", (int) clim_data->current_temp);
	sprintf(hum_str_buffer, "Cur Hum:  %d%%", (int) clim_data->current_hum);

	lcd_print_2_lines(clim_data->hal_i2c, temp_str_buffer, hum_str_buffer);
}

void lcd_print_minmax_temp(climate_data *clim_data)
{
	char temp_str_buffer[32] = {0};
	char hum_str_buffer[32] = {0};

	sprintf(temp_str_buffer, "Max Temp: %dF", (int) clim_data->max_temp);
	sprintf(hum_str_buffer, "Min Temp: %dF", (int) clim_data->min_temp);

	lcd_print_2_lines(clim_data->hal_i2c, temp_str_buffer, hum_str_buffer);
}

void lcd_print_minmax_hum(climate_data *clim_data)
{
	char temp_str_buffer[32] = {0};
	char hum_str_buffer[32] = {0};

	sprintf(temp_str_buffer, "Max Hum: %d%%", (int) clim_data->max_hum);
	sprintf(hum_str_buffer, "Min Hum: %d%%", (int) clim_data->min_hum);

	lcd_print_2_lines(clim_data->hal_i2c, temp_str_buffer, hum_str_buffer);
}

void lcd_print_time(climate_data *clim_data)
{
	RTC_TimeTypeDef gTime;

	char time_buf[32] = {0};
	char date_buf[32] = {0};

	get_time_date(time_buf, date_buf);

	lcd_print_2_lines(clim_data->hal_i2c, "Uptime:", time_buf);
}


/*
void print_climate_data_to_uart(climate_data * climdata, UART_HandleTypeDef *huart)
{
	char str_buffer[32] = {0};
	sprintf(str_buffer, "Cur Temp: %dF", (int) climdata->current_temp);
	uart_print(huart, str_buffer);

	sprintf(str_buffer, "Cur Hum: %d%%", (int) climdata->current_hum);
	uart_print(huart, str_buffer);

	sprintf(str_buffer, "Max Temp: %dF", (int) climdata->max_temp);
	uart_print(huart, str_buffer);

	sprintf(str_buffer, "Min Temp: %dF", (int) climdata->min_temp);
	uart_print(huart, str_buffer);

	sprintf(str_buffer, "Max Hum: %d%%", (int) climdata->max_hum);
	uart_print(huart, str_buffer);

	sprintf(str_buffer, "Min Hum: %d%%", (int) climdata->min_hum);
	uart_print(huart, str_buffer);
}
*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_StatusTypeDef retval = HAL_OK;
  //retval = reset_climate_data_eeprom(&hi2c1);
  retval = init_climate_data(&cdata, &hi2c1);
  init_lcd_screens();

  set_time(0,0,0);
  while (1)
  {
	  if(lcd_refresh_clock)
	  {
		  lcd_refresh_clock = 0;
		  lcd_screens[current_lcd_screen](&cdata);
	  }

	  if (lcd_update_pending)
	  {
		  lcd_update_pending = 0;  // Clear the flag
		  lcd_clear_display(cdata.hal_i2c);
		  HAL_Delay(10);
		  lcd_screens[current_lcd_screen](&cdata);  // Update LCD immediately
	  }

	  // Check for temperature sampling
	  uint8_t trigger_screen_refresh = 0;
	  if(reset_eeprom_flag)
	  {
		  reset_eeprom_flag = 0;

		  lcd_print_2_lines(&hi2c1, "RESETTING EEPROM", ".......");
		  HAL_Delay(2500);
		  if (reset_climate_data_eeprom(&hi2c1) != HAL_OK)
		  {
			  break;
		  }
		  if(init_climate_data(&cdata, &hi2c1) != HAL_OK)
		  {
			  break;
		  }
		  sample_pending = 1;
		  trigger_screen_refresh = 1;
		  HAL_Delay(1000);
	  }


	  if (sample_pending)
	  {
		  sample_pending = 0;  // Clear the flag
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

		  if (update_climate_data_in_eeprom(&cdata) != HAL_OK)
		  {
			  break;
		  }
		  //print_climate_data_to_uart(&cdata, &huart2);
		  if(current_lcd_screen == 0 || trigger_screen_refresh)
		  {
			  lcd_screens[current_lcd_screen](&cdata);
		  }
		 // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
	  }
/*
	  if(_1_second_elapsed && current_lcd_screen == 3)
	  {
		  lcd_print_time(&cdata);
		  _1_second_elapsed = 0;
	  }
	  */

	  // Other tasks can be added here...

	  __WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 295;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x1;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
