/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
void setup_pin_states(void);
void LED_blinky(void);
void testFunc(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC_Init();

  //uint32_t adc_values[10];

  // initialize digital output pin states
  setup_pin_states();

  //blinky on start up
  LED_blinky();

  //test function
  testFunc();

  while (1)
    {
      // no-op for break point
      asm("nop");

      // blink heart beat LED to indicate FW 'OK'
      HAL_GPIO_WritePin(GPIOB, DEBUG_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOB, DEBUG_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);

      // sample and update DIGITAL INPUTS
      HAL_GPIO_ReadPin(GPIOC, UV_INPUT_Pin); // BMS UV
      HAL_GPIO_ReadPin(GPIOC, OV_INPUT_Pin); // BMS OV

      // sample and update ANALOG INPUTS

	  // fuse status (valid range = 2.036V - 2.655V)

	  // thermistors (shunt and isolated PCBA temp)

      // update DIGITAL OUTPUTS

	  // UV: LED and optoiso

	  // OV: LED and optoiso

	  // Fuse status LEDs

	  // Heater LED

	  // Heater ENABLE (check PCBA thermistor temperatures)

	  // disable GV if SHUNT thermistor is too high/low

	  // disable GV if PCBA thermistor is too high/low

    }
}

void setup_pin_states(void)
{
  HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT2_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT3_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT4_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT5_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT6_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, _60A_CKT2_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, DEBUG_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, UV_TO_INV_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, OV_TO_CHGR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GV_CLOSE_PULSE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GV_OPEN_PULSE_Pin, GPIO_PIN_RESET);
}

void LED_blinky(void)
{
  HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, _30A_CKT2_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT2_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, _30A_CKT3_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT3_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, _30A_CKT4_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT4_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, _30A_CKT5_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT5_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, _30A_CKT6_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOA, _30A_CKT6_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, _60A_CKT2_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOB, _60A_CKT2_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);
}

void testFunc(void)
{
  uint8_t i;
  for(i=0; i<10; i++)
  {
    HAL_GPIO_WritePin(GPIOB, UV_TO_INV_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);
    HAL_GPIO_WritePin(GPIOB, UV_TO_INV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_RESET);


    HAL_GPIO_WritePin(GPIOB, OV_TO_CHGR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);
    HAL_GPIO_WritePin(GPIOB, OV_TO_CHGR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_RESET);
  }
}

void adcScan(void)
{
  uint8_t i;
  averaged_shunt_val_new = 0;
  averaged_i_set_new = 0;
  averaged_time_set_new = 0;
  for(i = 0; i < 100; i++)
  {
    averaged_shunt_val_new = averaged_shunt_val_new + adc_values[0];
    averaged_i_set_new = averaged_i_set_new + adc_values[1];
    averaged_time_set_new = averaged_time_set_new + adc_values[2];
  }
  averaged_shunt_val_new = averaged_shunt_val_new/1000;
  averaged_i_set_new = averaged_i_set_new/1000;
  averaged_time_set_new = averaged_time_set_new/1000;

  fShunt_val = ((float)averaged_shunt_val_new/4095*3.3)/0.100;
  fIset_val = (float)averaged_i_set_new/4095*10;
  fTimeSet_val = (float)averaged_time_set_new/4095*10;

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      Error_Handler();
    }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
      Error_Handler();
    }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin|_60A_CKT2_LED_Pin|UV_TO_INV_Pin|OV_TO_CHGR_Pin
		    |GV_CLOSE_PULSE_Pin|GV_OPEN_PULSE_Pin|DEBUG_LED_Pin|UV_INDICATE_LED_Pin
		    |OV_INDICATE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin|_30A_CKT2_LED_Pin|_30A_CKT3_LED_Pin|_30A_CKT4_LED_Pin
		    |_30A_CKT5_LED_Pin|_30A_CKT6_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEATER_EN_Pin */
  GPIO_InitStruct.Pin = HEATER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEATER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_INPUT_Pin OV_INPUT_Pin */
  GPIO_InitStruct.Pin = UV_INPUT_Pin|OV_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : _60A_CKT1_LED_Pin _60A_CKT2_LED_Pin UV_TO_INV_Pin OV_TO_CHGR_Pin
                           GV_CLOSE_PULSE_Pin GV_OPEN_PULSE_Pin DEBUG_LED_Pin UV_INDICATE_LED_Pin
                           OV_INDICATE_LED_Pin */
  GPIO_InitStruct.Pin = _60A_CKT1_LED_Pin|_60A_CKT2_LED_Pin|UV_TO_INV_Pin|OV_TO_CHGR_Pin
      |GV_CLOSE_PULSE_Pin|GV_OPEN_PULSE_Pin|DEBUG_LED_Pin|UV_INDICATE_LED_Pin
      |OV_INDICATE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : _30A_CKT1_LED_Pin _30A_CKT2_LED_Pin _30A_CKT3_LED_Pin _30A_CKT4_LED_Pin
                           _30A_CKT5_LED_Pin _30A_CKT6_LED_Pin */
  GPIO_InitStruct.Pin = _30A_CKT1_LED_Pin|_30A_CKT2_LED_Pin|_30A_CKT3_LED_Pin|_30A_CKT4_LED_Pin
      |_30A_CKT5_LED_Pin|_30A_CKT6_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
