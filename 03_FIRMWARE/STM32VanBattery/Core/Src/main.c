/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
void setup_pin_states(void);
void LED_blinky(void);
void testFunc(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM2_Init();

  uint32_t adc_values[10];
  uint32_t openCktThreshold = 2100; // 2200 corresponds to 20V
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_values, 10);
  HAL_TIM_Base_Start(&htim2);

  setup_pin_states(); // initialize digital output pin states
  LED_blinky(); //blinky on start up
  // testFunc(); //test function

//  HAL_GPIO_WritePin(GPIOB, GV_OPEN_PULSE_Pin, GPIO_PIN_SET);
//  HAL_Delay(500);
//  HAL_GPIO_WritePin(GPIOB, GV_OPEN_PULSE_Pin, GPIO_PIN_RESET);

  while (1)
  {
    asm("nop"); // no-op for break point

    // blink heart beat LED to indicate FW 'OK'
    HAL_GPIO_WritePin(GPIOB, DEBUG_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOB, DEBUG_LED_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    if(HAL_GPIO_ReadPin(GPIOC, UV_INPUT_Pin) == GPIO_PIN_SET)
    {
//      HAL_GPIO_WritePin(GPIOB, GV_OPEN_PULSE_Pin, GPIO_PIN_SET);
//      HAL_Delay(500);
//      HAL_GPIO_WritePin(GPIOB, GV_OPEN_PULSE_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_SET);
    }
    else
      HAL_GPIO_WritePin(GPIOB, UV_INDICATE_LED_Pin, GPIO_PIN_RESET);

    if(HAL_GPIO_ReadPin(GPIOC, OV_INPUT_Pin) == GPIO_PIN_SET)
    {
      HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_SET);
    }
    else
      HAL_GPIO_WritePin(GPIOB, OV_INDICATE_LED_Pin, GPIO_PIN_RESET);

    // sample and update ANALOG INPUTS
    // fuse status (valid range = 2.036V - 2.655V)
    if (adc_values[0] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT6_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT6_LED_Pin, GPIO_PIN_SET);

    if (adc_values[1] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT5_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT5_LED_Pin, GPIO_PIN_SET);

    if (adc_values[2] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT4_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT4_LED_Pin, GPIO_PIN_SET);

    if (adc_values[3] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT3_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT3_LED_Pin, GPIO_PIN_SET);

    if (adc_values[4] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT2_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT2_LED_Pin, GPIO_PIN_SET);

    if (adc_values[5] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOA, _30A_CKT1_LED_Pin, GPIO_PIN_SET);

    if (adc_values[6] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOB, _60A_CKT2_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOB, _60A_CKT2_LED_Pin, GPIO_PIN_SET);

    if (adc_values[7] > openCktThreshold)
      HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin, GPIO_PIN_RESET);
    else
      HAL_GPIO_WritePin(GPIOB, _60A_CKT1_LED_Pin, GPIO_PIN_SET);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 10;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_192CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_20;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_21;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 33600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 714-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
