/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* PCA9685 Registers and parameters */
const uint8_t PCA9685_Address = 0x40;
const uint8_t PCA9685_MODE1 = 0x00;
const uint8_t PCA9685_PRESCALE = 0xFE;
const uint8_t MODE1_SLEEP = 0x10;
const uint8_t MODE1_RESTART = 0x80;
const uint8_t MODE1_AI = 0x20;
const uint8_t PIN_0 = 0x06;
uint16_t Desired_frequency = 50;
const uint32_t CLOCK = 25000000;
/* counters */
volatile uint16_t Ovf_Cnt = 0;
uint16_t Cnt_led = 0; // for the LEDs
uint8_t Cnt_mec = 0; // To trigger Arming()
/* servos' angular position */
uint8_t M1 = 0;
uint8_t M2 = 0;
uint8_t M3 = 0;
uint8_t M4 = 0;
uint8_t M5 = 0;
uint8_t M6 = 0;
uint8_t M7 = 0;
/* Analog values */
uint16_t volt[2];
volatile uint16_t PR = 0; // Photoresistance value for arm detection
volatile uint16_t Bat_level = 0; // EMG Battery Level
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void prescaler_choice(I2C_HandleTypeDef *hi2c, uint8_t address,
		uint16_t frequency);
static void pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t PIN,
		uint16_t angle);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
static void Dragon_crest(void);
static void Desarming(void);
static void Arming(void);
static void Rearming(void);
//void Step3(uint32_t delay1, uint32_t delay2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t waitTime = 0;
	uint32_t waitTime2 = 0;
	uint32_t waitTime3 = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	uint32_t oldtime = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	prescaler_choice(&hi2c1, PCA9685_Address, Desired_frequency);
	HAL_TIM_Base_Start_IT(&htim1);
	//Desarming();
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, volt, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/*uint32_t elapsedtime = HAL_GetTick() - oldtime;
		oldtime += elapsedtime;
		waitTime += elapsedtime;
		waitTime2 += elapsedtime;
  		waitTime3 += elapsedtime;*/

		//Dragon_crest();

		/*if ((waitTime >= 5000) & (PR < 480) & (M2 > 0)) { //		fermeture M2
			M2 = 0;
			pwm(&hi2c1, PCA9685_Address, 14, M2);
			waitTime = 0;
		}
		if ((waitTime <= 3000) & (M2 == 0) & (Cnt_mec == 1)) {
			Arming();
		}
		if ((waitTime > 3000) & (Cnt_mec == 0) & (M2 < 60)) {
			M2 = 60;
			pwm(&hi2c1, PCA9685_Address, 14, M2);
		}

		if ((waitTime2 >= 20000) & (PR > 480) & (M2 = 60)) { //
			Rearming();
			waitTime2 = 0;
		}*/

		/*if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) {
		 NVIC_SystemReset();
		 }

		if(Bat_Level < 2000){
			uint8_t Buff[5] = { PIN_0 + 4 * 9, 0, (0 >> 8), (4000 & 0xFF), (4000 >> 8) };
			HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buff, 5, 1000);
			if(waitTime3 >= 1000){
				Buff[3] = 4096 & 0xFF;
				Buff[4] = 4096 >> 8;
				HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buff, 5, 1000);
				waitTime3 = 0;
			}
		}*/

		//////////////// Code for debugging the PCA9685///////////////////////////
		/* uint8_t Buf[5] = { PIN_0 + 4 * 5, 0, (0 >> 8), (4000 & 0xFF),
		 (4000 >> 8) };
		 HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf, 5, 1);
		 HAL_Delay(100);
		 Buf[3] = 4096 & 0xFF;
		 Buf[4] = 4096 >> 8;
		 HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf, 5, 1);
		 HAL_Delay(200);
		 uint8_t Buf2[5] = { PIN_0 + 4 * 8, 0, (0 >> 8), (4000 & 0xFF), (4000
		 >> 8) };
		 HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf2, 5, 1);
		 HAL_Delay(100);
		 Buf2[3] = 4096 & 0xFF;
		 Buf2[4] = 4096 >> 8;
		 HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf2, 5, 1);
		 HAL_Delay(100);*/
		///////////////////////////////////////////////////////////////////////////////////
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void prescaler_choice(I2C_HandleTypeDef *hi2c, uint8_t address,
		uint16_t frequency) {
	address = address << 1;
	uint8_t buffer[2];
	uint8_t oldmode;
	uint8_t initPrescaler[2];
	buffer[0] = PCA9685_MODE1;

	HAL_I2C_Master_Transmit(hi2c, address, buffer, 1, 1000);
	//HAL_I2C_Master_Receive(hi2c, address, &oldmode, 1, 1000);
	oldmode = 0;
	uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;

	buffer[1] = newmode;

	HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 1000);

	uint8_t prescaler = (CLOCK / (frequency * 4096)) - 1;

	initPrescaler[0] = PCA9685_PRESCALE;
	initPrescaler[1] = prescaler;

	HAL_I2C_Master_Transmit(hi2c, address, initPrescaler, 2, 1000);

	buffer[1] = oldmode;

	HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 1000);
	HAL_Delay(5);

	buffer[1] = (oldmode | MODE1_RESTART | MODE1_AI);

	HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 1000);
}
static void pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t PIN,
		uint16_t angle) {
	address = address << 1;
	uint16_t off = ((((angle * 0.0055) + 1) / 20) * 4096) - 1;
	//uint16_t off = ((((angle * 0.011) + 0.5)/20) * 4096) - 1;
	uint8_t outputBuffer[5] = { PIN_0 + 4 * PIN, 0, (0 >> 8), off, (off >> 8) };
	HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, 5, 1000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		Ovf_Cnt++;
	}
}
static void Dragon_crest(void) {
	uint8_t Buf[5] = { PIN_0 + 4 * 5, 0, (0 >> 8), (4000 & 0xFF), (4000 >> 8) };
	uint8_t Buf2[5] = { PIN_0 + 4 * 8, 0, (0 >> 8), (4000 & 0xFF), (4000 >> 8) };

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
		Cnt_mec = 1;
		Cnt_led++;
		HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf, 5, 1000);

	} else {
		Buf[3] = 4096 & 0xFF;
		Buf[4] = 4096 >> 8;
		HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf, 5, 1000);
	}
	if (Ovf_Cnt > 20) {
		if (Cnt_led >= 80) {
			HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf2, 5, 1000);
			Ovf_Cnt = 0;
			Cnt_led = 0;
		} else {
			Buf2[3] = 4096 & 0xFF;
			Buf2[4] = 4096 >> 8;
			HAL_I2C_Master_Transmit(&hi2c1, 0x80, Buf2, 5, 1000);
			Ovf_Cnt = 0;
			Cnt_led = 0;
		}
	}
}
static void Desarming(void) {

	while (M1 < 175) {
		pwm(&hi2c1, PCA9685_Address, 2, M1);
		pwm(&hi2c1, PCA9685_Address, 2, M3);
		M1 += 5;
		M3 += 5;
		HAL_Delay(30);
	}
	while ((M5 > 0) || (M7 < 90)) {
		pwm(&hi2c1, PCA9685_Address, 2, M5);
		pwm(&hi2c1, PCA9685_Address, 2, M7);
		M5 -= 5;
		M7 += 5;
		HAL_Delay(30);
	}
	while ((M4 > 0) || (M6 < 170)) {
		pwm(&hi2c1, PCA9685_Address, 2, M4);
		pwm(&hi2c1, PCA9685_Address, 2, M6);
		M4 -= 5;
		M6 += 5;
		HAL_Delay(30);
	}
	while (M2 < 60) {
		pwm(&hi2c1, PCA9685_Address, 2, M2);
		M2 += 10;
		HAL_Delay(30);
	}
}
static void Arming(void) {
	while (M1 > 175){
		pwm(&hi2c1, PCA9685_Address, 2, M1);
		pwm(&hi2c1, PCA9685_Address, 2, M3);
		M1 -= 5;
		M3 += 5;
		HAL_Delay(30);
	}
	while ((M5 < 170) || (M7 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 2, M5);
		pwm(&hi2c1, PCA9685_Address, 2, M7);
		M5 += 10;
		M7 -= 10;
		HAL_Delay(30);
	}
}
static void Rearming(void) {
	M2 = 0;
	pwm(&hi2c1, PCA9685_Address, 14, M2);
	while (M1 > 0) {
		pwm(&hi2c1, PCA9685_Address, 2, M1);
		pwm(&hi2c1, PCA9685_Address, 2, M3);
		M1 -= 5;
		M3 += 5;
		HAL_Delay(30);
	}
	while ((M5 < 170) || (M7 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 2, M5);
		pwm(&hi2c1, PCA9685_Address, 2, M7);
		M5 += 10;
		M7 -= 10;
		HAL_Delay(30);
	}
	while ((M4 < 170) || (M6 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 2, M4);
		pwm(&hi2c1, PCA9685_Address, 2, M6);
		M4 += 10;
		M6 -= 10;
		HAL_Delay(30);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	PR = (volt[0] * 3300) / 4095;
	Bat_level = (volt[1] * 3300) / 4095;
	HAL_ADC_Start_DMA(&hadc1, volt, 2);
}
/*void Step3(uint32_t delay1, uint32_t delay2) {
 if (delay1 >= 50) {
 if ((M0 > 90) | (M1 < 100)) {
 pwm(&hi2c1, PCA9685_Address, 0, M0);
 pwm(&hi2c1, PCA9685_Address, 1, M1);
 M0 -= 2;
 M1 += 2;
 }
 delay1 = 0;
 }
 if (delay2 >= 50) {
 if ((M2 > 90) | (M3 < 100)) {
 pwm(&hi2c1, PCA9685_Address, 2, M2);
 pwm(&hi2c1, PCA9685_Address, 3, M3);
 M2 -= 2;
 M3 += 2;
 }
 delay2 = 0;
 }
 }*/

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
	while (1) {
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
