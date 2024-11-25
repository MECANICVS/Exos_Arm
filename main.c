/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* I2C return status for debugging and mode switching */
HAL_StatusTypeDef stat;
char *TX_STAT = "TX";
char *RX_STAT = "RX";

/* Mode selection */
typedef enum {
	BASE, I2C
} MODES;

/* PCA9685 Registers and parameters */
const uint8_t PCA9685_Address = 0x40;
const uint8_t PCA9685_MODE1 = 0x00;
const uint8_t PCA9685_PRESCALE = 0xFE;
const uint8_t MODE1_SLEEP = 0x10;
const uint8_t MODE1_RESTART = 0x80;
const uint8_t MODE1_AI = 0x20;
const uint8_t PIN_0 = 0x06;
const uint32_t CLOCK = 25000000;
uint16_t Desired_frequency = 50;  //frequency selection for PCA9685

/* counters and flag*/
volatile uint16_t Ovf_Cnt = 0;
uint16_t Cnt_led = 0; // for the LEDs
uint8_t Flag_mec = 0; // To trigger Arming()

/* servos' angular position */
uint8_t M1 = 0; //mg995 servo
uint8_t M2 = 70; //Lower Left servo
uint8_t M3 = 150; //Upper Left servo
uint8_t M4 = 0; //Lower Right servo
uint8_t M5 = 0; //Upper Right servo
uint8_t M6 = 0; //M6 = 91   when using fs90r(continous rotation servo)

/* Analog values */
uint16_t volt[2];
volatile uint16_t PR = 0; // Photo resistance value for hand detection
volatile uint16_t Bat_level = 0; // EMG Battery Level
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
static void Dragon_crest(void);
static void prescaler_choice(I2C_HandleTypeDef *hi2c, uint8_t address,
		uint16_t frequency);

static void pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t PIN,
		uint16_t angle);
static uint16_t RegU(uint8_t angle);
static uint16_t RegUD(uint8_t angle);
static void Desarming(void);
static void Arming(void);
static void Rearming(void);
static void Desarming_i2c(void);
static void Arming_i2c(void);
static void Rearming_i2c(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint32_t waitTime = 0;
	uint32_t waitTime2 = 0;
	uint32_t waitTime3 = 0;
	uint32_t elapsedtime = 0;
	uint32_t prev = 0;
	MODES Mode = BASE;
	uint8_t res;
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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	prescaler_choice(&hi2c1, PCA9685_Address, Desired_frequency);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) volt, 2);
	HAL_TIM_Base_Start_IT(&htim9);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	res = memcmp(RX_STAT, "OK_RX", strlen(RX_STAT));
	if (res == 0) {
		Mode = I2C;
		Desarming_i2c();
	} else
		Desarming();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		elapsedtime = HAL_GetTick() - oldtime;
		oldtime += elapsedtime;
		waitTime += elapsedtime;
		waitTime2 += elapsedtime;
		waitTime3 += elapsedtime;
		static uint8_t Flag_rearm = 0;

		Dragon_crest();

		switch (Mode) {
		case BASE:
			if ((waitTime >= 6000) && (PR < 50) && (M1 > 0)) { // Hand detected -> closing
				M1 = 0;

				TIM3->CCR1 = RegUD(M1);
				waitTime = 0;
			}

			if ((waitTime <= 3000) && (M1 == 0) && (Flag_mec == 1)) { // Muscle contracted -> arming
				Arming();

			}

			if ((waitTime > 3000) && (Flag_mec == 0) && (M1 == 0) && (PR < 50)
					&& (Flag_rearm == 0)) { // No contraction -> Reopening
				M1 = 150;

				TIM3->CCR1 = RegUD(M1);
			}

			if ((waitTime2 >= 20000) && (PR >= 80) && (M1 >= 150)) { // No Hand -> Rearming
				Rearming();
				Flag_rearm = 1;
				waitTime2 = 0;
			}

			break;

		case I2C:
			if ((waitTime >= 6000) && (PR < 50) && (M1 > 0)) {
				M1 = 0;
				pwm(&hi2c1, PCA9685_Address, 8, M1);
				waitTime = 0;
			}

			if ((waitTime <= 3000) && (M1 == 0) && (Flag_mec == 1)) {
				Arming_i2c();
			}

			if ((waitTime > 3000) && (Flag_mec == 0) && (M1 == 0) && (PR < 50)
					&& (Flag_rearm == 0)) {
				M1 = 150;
				pwm(&hi2c1, PCA9685_Address, 8, M1);

			}

			if ((waitTime2 >= 20000) && (PR >= 80) && (M1 >= 150)) {
				Rearming_i2c();
				Flag_rearm = 1;
				waitTime2 = 0;
			}

			break;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
			NVIC_SystemReset();
		}
		if (Bat_level < 2000) {
			if (waitTime3 - prev >= 1000) {
				prev = waitTime3;
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); // Blinking Red LED
			}
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 16 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 10000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 1000 - 1;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 800 - 1;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint16_t RegU(uint8_t Angle) {
	return ((((Angle * 0.0055) + 1) / 20) * 10000) - 1;
}
static uint16_t RegUD(uint8_t Angle) {
	return (((Angle * 0.0055) + 1) / 20) * 20000;
}
// I highly recommend to use external pull up resistors besides internal pull up for SDA & SCL
static void prescaler_choice(I2C_HandleTypeDef *hi2c, uint8_t address,
		uint16_t frequency) {
	address = address << 1; // Going from 7 bits address to 8 bits
	uint8_t buffer[2];
	buffer[0] = PCA9685_MODE1;
	uint8_t initPrescaler[2];
	uint8_t oldmode;

	stat = HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 10);
	//stat = HAL_I2C_Master_Transmit(hi2c, address, buffer, 1, 10); // You can try this as well, sometimes it didn't work for me
	if (stat != HAL_OK) {
		TX_STAT = "KO_TX";
	} else {
		TX_STAT = "OK_TX";
		stat = HAL_I2C_Master_Receive(hi2c, address, &oldmode, 1, 10);
		if (stat != HAL_OK) {
			RX_STAT = "KO_RX";
		} else {
			RX_STAT = "OK_RX";

			uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;

			buffer[1] = newmode;

			HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 10);

			uint8_t prescaler = (CLOCK / (frequency * 4096)) - 1;
			initPrescaler[0] = PCA9685_PRESCALE;
			initPrescaler[1] = prescaler;

			HAL_I2C_Master_Transmit(hi2c, address, initPrescaler, 2, 10);

			buffer[1] = oldmode;

			HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 10);
			HAL_Delay(5);

			buffer[1] = (oldmode | MODE1_RESTART | MODE1_AI);

			HAL_I2C_Master_Transmit(hi2c, address, buffer, 2, 10);
		}
	}
}

static void pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t PIN,
		uint16_t angle) {
	address = address << 1; // Going from 7 bits address to 8 bits
	uint16_t off = ((((angle * 0.0055) + 1) / 20) * 4096) - 1;
	uint8_t outputBuffer[5] = { PIN_0 + 4 * PIN, 0, (0 >> 8), off, (off >> 8) };
	HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, 5, 10);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim9) {
		Ovf_Cnt++;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) volt, 2);
	}
}
static void Dragon_crest(void) {

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) { // If muscle contracted
		Flag_mec = 1;
		Cnt_led++;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); // Turn on Green LED
	} else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	if (Ovf_Cnt > 5) {
		if (Cnt_led >= 20) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // Turn on Blue LED for 500ms
			Ovf_Cnt = 0;
			Cnt_led = 0;
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			Ovf_Cnt = 0;
			Cnt_led = 0;
		}
	}
}

static void Desarming(void) {
/////uncomment when using fs90r/////
/*	M6 = 75;
	TIM3->CCR2 = RegUD(M6);
	HAL_Delay(500);
	M6 = 91;     */
	TIM3->CCR2 = RegUD(M6);

	while ((M3 > 0) || (M5 < 150)) {
		TIM1->CCR2 = RegU(M3);
		TIM1->CCR4 = RegU(M5);
		M3 -= 10;
		M5 += 10;
		HAL_Delay(50);
	}
	while ((M2 > 0) || (M4 < 70)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 -= 10;
		M4 += 10;
		HAL_Delay(50);
	}
	while (M1 < 150) {
		TIM3->CCR1 = RegUD(M1);
		M1 += 30;
		HAL_Delay(50);
	}
}

static void Arming(void) {
	/////uncomment when using fs90r/////
/*	M6 = 120;
	TIM3->CCR2 = RegUD(M6);
	HAL_Delay(500);
	M6 = 91;
	TIM3->CCR2 = RegUD(M6);  */
	M6 = 179;
	TIM3->CCR2 = RegUD(M6);

	while ((M2 < 70) || (M4 > 0)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 += 10;
		M4 -= 10;
		HAL_Delay(50);
	}
}

static void Rearming(void) {

	while (M1 > 0) {
		TIM3->CCR1 = RegUD(M1);
		M1 -= 30;
		HAL_Delay(50);
	}
/////uncomment when using fs90r/////
/*	M6 = 120;
	TIM3->CCR2 = RegUD(M6);
	HAL_Delay(500);
	M6 = 91;
	TIM3->CCR2 = RegUD(M6);  */
	M6 = 179;
	TIM3->CCR2 = RegUD(M6);

	while ((M2 < 70) || (M4 > 0)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 += 10;
		M4 -= 10;
		HAL_Delay(50);
	}
	while ((M3 < 150) || (M5 > 0)) {
		TIM1->CCR2 = RegU(M3);
		TIM1->CCR4 = RegU(M5);
		M3 += 10;
		M5 -= 10;
		HAL_Delay(50);
	}
}
//////////////////////////// I2C mode ///////////////////////////////////
static void Desarming_i2c(void) {

/*	M6 = 75;
	pwm(&hi2c1, PCA9685_Address, 13, M6);
	HAL_Delay(500);
	M6 = 91;   */
	pwm(&hi2c1, PCA9685_Address, 13, M6);

	while ((M3 > 0) || (M5 < 150)) {
		pwm(&hi2c1, PCA9685_Address, 10, M3);
		pwm(&hi2c1, PCA9685_Address, 12, M5);
		M3 -= 10;
		M5 += 10;
		HAL_Delay(50);
	}
	while ((M2 > 0) || (M4 < 70)) {
		pwm(&hi2c1, PCA9685_Address, 9, M2);
		pwm(&hi2c1, PCA9685_Address, 11, M4);
		M2 -= 10;
		M4 += 10;
		HAL_Delay(50);
	}
	while (M1 < 150) {
		pwm(&hi2c1, PCA9685_Address, 8, M1);
		M1 += 30;
		HAL_Delay(50);
	}
}
static void Arming_i2c(void) {

/*	M6 = 120;
	pwm(&hi2c1, PCA9685_Address, 13, M6);
	HAL_Delay(500);
	M6 = 91;
	pwm(&hi2c1, PCA9685_Address, 13, M6);  */
	M6 = 179;
	pwm(&hi2c1, PCA9685_Address, 13, M6);

	while ((M2 < 70) || (M4 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 9, M2);
		pwm(&hi2c1, PCA9685_Address, 11, M4);
		M2 += 10;
		M4 -= 10;
		HAL_Delay(50);
	}
}
static void Rearming_i2c(void) {

	while (M1 > 0) {
		pwm(&hi2c1, PCA9685_Address, 8, M1);
		M1 -= 30;
		HAL_Delay(50);
	}
/*	M6 = 120;
	pwm(&hi2c1, PCA9685_Address, 13, M6);
	HAL_Delay(500);
	M6 = 91;
	pwm(&hi2c1, PCA9685_Address, 13, M6); */
	M6 = 179;
	pwm(&hi2c1, PCA9685_Address, 13, M6);

	while ((M2 < 70) || (M4 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 9, M2);
		pwm(&hi2c1, PCA9685_Address, 11, M4);
		M2 += 10;
		M4 -= 10;
		HAL_Delay(50);
	}
	while ((M3 < 150) || (M5 > 0)) {
		pwm(&hi2c1, PCA9685_Address, 10, M3);
		pwm(&hi2c1, PCA9685_Address, 12, M5);
		M3 += 10;
		M5 -= 10;
		HAL_Delay(50);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	PR = (volt[0] * 3300) / 4095;
	Bat_level = (volt[1] * 3300) / 4095;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
