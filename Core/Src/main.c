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
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "u8g2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float heartbeat[1250];
int raw[1250];
float segments[num_segments][187];
char buf[64];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static u8g2_t u8g2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(3);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
    break;
  }
  return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
    void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    HAL_SPI_Transmit(&hspi1, (uint8_t *) arg_ptr, arg_int, 10000);
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    break;
  default:
    return 0;
  }
  return 1;
}

void sample_ecg_data(float *ecg_data, uint32_t sampling_rate, uint32_t duration_ms) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

    uint32_t startTime = HAL_GetTick();
    uint32_t sampleTime = 1000 / sampling_rate; // Sampling period in milliseconds
    uint32_t lastSampleTime = startTime;
    uint32_t n = 0;

    while (HAL_GetTick() - startTime < duration_ms) // Continue sampling for the given duration
    {
        if (HAL_GetTick() - lastSampleTime >= sampleTime) // Sample every sampleTime ms
        {
            lastSampleTime = HAL_GetTick();
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            uint32_t rawValue = HAL_ADC_GetValue(&hadc1);
            ecg_data[n] = rawValue * (3.3 / 4096.0);
            n++;
        }
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
}

void normalize(float *arr, uint32_t len) {
  float max_amplitude = 0;
  for (uint32_t i = 0; i < len; i++) {
    if (arr[i] > max_amplitude) {
      max_amplitude = arr[i];
    }
  }

  for (uint32_t i = 0; i < len; i++) {
    arr[i] /= max_amplitude;
  }
}

uint32_t find_r_peaks(float *ecg_data, uint32_t len, int *r_peaks, uint32_t refractory_period) {
    uint32_t r_peak_count = 0;
    uint32_t refractory_counter = 0;

    for (uint32_t i = 0; i < len; i++) {
        if (refractory_counter > 0) {
            refractory_counter--;
            continue;
        }
        if (ecg_data[i] >= 0.90) {
            r_peaks[r_peak_count++] = i;
            refractory_counter = refractory_period;
        }
    }

    return r_peak_count;
}

int compare_ints(const void *a, const void *b) {
    int arg1 = *(const int *)a;
    int arg2 = *(const int *)b;

    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}


float find_median_rr_interval(int *r_peaks, uint32_t len) {
    int rr_intervals[len - 1];

    // Calculate the R-R intervals
    for (uint32_t i = 0; i < len - 1; i++) {
        rr_intervals[i] = r_peaks[i + 1] - r_peaks[i];
    }

    // Sort the R-R intervals in ascending order
    qsort(rr_intervals, len - 1, sizeof(int), compare_ints);

    // Calculate the median
    float median;
    if ((len - 1) % 2 == 0) {
        median = (rr_intervals[(len - 1) / 2] + rr_intervals[(len - 1) / 2 - 1]) / 2.0;
    } else {
        median = rr_intervals[(len - 1) / 2];
    }

    return median;
}


void select_segments(float *heartbeat, int *r_peaks, uint32_t r_peak_count, float T, float segments[][187]) {
    uint32_t segment_length = (uint32_t)(1.2 * T);

    // Loop through each R-peak
    for (uint32_t i = 0; i < r_peak_count; i++) {
        // Get the index of the R-peak
        uint32_t r_peak_index = (uint32_t) r_peaks[i];

        // Copy the segment from the heartbeat array to the segments array
        uint32_t j, k;
        for (j = r_peak_index, k = 0; j < 1250 && k < segment_length; j++, k++) {
            segments[i][k] = heartbeat[j];
        }

        // Pad the remaining part of the segment with zeros until it reaches 187 data points
        for (; k < 187; k++) {
            segments[i][k] = 0;
        }
    }
}


void process_ecg_data(float *heartbeat, uint32_t len, float segments[][187]) {
    // Step 1: Normalize the amplitude values to the range between zero and one
    normalize(heartbeat, len);

    // Step 2: Find the set of ECG R-peak candidates by applying a threshold of 0.9 on the normalized value of the local maximums
    int r_peaks[20];
    uint32_t refractory_period = 25; // Adjust this value as needed, in data points
    uint32_t r_peak_count = find_r_peaks(heartbeat, 1250, r_peaks, refractory_period);

    // Step 3: Find the median of R-R time intervals as the nominal beat period of that window (T)
    float T = find_median_rr_interval(r_peaks, r_peak_count);

    // Step 4: For each R-peak, select a signal part with the length equal to 1.2T. Pad each selected part with zeros to make its length equal to a predefined fixed length(187)
    select_segments(heartbeat, r_peaks, r_peak_count, T, segments);
}

void drawECG(u8g2_t *u8g2) {

	for (int i = 0; i < 127; i++) {

		//u8g2_DrawLine(u8g2,i, 60-(45*segments[1][i]), i, 60-(45*segments[1][i+1])) ;
		u8g2_DrawLine(u8g2,i, 70-(55*heartbeat[i]), i, 70-(55*heartbeat[i+1])) ;
	}
}

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
  u8g2_Setup_sh1106_128x64_noname_1(&u8g2, U8G2_R2, u8x8_byte_4wire_hw_spi,
      u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_ClearDisplay(&u8g2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	u8g2_ClearBuffer(&u8g2);

	sample_ecg_data(heartbeat, 125, 10000);
	process_ecg_data(heartbeat, 1250, segments);
    /* USER CODE END WHILE */
    MX_X_CUBE_AI_Process();

  u8g2_FirstPage(&u8g2);
  	do
  	{
  	  //u8g2_SetFont(&u8g2,u8g_font_profont10);
      u8g2_SetFont(&u8g2,u8g2_font_5x7_tf);
  	  u8g2_DrawStr(&u8g2, 0, 10, buf);
  	  drawECG(&u8g2);

  	} while (u8g2_NextPage(&u8g2));
  	HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|OLED_RES_Pin|OLED_DC_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 OLED_RES_Pin OLED_DC_Pin PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|OLED_RES_Pin|OLED_DC_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
