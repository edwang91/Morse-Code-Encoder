/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
const uint16_t LONG_SIGN = 500;
const uint16_t SHORT_SIGN = 200;
const uint16_t DELAY = 100;

const uint8_t buff_size = 100;
uint8_t encode_flag = 1;				// Initialize in encode mode
uint8_t overflow[buff_size] = "Overflow has occurred\n\n\r";
uint8_t bufferRx[buff_size];		// Reads one byte at a time from UART input
int rx_index = 0;
volatile uint8_t rx_flag = 0;
volatile uint8_t rx_char;
uint8_t flash_flag = 0;
uint8_t buff[6] = "\n\n\r>>>";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void toggle_LED(void);
void encode(uint8_t* msg);

void signal(uint16_t waitTime);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart2, buff, 6, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive_IT(&huart2, bufferRx, buff_size);
		
		while (rx_flag == 0) {
			// do nothing
		}
		
		if (rx_flag == 1) {
			rx_flag = 0;
			if (rx_char == '\n' || rx_char == '\r') {
				// From the previous message, set all the unused spaces to 0 after the last-updated element
				memset(bufferRx+rx_index, 0, buff_size-rx_index);		
				if (encode_flag == 1) {
					encode(bufferRx);
					rx_index = 0;
					//memset(bufferRx, 0, buff_size);
				} else {
					toggle_LED();
				}
				HAL_UART_Transmit(&huart2, buff, 6, 100);
			} else {
				if (rx_index >= buff_size) {
					HAL_UART_Transmit(&huart2, overflow, buff_size, 100);
					memset(bufferRx, 0, buff_size);
					rx_index = 0;
					HAL_UART_Transmit(&huart2, buff, 6, 100);
				} else {
					bufferRx[rx_index++] = rx_char;
				}
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
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(Green_LED_LD2_GPIO_Port, Green_LED_LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_LD2_Pin */
  GPIO_InitStruct.Pin = Green_LED_LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// User button interrupt callback routine
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin) {
		encode_flag = encode_flag ^ 1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, encode_flag^1);
	}
	else
		__nop();
}

// Toggle LED routine
void toggle_LED(void) {
	HAL_UART_Transmit(&huart2, bufferRx, buff_size, 100);
	if (strcmp(bufferRx, "on") == 0)
		HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 1);
	if (strcmp(bufferRx, "off") == 0)
		HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 0);
	memset(bufferRx, 0, buff_size);
	rx_index = 0;
}

// Function call for a signal - pass the long and short wait times as parameters and do a delay after finished signal
void signal(uint16_t waitTIme) {
	HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 1);
	HAL_Delay(waitTIme);
	HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 0);
	HAL_Delay(DELAY);
}


// Takes a pointer to msg buffer and will blink the message on green LED in morse code
void encode(uint8_t *msg) {
	char toCode;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	HAL_UART_Transmit(&huart2, bufferRx, buff_size, 100);
	for (int i = 0; i < rx_index; i++) {
		toCode = (char) *(msg+i);
		
		switch(toCode) {
			case 'A':
			case 'a':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'B':
			case 'b':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'C':
			case 'c':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'D':
			case 'd':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'E':
			case 'e':
				signal(SHORT_SIGN);
				break;
			
			case 'F':
			case 'f':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'G':
			case 'g':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'H':
			case 'h':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'I':
			case 'i':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'J':
			case 'j':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'K':
			case 'k':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'L':
			case 'l':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'M':
			case 'm':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'N':
			case 'n':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'O':
			case 'o':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'P':
			case 'p':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'Q':
			case 'q':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'R':
			case 'r':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'S':
			case 's':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case 'T':
			case 't':
				signal(LONG_SIGN);
				break;
			
			case 'U':
			case 'u':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'V':
			case 'v':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'W':
			case 'w':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'X':
			case 'x':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'Y':
			case 'y':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case 'Z':
			case 'z':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '0':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case '1':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case '2':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case '3':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			case '4':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case '5':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '6':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '7':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '8':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '9':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '.':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case ',':
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				break;
			
			case ';':
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case ':':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '?':
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				break;
			
			case '!':
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				signal(SHORT_SIGN);
				signal(SHORT_SIGN);
				signal(LONG_SIGN);
				signal(LONG_SIGN);
				break;
			
			default:
				HAL_GPIO_WritePin(GPIOA, Green_LED_LD2_Pin, 0);
				HAL_Delay(1000);
		}
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
