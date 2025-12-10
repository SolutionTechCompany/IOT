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
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char rxCmd[50];
uint8_t rxIndex = 0;
uint8_t rxByte;
uint8_t buttonstate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
	uint8_t levelState;        // ค่าปัจจุบันจาก sensor
	uint8_t prevState = 0xFF; // ค่าเดิม (ตั้งให้ไม่เท่ากันเพื่อให้พิมพ์ครั้งแรก)
	uint32_t adcValue;
	char msg[20];

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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// อ่านค่าสถานะ sensor (0 หรือ 1)
		levelState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // เปลี่ยนเป็นพอร์ต/พินของคุณ

		// พิมพ์ค่าที่อ่านได้ตลอด (แบบสั้น ๆ)
		char buffer[10];
		sprintf(buffer, "state: %d\r\n", levelState);
		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100); // ไป ESP32

		// ========== AUTO MODE: sensor ควบคุมวาล์ว เฉพาะตอน buttonstate == 0 ==========
		if (buttonstate == 0) {
			if (levelState != prevState) {
				if (levelState == 1) {
					// (ใช้ logic เดิมของคุณ)
					uint8_t msg[] = "HIGH\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 100);
					HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);

					// Relay OFF (ถ้านิยามคุณเป็นแบบนี้)
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					uint8_t msgOff[] = "Relay = LOW (OFF)\r\n";
					HAL_UART_Transmit(&huart2, msgOff, sizeof(msgOff) - 1, 100);
					HAL_UART_Transmit(&huart1, msgOff, sizeof(msgOff) - 1, 100);
				} else { // levelState == 0
					uint8_t msg[] = "LOW\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 100);
					HAL_UART_Transmit(&huart1, msg, sizeof(msg) - 1, 100);

					// Relay ON
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					uint8_t msgOn[] = "Relay = HIGH (ON)\r\n";
					HAL_UART_Transmit(&huart2, msgOn, sizeof(msgOn) - 1, 100);
					HAL_UART_Transmit(&huart1, msgOn, sizeof(msgOn) - 1, 100);
				}

				prevState = levelState;
			}
		} else {
			// MANUAL MODE (buttonstate == 1)
			// sensor เปลี่ยนก็แค่ update prevState เฉย ๆ ไม่แตะวาล์ว
			if (levelState != prevState) {
				prevState = levelState;
			}
		}

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		uint32_t raw = HAL_ADC_GetValue(&hadc1);

		// 1) ADC -> Vadc (0–3.3V)
		float Vadc = (raw / 4095.0f) * 3.3f;

		// 2) Vadc -> Vsensor (เพราะมีตัวหาร 0.6 จาก divider)
		float Vsensor = Vadc / 0.6f;

		// 3) Vsensor -> pressure (0–0.5 bar)
		float pressure = (Vsensor - 0.5) * (5.0 / 4.0);
		if (pressure < 0.0f)
			pressure = 0.0f;   // ถ้าน้อยกว่า 0 ให้เป็น 0 bar

		// 4) แปลงเป็น mbar*1000 เพื่อ print แบบ int
		uint32_t p_milli = (uint32_t) (pressure * 1000.0f + 0.5f); // เช่น 0.123 bar -> 123

		char msg[50];
		sprintf(msg, "ADC=%lu  P=%lu.%03lu bar\r\n", raw, p_milli / 1000, // ส่วนเต็ม
		p_milli % 1000);       // สามหลักทศนิยม
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 100);

//	          uint32_t val = HAL_ADC_GetValue(&hadc1);
//	          float Vadc = (val / 4095.0f) * 3.3f;
//	          float Vsensor = Vadc / 0.68f;
//	          pressure = (Vsensor - 0.5) * (0.5 / (4.5 - 0.5))
//	          char msg[20];
//	          sprintf(msg, "P = %.2f bar\r\n", pressure_bar);
//	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

//	          char msg[20];
//	          sprintf(msg, "%lu\r\n", val);
//	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);

		HAL_Delay(200);
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
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 LD2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	ADC_ChannelConfTypeDef sConfig;

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rxByte == '\n') {
			rxCmd[rxIndex] = '\0';

			HAL_UART_Transmit(&huart2, (uint8_t*) "CMD: ", 5, 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) rxCmd, strlen(rxCmd), 100);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 100);

			if (strcmp(rxCmd, "CMD_VALVE_ON") == 0) {
				buttonstate = 1;  // เข้า manual mode
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				char *msg = "Valve forced ON by app\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 100);
				uint8_t msgOn[] = "Relay = HIGH (ON)\r\n";
				HAL_UART_Transmit(&huart1, msgOn, sizeof(msgOn) - 1, 100);
			} else if (strcmp(rxCmd, "CMD_VALVE_OFF") == 0) {
				buttonstate = 0;  // กลับไป auto mode
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				char *msg = "Valve forced OFF by app\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 100);
				uint8_t msgOff[] = "Relay = LOW (OFF)\r\n";
				HAL_UART_Transmit(&huart1, msgOff, sizeof(msgOff) - 1, 100);
			}

			rxIndex = 0;
		} else if (rxByte != '\r') {
			if (rxIndex < sizeof(rxCmd) - 1) {
				rxCmd[rxIndex++] = rxByte;
			}
		}

		HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	}
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
