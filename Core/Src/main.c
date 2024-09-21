/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "oled.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t rx_data[2];
volatile uint8_t uart_receive_flag = 0;
static uint8_t prev_mapped_value = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENCODER_MAX 65535  // Assuming a 16-bit timer
#define ENCODER_MIN 30
#define ENCODER_MAX 99
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 串口接收缓冲区
uint8_t rx_data[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	//启动PWM输出
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	htim1.Instance->CNT = 70;

	HAL_Delay(20); // 单片机启动比OLED上电快,需要延迟等待一下
	OLED_Init(); // 初始化OLED
	// 开启串口中断接收
	HAL_UART_Receive_IT(&huart2, rx_data, 2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	char send_buf[50] = { 0 };
	char send_buf_2[50] = { 0 };
	uint8_t mapped_value;
	uint16_t cnt_encoder = 0;
	uint16_t cnt_encoder_2 = 0;

	while (1) {

//		if (cnt_encoder_2 != __HAL_TIM_GET_COUNTER(&htim3)) {
//			cnt_encoder_2 = __HAL_TIM_GET_COUNTER(&htim3);
////			mapped_value = MAP_MIN + cnt_encoder;
//			sprintf(send_buf_2, "%d", cnt_encoder_2);
//			OLED_PrintASCIIString(2, 2, "output ", &afont24x12, OLED_COLOR_NORMAL);
//			OLED_PrintASCIIString(2, 12, send_buf_2, &afont24x12, OLED_COLOR_NORMAL);
//		}

		uint16_t new_encoder_value = __HAL_TIM_GET_COUNTER(&htim1);
		if (cnt_encoder != new_encoder_value) {
			cnt_encoder = ENCODER_MIN
					+ (new_encoder_value % (ENCODER_MAX - ENCODER_MIN + 1));
			mapped_value = cnt_encoder;  // No need for additional mapping
			sprintf(send_buf, "%d", mapped_value);

			if (mapped_value != prev_mapped_value) {
				HAL_UART_Transmit(&huart2, (uint8_t*) send_buf,
						strlen(send_buf), 10);
				prev_mapped_value = mapped_value;
			}
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			// 配置通道1的占空比，影响电机转速（占空比过低可能导致电机无法启动）
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, mapped_value);
		}
		HAL_Delay(10);

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
// 串口接收完成（收到2个字节）中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART2) {
		uint8_t tens = rx_data[0] - '0';
		uint8_t ones = rx_data[1] - '0';

		uint8_t numeric_value = tens * 10 + ones;
		char send_buf[2];
		sprintf(send_buf, "%d", numeric_value);
		HAL_UART_Transmit(&huart2, (uint8_t*) send_buf, strlen(send_buf), 10);
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 10);

		// Re-enable interrupt receive
		HAL_UART_Receive_IT(&huart2, rx_data, 2);
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
