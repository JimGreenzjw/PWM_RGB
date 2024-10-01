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
static uint8_t prev_mapped_value = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENCODER_MAX_COUNT 65535  // 假设使用16位定时器

#define GRATING_RESOLUTION 0.001  // 光栅尺分辨率，单位：mm/脉冲，根据实际光栅尺规格调整
#define TIMER_OVERFLOW 65536      // 16位定时器溢出值
#define RX_BUFFER_SIZE 3  // 增加缓冲区大小以容纳更长的数字字符串
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 串口接收缓冲区
uint8_t rx_data[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
float test_pos = 0, amplitude = 200, time = 0;
volatile uint32_t g_sys_time = 0; // 系统时间
uint8_t controlPeriodFlag = 0;	  // 控制周期标志
volatile float goal_pos_loop = 0;
volatile float goal_vel_loop = 0;
volatile float goal_cur_loop; // 目标值
volatile uint32_t raw_count; //编码器计数值
volatile float motor_pos = 0.0f; // 光栅尺位置，单位：mm
volatile float motor_vel = 0.0f; // 光栅尺速度，单位：mm/s

int32_t last_count = 0;
int32_t total_count = 0;
uint8_t reference_detected = 0;
float reference_position = 0.0f;  // S相参考点位置
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
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	__HAL_TIM_SET_COUNTER(&htim3, 65536);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_Delay(20); // 单片机启动比OLED上电快,需要延迟等待一下
	OLED_Init();   // 初始化OLED
	// 开启串口中断接收
	HAL_UART_Receive_IT(&huart2, rx_data, 2);
	servoCtrlInit();
	HAL_Delay(100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
	HAL_Delay(500);
	htim2.Instance->CNT = 0;
	motor_pos = 0.0;
	while (1) {
		if (getControlPeriodFlag()) // 位置更新1000HZ
		{
			time += 0.00785;
//			test_pos = amplitude * sin(time); //正弦位置测试
//			servoSetGoalPos(test_pos);
			goal_pos_loop = fabs(amplitude * sin(time)) + 15;
			setControlPeriodFlag(0);
		}

		servoCtrlLoop(); // 伺服控制
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
		// 直接使用接收到的1字节数据
		uint8_t received_value = rx_data[0];

		// 设置goal_pos_loop
		goal_pos_loop = (float) received_value;

		// 发送确认信息
//        char send_buf[50];
//        sprintf(send_buf, "Goal set to: %.1f (0x%02X)\r\n", goal_pos_loop, received_value);
//        HAL_UART_Transmit(&huart2, (uint8_t*)send_buf, strlen(send_buf), 100);

		// 重新启动接收中断
		HAL_UART_Receive_IT(&huart2, rx_data, 1);
	}
}

uint8_t getControlPeriodFlag(void) {
	return controlPeriodFlag;
}

void setControlPeriodFlag(uint8_t val) {
	controlPeriodFlag = val;
}

static double KalmanFilterSpeed(const double ResrcData, double ProcessNiose_Q,
		double MeasureNoise_R) {
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	static double x_last;
	double x_mid = x_last;
	double x_now;

	static double p_last;
	double p_mid;
	double p_now;

	double kg;

	x_mid = x_last;
	p_mid = p_last + Q;

	kg = p_mid / (p_mid + R);
	x_now = x_mid + kg * (ResrcData - x_mid);
	p_now = (1 - kg) * p_mid;
	p_last = p_now;
	x_last = x_now;

	return x_now;
}

float calculatePosition(uint32_t current_count) {
	int32_t diff = (int32_t) current_count - last_count;

	// 检测方向和计算增量
	if (diff > TIMER_OVERFLOW / 2) {
		diff -= TIMER_OVERFLOW;
	} else if (diff < -TIMER_OVERFLOW / 2) {
		diff += TIMER_OVERFLOW;
	}

	total_count += diff;
	last_count = current_count;

	// 转换为实际位置（mm）
	float position = (float) total_count * GRATING_RESOLUTION;

	return position;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // 假设S相信号连接到TIM2的通道3
			{
		// S相信号检测到
		reference_detected = 1;
		reference_position = calculatePosition(__HAL_TIM_GET_COUNTER(&htim2));
		// 可以选择在这里重置计数或者记录参考点位置
		total_count = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) // Assuming you're using TIM3 for the interrupt
	{
		if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == SET) {
			int16_t pos = (int16_t) motor_pos;  // 将浮点数转换为16位整数
			uint8_t send_buf[2];  // 用于存储16位整数的两个字节

			// 将16位整数拆分为两个字节
			send_buf[0] = (pos >> 8) & 0xFF;  // 高字节
			send_buf[1] = pos & 0xFF;         // 低字节

			// 直接发送两个字节
			HAL_UART_Transmit_IT(&huart2, send_buf, 2);
		}
	} else if (htim->Instance == TIM4) {
		static float last_pos = 0;
		raw_count = __HAL_TIM_GET_COUNTER(&htim3);

		// 计算位置（mm）
		motor_pos = calculatePosition(raw_count);
		// 计算速度（mm/s）并应用卡尔曼滤波
		float delta_pos = motor_pos - last_pos;
		motor_vel = KalmanFilterSpeed(delta_pos / 0.001f, 0.001f, 1.0f);
		last_pos = motor_pos;
		g_sys_time++;
		if (g_sys_time % 10 == 0) // 1ms
			setControlPeriodFlag(1);
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
