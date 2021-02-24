/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "remote_controller.h"
#include "mpu6050.h"
#include "gps.h"
#include "oled.h"
#include "pid.h"
#include "utils.h"
#include "logger.h"
#include "lpf.h"
#include <stdio.h>
#include <math.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t motor_speed[] = { 1000, 1000, 1000, 1000 };
char log_buffer[512];

static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void) {
	return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
			== (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void) {
	/* Ensure COUNTFLAG is reset by reading SysTick control and status register */
	LL_SYSTICK_IsActiveCounterFlag();
	uint32_t m = HAL_GetTick();
	const uint32_t tms = SysTick->LOAD + 1;
	__IO uint32_t u = tms - SysTick->VAL;
	if (LL_SYSTICK_IsActiveCounterFlag()) {
		m = HAL_GetTick();
		u = tms - SysTick->VAL;
	}
	return (m * 1000 + (u * 1000) / tms);
}
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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	motor_init();
	HAL_Delay(5000);

	uint8_t init_res = mpu6050_init();

	if (init_res != HAL_OK) {
		led_flash_forever(1);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	logger_reset();
	uint32_t logger_index = 0;
	uint32_t prev_us = getCurrentMicros();

	FirstOrderLPF pitch_lpf, roll_lpf, yaw_lpf;
	lpf_first_order_init(&pitch_lpf);
	lpf_first_order_init(&roll_lpf);
	lpf_first_order_init(&yaw_lpf);
	mpu6050_attitude_reset();

	uint32_t unlock_cnt = 0;

	float target_yaw = 0.0f;

	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		remote_update();
		RCInfo rc_info = remoter_get_info();

		if (rc_info.channel[4] < 1500) {
			// LOCKED
			motor_speed[0] = motor_speed[1] = motor_speed[2] = motor_speed[3] = 1000;
			motor_set_speeds(motor_speed);
			pid_init();
			lpf_first_order_init(&pitch_lpf);
			lpf_first_order_init(&roll_lpf);
			lpf_first_order_init(&yaw_lpf);
			mpu6050_attitude_reset();
			unlock_cnt = 0;

			target_yaw = 0.0f;
		} else {
			// UNLOCK
			const uint32_t UNLOCK_DELAY = 500;
			// SAFETY CONSIDERATION
			unlock_cnt += 1;
			if (unlock_cnt < UNLOCK_DELAY) { // delay 1s
				motor_speed[0] = motor_speed[1] = motor_speed[2] = motor_speed[3] = 1000;
				motor_set_speeds(motor_speed);
				continue;
			} else {
				unlock_cnt = UNLOCK_DELAY;
			}
			// UNLOCK THE FLIGHT

			uint32_t now_us = getCurrentMicros();
			// the control loop runs on 500 Hz
			if (now_us - prev_us < 2000) {
				continue;
			}

			Attitude now_attitude = mpu6050_attitude_estimate();

			now_attitude.pitch = lpf_first_order_get_value(&pitch_lpf, now_attitude.pitch);
			now_attitude.roll = lpf_first_order_get_value(&roll_lpf, now_attitude.roll);
			now_attitude.yaw = lpf_first_order_get_value(&yaw_lpf, now_attitude.yaw);

			RCAttitude target_attitude = remote_get_attitude(&rc_info);

			// set base throttle
			for (int i = 0; i < 4; ++i) {
				motor_speed[i] = rc_info.channel[2];
			}

			// update pitch_control
			float pitch_control = pid_update_pitch(now_attitude.pitch, target_attitude.pitch);
			int32_t pitch_control_input = roundf(pitch_control);
			motor_speed[1] += pitch_control_input;
			motor_speed[3] += pitch_control_input;
			motor_speed[0] -= pitch_control_input;
			motor_speed[2] -= pitch_control_input;

			// update roll_control
			float roll_control = pid_update_roll(now_attitude.roll, target_attitude.roll);
			int32_t roll_control_input = roundf(roll_control);
			motor_speed[2] += roll_control_input;
			motor_speed[3] += roll_control_input;
			motor_speed[0] -= roll_control_input;
			motor_speed[1] -= roll_control_input;

			// update yaw_control
			target_yaw -= target_attitude.yaw * 2e-3;
			float yaw_control = pid_update_yaw(now_attitude.yaw, target_yaw);
			int32_t yaw_control_input = roundf(yaw_control);

			motor_speed[0] += yaw_control_input;
			motor_speed[3] += yaw_control_input;
			motor_speed[1] -= yaw_control_input;
			motor_speed[2] -= yaw_control_input;

			for (int i = 0; i < 4; ++i) {
				motor_speed[i] = clip(motor_speed[i], 1000, 2000);
				if (motor_speed[i] < 1050) motor_speed[i] = 1000;
			}
			motor_set_speeds(motor_speed);

			prev_us = now_us;
			// logger
			logger_index += 1;
			if (logger_index == 20) {
//				sprintf(log_buffer,
//						"%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", now_us,
//						g_pitch_p_out, g_pitch_i_out, g_pitch_d_out,
//						g_roll_p_put, g_roll_i_out, g_roll_d_out,
//						g_yaw_p_out, g_yaw_i_out, g_yaw_d_out);
				sprintf(log_buffer, "%lu,%lu,%lu,%lu,%lu,%f,%f,%f,%f,%f,%f\n",
						now_us,
						motor_speed[0],
						motor_speed[1],
						motor_speed[2],
						motor_speed[3],
						now_attitude.pitch,
						now_attitude.roll,
						now_attitude.yaw,
						target_attitude.pitch,
						target_attitude.roll,
						target_yaw);
				logger_log(log_buffer);
				logger_index = 0;
			}
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
