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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PPM.hpp"
#include "PWM.hpp"
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
static void setPWMChannel(PWMChannel &pwm, int channel, int percentage);
static uint8_t getPPM(PPMChannel &ppm, int channel);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/* defining channels begin */
	constexpr uint8_t elevator_channel{0};
	constexpr uint8_t left_throttle_channel{1};
	constexpr uint8_t right_throttle_channel{2};
	constexpr uint8_t left_aileron_channel{3};
	constexpr uint8_t right_aileron_channel{4};
	constexpr uint8_t rudder_channel{5};

	constexpr uint8_t elevator_ppm{0};
	constexpr uint8_t throttle_ppm{1};
	constexpr uint8_t aileron_ppm{2};
	constexpr uint8_t rudder_ppm{3};
	/* defining channels end */

	/* mixes */
	uint16_t left_aileron_mix{0};
	uint16_t right_aileron_mix{0};
	uint16_t left_throttle_mix{0};
	uint16_t right_throttle_mix{0};
	uint16_t elevator_mix{0};
	uint16_t rudder_mix{0};
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  PWMChannel pwm;
  PPMChannel ppm;
  ppm.setNumChannels(8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* getting values */

	  // todo: add arming
	  uint8_t elevator_percent = getPPM(ppm, elevator_ppm);
	  uint8_t throttle_percent = getPPM(ppm, throttle_ppm);
	  uint8_t rudder_percent = getPPM(ppm, rudder_ppm);
	  uint8_t roll_percent = getPPM(ppm, aileron_ppm);

	  /* insert mix here */
	  left_aileron_mix = 0 - roll_percent;
	  right_aileron_mix = 0+ roll_percent;
	  elevator_mix = elevator_percent;
	  left_throttle_mix = throttle_percent + 0.3 * rudder_percent;
	  right_throttle_mix = throttle_percent - 0.3 * rudder_percent;
	  rudder_mix = rudder_percent;

	  /* pwm outputs */
	  setPWMChannel(pwm, left_aileron_channel, left_aileron_mix);
	  setPWMChannel(pwm, right_aileron_channel, right_aileron_mix);
	  setPWMChannel(pwm, elevator_channel, elevator_mix);
	  setPWMChannel(pwm, left_throttle_channel, left_throttle_mix);
	  setPWMChannel(pwm, right_throttle_channel, right_throttle_mix);
	  setPWMChannel(pwm, rudder_channel, rudder_mix);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
* Sets an individual PWM channel to the desired output.
*/
static void setPWMChannel(PWMChannel &pwm, int channel, int percentage)
{
    //Input protection
    if(percentage > 100) {percentage = 100;}
    if(percentage < 0) {percentage = 0;}
    pwm.set(channel, percentage);
}

static uint8_t getPPM(PPMChannel &ppm, int channel)
{
    return ppm.get(channel);
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
