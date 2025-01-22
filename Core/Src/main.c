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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// This structure is used to conveniently form DMA buffer from RGB values
typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LOW and HIGH values for pixels are easily calculated from the Counter Period (ARR) from CubeMX
#define NEOPIXEL_ZERO 29  // (ARR + 1) * 0.32
#define NEOPIXEL_ONE 58   // (ARR + 1) * 0.64
						  // round everything to the nearest integer
#define NUM_PIXELS 10  // Number of LEDs
#define DMA_BUFF_SIZE (NUM_PIXELS * 24) + 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PixelRGB_t pixel[NUM_PIXELS] = {0};  // Used to store color values of each led
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};  // Apparently DMA buffer needs to be incremental,
										  // so you cant just create a new one on each
										  // function call
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Needed to break pwm cycle (to stop changing leds, because led adresses start acting funny)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
}

// The color is set in range [0, 255], where 0 is the lowest brightness and 255 is the highest
// For example to turn 3th LED red with max brightness you would call set_led(2, 255, 0, 0)
// And to set 5th LED yellow with reduced brightness you might use set_led(4, 100, 100, 0)
// Notice that LED's indexes start from 0, so the 5th led has id = 4
void set_led(int id, int red, int green, int blue)
{
	if (id < 0 || id >= NUM_PIXELS)
	{
		return;  // LED id outside of given range
	}
	if (red < 0 || green < 0 || blue < 0)
	{
		return;  // Color values must be positive
	}
	if (red > 255 || green > 255 || blue > 255)
	{
		return;  // Color values must not exceed 255
	}
	uint32_t *pBuff;
	pixel[id].color.g = green;
	pixel[id].color.r = red;
	pixel[id].color.b = blue;

	pBuff = dmaBuffer;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
	 for (int j = 23; j >= 0; j--)
	 {
	   if ((pixel[i].data >> j) & 0x01)
	   {
		 *pBuff = NEOPIXEL_ONE;
	   }
	   else
	   {
		 *pBuff = NEOPIXEL_ZERO;
	   }
	   pBuff++;
	 }
	}
	dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
}

void clear_leds()
{
	uint32_t *pBuff;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
		pixel[i].color.g = 0;
		pixel[i].color.r = 0;
		pixel[i].color.b = 0;
	}

	pBuff = dmaBuffer;
	for (int i = 0; i < NUM_PIXELS; i++)
	{
	 for (int j = 23; j >= 0; j--)
	 {
	   if ((pixel[i].data >> j) & 0x01)
	   {
		 *pBuff = NEOPIXEL_ONE;
	   }
	   else
	   {
		 *pBuff = NEOPIXEL_ZERO;
	   }
	   pBuff++;
	 }
	}
	dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  clear_leds();
  set_led(8, 100, 100, 0); // Yup, it is that easy
  set_led(9, 100, 100, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
