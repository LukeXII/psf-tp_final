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
#include "adc.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "pds.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define N_MUESTRAS  	256
#define FREQ_MUESTREO	10000

/* Header added to the stream */
struct header_struct {
   char     head[4];
   uint32_t id;
   uint16_t N;
   uint16_t fs ;
   char     tail[4];
} header={"head", 0, N_MUESTRAS, FREQ_MUESTREO, "tail"};

typedef uint8_t q7_t;

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

q7_t fixedPointMult(q7_t a, q7_t b)
{
	uint16_t res;

	res = (uint16_t)a*(uint16_t)b;
	res = res >> 7;

	return (q7_t)res;
}

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint16_t sample = 0;
  DBG_CyclesCounterInit(CLOCK_SPEED); // Enable the cycle counter
  int16_t adc [N_MUESTRAS];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  q7_t a, b, c;
	  uint8_t str[10], mask, i;
	  float weight, resDec, dummy;
	  /* Reset cycle counter to 0 */
	  DBG_CyclesCounterReset();

	  a = 0x40;
	  b = 0x23;

	  c = fixedPointMult(a, b);

	  sprintf(str, "%x", c);

	  str[2] = '\t';

	  uartWriteByteArray(&huart3, (uint8_t *)"HEX: ", 5);
	  uartWriteByteArray(&huart3, str, 3);

	  mask = 0b01000000;
	  weight = 0.5;
	  resDec = 0.0;
	  for(i = 0;i < 7;i++)
	  {
		  if(mask & c)
			  resDec += weight;

		  mask /= 2;
		  weight /= 2.0;
	  }

	  str[0] = (uint8_t)(resDec + 48);
	  str[1] = '.';

	  dummy = resDec;
	  for(i = 0;i < 7;i++)
	  {
		  dummy *= 10.0;
		  str[i+2] = (uint8_t)(dummy) + 48;
		  dummy -= (float)(str[i+2] - 48);

	  }

	  uartWriteByteArray(&huart3, (uint8_t *)"DEC: ", 5);
	  uartWriteByteArray(&huart3, str, 9);

	  str[0] = 10;			// CR
	  str[1] = 13;			// LF

	  uartWriteByteArray(&huart3, str, 2);


	  //	  /* Get the ADC sample */
//	  adc[sample] = (int16_t)ADC_Read(0)-512;
//
//	  /* Send the sample in an Array */
//	  uartWriteByteArray(&huart2, (uint8_t* )&adc[sample], sizeof(adc[0]));
//
//	  /* Increment the sample counter and check if we are in the last sample */
//	  if ( ++sample==header.N ) {
//
//		 /* Blinks at fs/N frequency */
//		 gpioToggle (GPIOB,LD1_Pin);
//
//		 /* Reset the samples */
//		 sample = 0;
//
////		 trigger(2);
//
//		 /* Increment id */
//		 header.id++;
//
//		 /* Send the header in an Array */
//		 uartWriteByteArray (&huart2, (uint8_t*)&header, sizeof(header));
//
//		 //ADC_Read(0);
//	  }
	  /* Blinks at fs/2 frequency */
	  gpioToggle (GPIOB,LD3_Pin);

	  /* Wait until it completes the Cycles. 168.000.000/10.000 = 16.800 cycles */
	  while(DBG_CyclesCounterRead() < CLOCK_SPEED/2);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
