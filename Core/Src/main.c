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

#include "arm_math.h"

uint8_t uint32_to_string(uint32_t value, char *buffer, size_t buffer_size);
q15_t get_decimal_part(float32_t n);
void transmitNewLine(void);
uint8_t hallarIndiceNota(uint16_t frec);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define	BITS					10
#define MUESTRAS_TOMADAS		128.0
#define MUESTRAS_ZERO_PADDING	128.0
#define FREQ_MUESTREO			2000.0
#define POWER_THRESHOLD			2			// 7 sin zero padding, 3 con zero padding

struct header_struct {
   char		pre[4];
   uint32_t id;
   uint16_t N;
   uint16_t fs ;
   uint32_t maxIndex; 			// indexador de maxima energia por cada fft
   q15_t 	maxValue;	  		// maximo valor de energia del bin por cada fft
   char		pos[4];
} __attribute__ ((packed));

struct header_struct header = {"head", 0, MUESTRAS_TOMADAS + MUESTRAS_ZERO_PADDING, FREQ_MUESTREO, 0, 0, "tail"};

uint16_t frecNotas[12] = {261,277,293,311,329,349,369,392,415,440,466,493};
//						  C4  C4# D4  D4# E4  F4  F4# G4  G4# A4  A4# B4
char * stringNotas[] = {
		"C4",
		"C4#",
		"D4",
		"D4#",
		"E4",
		"F4",
		"F4#",
		"G4",
		"G4#",
		"A4",
		"A4#",
		"B4"
};

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


int main(void)
{
  /* USER CODE BEGIN 1 */

   uint16_t sample = 0;
   arm_rfft_instance_q15 S;
   q15_t 	fftIn 	[ header.N	   	];		// guarda copia de samples en Q15 como in para la fft. La fft corrompe los datos de la entrada!
   q15_t 	fftOut	[ header.N *2   ];		// salida de la fft
   q15_t 	fftMag	[ header.N /2+1 ]; 		// magnitud de la FFT
   int16_t 	adc 	[ header.N	   	];
   uint32_t	temp;
   char str[10];
   uint8_t c = '\t', indicePrimeraNota, indiceSegundaNota;
   uint16_t frecPrimeraNota, frecSegundaNota, i;
   float32_t dummy;
   uint8_t newlineflag;

   temp = FREQ_MUESTREO/(MUESTRAS_TOMADAS + MUESTRAS_ZERO_PADDING)*1000.0;

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

  /* USER CODE BEGIN 2 */

  DBG_CyclesCounterInit(CLOCK_SPEED); // Enable the cycle counter

  // Fill with zeros
  for(i = 128;i < header.N;i++)
	  fftIn[i] = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Reset cycle counter to 0 */
	  DBG_CyclesCounterReset();

	  /* Get the ADC sample */
	  adc[sample] = (((int16_t)ADC_Read(0)-512)>>(10-BITS))<<(6+10-BITS);

	  fftIn[sample] = adc[sample];

	  /* Increment the sample counter and check if we are in the last sample */
	  if ( ++sample == (uint16_t)MUESTRAS_TOMADAS )
	  {
		 newlineflag = 0;
		 /* Reset the samples */
		 sample = 0;

		 arm_rfft_init_q15		   ( &S		,header.N	  , 0				,1				  	);
		 arm_rfft_q15			   ( &S		,fftIn		  , fftOut							  	);
		 arm_cmplx_mag_squared_q15 ( fftOut ,fftMag		  , header.N/2+1						);

		 arm_max_q15			   ( fftMag ,header.N/2+1 , &header.maxValue, &header.maxIndex 	);
         arm_q15_to_float(&header.maxValue, &dummy, 1);

    	 fftMag[0] = 0;	fftMag[1] = 0;				// Supress the DC component

         if(dummy*10000 >= POWER_THRESHOLD)
         {
        	 // Supress the fundamental freq.
        	 fftMag[header.maxIndex] = 0;
        	 fftMag[header.maxIndex - 1] = 0;
        	 fftMag[header.maxIndex + 1] = 0;

        	 // Supress the first harmonic
        	 fftMag[2*header.maxIndex] = 0;
        	 fftMag[2*header.maxIndex - 1] = 0;
        	 fftMag[2*header.maxIndex + 1] = 0;

        	 // Imprime el bin del maximo encontrado
//        	 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(header.maxIndex, str, 5));
//        	 HAL_UART_Transmit(&huart3, &c, 1, 1);

    		 uartWriteByteArray(&huart3, (uint8_t*)"Frec.: ", 7);

             frecPrimeraNota = temp*header.maxIndex/1000;
    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(frecPrimeraNota, str, 5));
    		 uartWriteByteArray(&huart3, (uint8_t*)" Hz", 3);

             HAL_UART_Transmit(&huart3, &c, 1, 1);

             // Imprime la potencia del bin
//    		 uartWriteByteArray(&huart3, (uint8_t*)"Pot.: ", 6);
//    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string((uint32_t)(dummy*10000), str, 10));
//           HAL_UART_Transmit(&huart3, &c, 1, 1);

    		 indicePrimeraNota = hallarIndiceNota(frecPrimeraNota);

    		 uartWriteByteArray(&huart3, (uint8_t*)"Nota: ", 6);
    		 uartWriteByteArray(&huart3, (uint8_t*)stringNotas[indicePrimeraNota], 3);

    		 newlineflag = 1;
         }

         arm_max_q15( fftMag ,header.N/2+1, &header.maxValue, &header.maxIndex);
         arm_q15_to_float(&header.maxValue, &dummy, 1);

         if(dummy*10000 >= POWER_THRESHOLD)
         {

             HAL_UART_Transmit(&huart3, &c, 1, 1);
             HAL_UART_Transmit(&huart3, "[]", 2, 1);
             HAL_UART_Transmit(&huart3, &c, 1, 1);

        	 // Imprime el bin del maximo encontrado
//        	 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(header.maxIndex, str, 5));
//           HAL_UART_Transmit(&huart3, &c, 1, 1);

    		 uartWriteByteArray(&huart3, (uint8_t*)"Frec.: ", 7);

             frecSegundaNota = temp*header.maxIndex/1000;
    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(frecSegundaNota, str, 5));
    		 uartWriteByteArray(&huart3, (uint8_t*)" Hz", 3);

             HAL_UART_Transmit(&huart3, &c, 1, 1);

             // Imprime la potencia del bin
//    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string((uint32_t)(dummy*10000), str, 10));

    		 indiceSegundaNota = hallarIndiceNota(frecSegundaNota);

    		 uartWriteByteArray(&huart3, (uint8_t*)"Nota: ", 6);
     		 uartWriteByteArray(&huart3, (uint8_t*)stringNotas[indiceSegundaNota], 3);
         }

         if(newlineflag)
         	 transmitNewLine();

	  }

	  /* Wait until it completes the Cycles. 168.000.000/10.000 = 16.800 cycles */
	  while(DBG_CyclesCounterRead() < CLOCK_SPEED/header.fs);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// Recorre el arreglo de frecuencias buscando el mas cercano al argumento
uint8_t hallarIndiceNota(uint16_t frec)
{
	uint16_t indiceNota, valorNuevo, valorPrevio = 300;

	for(indiceNota = 0;indiceNota < 12;indiceNota++)
	{
		valorNuevo = abs(frec - frecNotas[indiceNota]);

		if(valorNuevo > valorPrevio)
			break;
		else
			valorPrevio = valorNuevo;
	}

	return --indiceNota;
}

void transmitNewLine(void)
{
	uint8_t c = 10;

	HAL_UART_Transmit(&huart3, &c, 1, 1);
	c = 13;
    HAL_UART_Transmit(&huart3, &c, 1, 1);
}

uint8_t uint32_to_string(uint32_t value, char *buffer, size_t buffer_size)
{
	uint8_t i, j, index = 0;

	//	Verifica si el buffer es suficientemente grande para almacenar la cadena
    if (buffer_size < 5)
    {
        printf("El buffer no es lo suficientemente grande.\n");
        return 0;
    }

	do {
		buffer[index++] = 48 + (value % 10);
		value /= 10;
	} while (value > 0);

	//	Invertir la cadena para obtener el orden correcto
	for (i = 0, j = index - 1; i < j; i++, j--) {
		char temp = buffer[i];
		buffer[i] = buffer[j];
		buffer[j] = temp;
	}

	return index;
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
