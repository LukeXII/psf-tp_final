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
void delay_us(uint32_t us);
q15_t get_decimal_part(float32_t n);
void transmitNewLine(void);
uint8_t hallarIndiceNota(uint16_t frec);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define	BITS			10
#define N_MUESTRAS  	128.0
#define FREQ_MUESTREO	2000.0
#define POWER_THRESHOLD	6

struct header_struct {
   char		pre[4];
   uint32_t id;
   uint16_t N;
   uint16_t fs ;
   uint32_t maxIndex; 			// indexador de maxima energia por cada fft
   q15_t 	maxValue;	  		// maximo valor de energia del bin por cada fft
   char		pos[4];
} __attribute__ ((packed));

struct header_struct header = {"head", 0, N_MUESTRAS, FREQ_MUESTREO, 0, 0, "tail"};

typedef enum{
	C4,
	Cs4,
	D4,
	Ds4,
	E4,
	F4,
	Fs4,
	S4,
	Ss4,
	A4,
	As4,
	B4
}NOTA;

uint16_t frecNotas[12] = {261,277,293,311,329,349,369,392,415,440,466,493};

char * stringNotasp[] = {
		"C4",
		"Cs4",
		"D4",
		"Ds4",
		"E4",
		"F4",
		"Fs4",
		"S4",
		"Ss4",
		"A4",
		"As4",
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
   uint8_t c, indicePrimeraNota, indiceSegundaNota;
   uint16_t frecPrimeraNota, frecSegundaNota;
   float32_t dummy;

   temp = FREQ_MUESTREO/N_MUESTRAS*1000;

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
//  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  DBG_CyclesCounterInit(CLOCK_SPEED); // Enable the cycle counter

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Reset cycle counter to 0 */
	  DBG_CyclesCounterReset();

//	  uartWriteByteArray ( &huart3, (uint8_t *)&adc[sample]			,sizeof(adc[0]) 	);	 	// envia el sample ANTERIOR
//	  uartWriteByteArray ( &huart3, (uint8_t*)&fftOut[sample*2]		,sizeof(fftOut[0])	); 		// envia la fft del sample ANTERIOR
//	  uartWriteByteArray ( &huart3, (uint8_t*)&fftOut[sample*2+1] 	,sizeof(fftOut[0])	); 		// envia la fft del sample ANTERIOR

	  /* Get the ADC sample */
	  adc[sample] = (((int16_t)ADC_Read(0)-512)>>(10-BITS))<<(6+10-BITS);

	  fftIn[sample] = adc[sample];

	  /* Increment the sample counter and check if we are in the last sample */
	  if ( ++sample == header.N )
	  {

		 /* Reset the samples */
		 sample = 0;

		 arm_rfft_init_q15		   ( &S		,header.N	  , 0				,1				  	); 	// inicializa una estructura que usa la funcion fft para procesar los datos. Notar el /2 para el largo
		 arm_rfft_q15			   ( &S		,fftIn		  , fftOut							  	); 	// por fin.. ejecuta la rfft REAL fft
		 arm_cmplx_mag_squared_q15 ( fftOut ,fftMag		  , header.N/2+1						);
		 arm_max_q15			   ( fftMag ,header.N/2+1 , &header.maxValue, &header.maxIndex 	);


         arm_q15_to_float(&header.maxValue, &dummy, 1);

         if(dummy*10000 >= POWER_THRESHOLD)
         {
        	 fftMag[header.maxIndex] = 0;
        	 fftMag[header.maxIndex - 1] = 0;
        	 fftMag[header.maxIndex + 1] = 0;

        	 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(header.maxIndex, str, 10));

    		 c = '\t';
             HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

             frecPrimeraNota = temp*header.maxIndex/1000;
    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(frecPrimeraNota, str, 10));

    		 c = '\t';
             HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string((uint32_t)(dummy*10000), str, 10));


    		 indicePrimeraNota = hallarIndiceNota(frecPrimeraNota);

    		 c = '\t';
             HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

    		 uartWriteByteArray(&huart3, (uint8_t*)stringNotasp[indicePrimeraNota], 2);
         }

         arm_max_q15( fftMag ,header.N/2+1, &header.maxValue, &header.maxIndex);

		 c = '\t';
         HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

         if(dummy*10000 >= POWER_THRESHOLD)
         {
        	 fftMag[header.maxIndex] = 0;
        	 fftMag[header.maxIndex - 1] = 0;
        	 fftMag[header.maxIndex + 1] = 0;

        	 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(header.maxIndex, str, 10));

    		 c = '\t';
             HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string(temp*header.maxIndex/1000, str, 10));

    		 c = '\t';
             HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);

    		 uartWriteByteArray(&huart3, (uint8_t*)str, uint32_to_string((uint32_t)(dummy*10000), str, 10));

         }

		 transmitNewLine();

		 /* Increment id */
//		 header.id++;

		 /* Send the header in an Array */
//		 uartWriteByteArray (&huart3, (uint8_t*)&header, sizeof(header));

	  }

	  /* Wait until it completes the Cycles. 168.000.000/10.000 = 16.800 cycles */
	  while(DBG_CyclesCounterRead() < CLOCK_SPEED/header.fs);

//	  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

uint8_t hallarIndiceNota(uint16_t frec)
{
	uint8_t indiceNota = 0;

	while( abs(frec - frecNotas[indiceNota]) > 10 )
		indiceNota++;

	return indiceNota;
}

void transmitNewLine(void)
{
	uint8_t c = 10;

	HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
	c = 13;
    HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
}

void delay_us(uint32_t us)
{
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000); // Convierte microsegundos a ciclos de reloj
    uint32_t start = SysTick->VAL;

    while ((SysTick->VAL - start) < ticks) {
        // Espera hasta que el temporizador de conteo (SysTick) alcance el valor deseado
    }
}

uint8_t uint32_to_string(uint32_t value, char *buffer, size_t buffer_size)
{
	uint8_t i, j, index = 0;

	// Verifica si el buffer es suficientemente grande para almacenar la cadena.
    if (buffer_size < 5)
    {
        printf("El buffer no es lo suficientemente grande.\n");
        return 0;
    }

	do {
		buffer[index++] = 48 + (value % 10);
		value /= 10;
	} while (value > 0);

//	 Invertir la cadena para obtener el orden correcto.
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
