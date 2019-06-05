/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2019 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MEASURE 0x0A
#define MEAN	0x0B
#define STORE	0x0C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

osThreadId SupervisorTaskHandle;
osThreadId ExecTaskHandle;
osThreadId ADCTaskHandle;
osMessageQId InternalProtocolHandle;
osMutexId MemoryMutexHandle;
osSemaphoreId SemaphoreAHandle;
osSemaphoreId SemaphoreBHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartSupervisorTask(void const * argument);
void startExecTask(void const * argument);
void StartADCTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t CMD = 0x0A;
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
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of MemoryMutex */
	osMutexDef(MemoryMutex);
	MemoryMutexHandle = osMutexCreate(osMutex(MemoryMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of SemaphoreA */
	osSemaphoreDef(SemaphoreA);
	SemaphoreAHandle = osSemaphoreCreate(osSemaphore(SemaphoreA), 1);

	/* definition and creation of SemaphoreB */
	osSemaphoreDef(SemaphoreB);
	SemaphoreBHandle = osSemaphoreCreate(osSemaphore(SemaphoreB), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of SupervisorTask */
	osThreadDef(SupervisorTask, StartSupervisorTask, osPriorityNormal, 0, 512);
	SupervisorTaskHandle = osThreadCreate(osThread(SupervisorTask), NULL);

	/* definition and creation of ExecTask */
	osThreadDef(ExecTask, startExecTask, osPriorityIdle, 0, 512);
	ExecTaskHandle = osThreadCreate(osThread(ExecTask), NULL);

	/* definition and creation of ADCTask */
	osThreadDef(ADCTask, StartADCTask, osPriorityIdle, 0, 128);
	ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of InternalProtocol */
	osMessageQDef(InternalProtocol, 32, uint16_t);
	InternalProtocolHandle = osMessageCreate(osMessageQ(InternalProtocol), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */


	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

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

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
			|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
			|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSupervisorTask */
/**
 * @brief  Function implementing the SupervisorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSupervisorTask */
void StartSupervisorTask(void const * argument)
{
	CMD = MEASURE;
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1000);
		if ((xSemaphoreTake(SemaphoreAHandle, 1000/ portTICK_RATE_MS)) == pdTRUE )
		{
			for(int j = 0; j<4; j++)
			{
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
				vTaskDelay( 200 / portTICK_RATE_MS );
			}
			xSemaphoreGive(SemaphoreAHandle);
			vTaskDelay( 200 / portTICK_RATE_MS );
		}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startExecTask */
/**
 * @brief Function im`plementing the ExecTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startExecTask */
void startExecTask(void const * argument)
{
	/* USER CODE BEGIN startExecTask */
	/* Infinite loop */
	for(;;)
	{
		/* Uzycie semafora binarnego */
		/* portTICK_RATE_MS == 1 */
		/* Jezeli semafor jest wolny to wtedy przejdz dalej do wykonywania w¹tku */
		if ((xSemaphoreTake(SemaphoreAHandle, 1000/ portTICK_RATE_MS)) == pdTRUE )
		{
			/* Pomiar ADC */
			if(CMD == MEASURE)
			{
				CMD = MEAN;
				for(int j = 0; j<4; j++) {
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
					vTaskDelay( 300 / portTICK_RATE_MS );
				}
				/* zwalnianie semafora */
				xSemaphoreGive(SemaphoreAHandle);
			} else if(CMD == MEAN) {			/* Liczenie sredniej */
				CMD = STORE;
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
				vTaskDelay( 1000 / portTICK_RATE_MS );
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
				/* zwalnianie semafora */
				xSemaphoreGive(SemaphoreAHandle);
			} else if(CMD == STORE){		/* Zapis do pamieci */
				CMD = MEASURE;
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				vTaskDelay( 1000 / portTICK_RATE_MS );
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
				/* zwalnianie semafora */
				xSemaphoreGive(SemaphoreAHandle);
			}
		}

		/*		int i = 0;
		while(i < 1000000) {
			i++;
		}
		 */
	}


}
/* USER CODE END startExecTask */

/* USER CODE BEGIN Header_StartADCTask */
/**
 * @brief Function implementing the ADCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
	/* USER CODE BEGIN StartADCTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartADCTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM15 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM15) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
