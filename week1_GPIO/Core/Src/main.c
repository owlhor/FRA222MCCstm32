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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int a;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*----1-----set pin--------------------*/
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED2 PA5

	  /*----2------Button pin----------------*/
//	  GPIO_PinState BtnVal = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // Button Read P C 13
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, BtnVal); // LED2 PA5

	  /*----3------toggle pin----------------*/
//	  static GPIO_PinState BtnArl[2] = {0};
//	  BtnArl[0] =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // Button Read P C 13
//	  if (BtnArl[1] == GPIO_PIN_SET && BtnArl[0] == GPIO_PIN_RESET)
//	  {
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }
//	  BtnArl[1] = BtnArl[0];

	  /*----4------1KHz button--------------------*/
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  HAL_Delay(500);

	  /*----5------1KHz btn-gettik-------------------*/
//	  static uint32_t timestp = 0;
//	  if(HAL_GetTick() - timestp >= 500){ // 500 means (500,1) -> (500,0)
//		  timestp = HAL_GetTick(); // stamp time
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }

	  /*----6---- 1KHz - 500Hz LED shuffle-------------*/
//	  static GPIO_PinState BtnArl[2] = {0};
//	  static uint32_t freq = 500; //ms
//	  BtnArl[0] =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // Button Read P C 13
//	  if (BtnArl[1] == GPIO_PIN_SET && BtnArl[0] == GPIO_PIN_RESET) // Falling Edge Det.
//	  {
//		  if(freq == 100)
//		  {
//			  freq = 1000;
//		  }
//		  else{
//			  freq = 100;
//		  }
//	  }
//	  BtnArl[1] = BtnArl[0];
//
//	  static uint32_t timestp = 0;
//	  if(HAL_GetTick() - timestp >= freq){ // 500 means (500,1) -> (500,0)
//		  timestp = HAL_GetTick(); // stamp time
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//	  }

	 /*Challenge breathe blink*/
	  // counter circuit
	  static uint32_t timestp = 0;
	  static uint32_t counter = 0;
	  uint8_t period = 4; // period 1 cycle
	  uint8_t duty = 1;
	  if(HAL_GetTick() - timestp >= duty){ //every 1 m sec
		  timestp = HAL_GetTick(); // stamp time
		  if (counter >= period){
			  counter = 0;
		  }
		  else{counter++;}
	  }

	  static GPIO_PinState BtnArl[2] = {0};
	  static uint8_t freq = 0;
	  BtnArl[0] =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); // Button Read P C 13
	  if (BtnArl[1] == GPIO_PIN_SET && BtnArl[0] == GPIO_PIN_RESET) // Falling Edge Det.
	  {
		  if(freq >= period){
			  freq = 0;}
		  else{
			  freq++;
		  }
	  }
	  BtnArl[1] = BtnArl[0];

	  if (counter < freq){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  }
	  else{HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);}




	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // write here ------!!!!!!!!!!!!!
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Btn_1_Pin */
  GPIO_InitStruct.Pin = Btn_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

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

