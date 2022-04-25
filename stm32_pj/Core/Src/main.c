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
uint64_t _micros = 0; //(5) [TIM] microsec
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// (1) [GPIO] Matrix brd
uint16_t BtnState = 0b0; // raw input
uint8_t ShiftPos = 0; // position button pressed
uint16_t CodePos  = 0; // x x x position log data before confirmed
uint16_t TargetOkPos[2] = {0}; // ok confirmed position
uint8_t MtxRdTgr = 0;  // trigger for btn manager
uint32_t BtnMtxtimestp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();  // (5) [TIM] microsec func

// (1) [GPIO] Matrix brd
void BtnMtxRd();
void NumpadManager();
void BMtxShift();
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
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  // (5) [TIM] Start microsec
 	HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // (1) [GPIO] Function
	  BtnMtxRd();
	  NumpadManager();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L2_MP_Pin|L4_MP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L1_MP_GPIO_Port, L1_MP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L1_MP_Pin */
  GPIO_InitStruct.Pin = L1_MP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L1_MP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_MP_Pin R2_MP_Pin R3_MP_Pin */
  GPIO_InitStruct.Pin = R1_MP_Pin|R2_MP_Pin|R3_MP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L2_MP_Pin L4_MP_Pin */
  GPIO_InitStruct.Pin = L2_MP_Pin|L4_MP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////////////////////////// (1) [GPIO]/////////////////////////////////////////////////
/* ////  4x4 Matrix table mapping (use 3x3 only)
 *  hunderd / ten / one
 *      R1  R2  R3  R4
	L1  Uh	Ut	Uo	-	|	0	1	2	-
	L2  Dh	Dt	Do	-   |	3	4	5	-
	L3  -	-	-	-	|	-	-	-	-
	L4  ok	SZ	Cc	- 	|	6	7	8	-
*/
// 3 port input and data stock
//GPIO_TypeDef* BtnMtxPortR[3] = {R1_D3_GPIO_Port, R2_D4_GPIO_Port, R3_D5_GPIO_Port};
//uint16_t BtnMtxPinR[3] = {R1_D3_Pin, R2_D4_Pin, R3_D5_Pin};
// 4 port output and data stock
//GPIO_TypeDef* BtnMtxPortL[3] = {L1_D9_GPIO_Port,L2_D10_GPIO_Port,L4_D11_GPIO_Port};
//uint16_t BtnMtxPinL[3] = {L1_D9_Pin,L2_D10_Pin, L4_D11_Pin};
// changed 2 morpho
GPIO_TypeDef* BtnMtxPortR[3] = {R1_MP_GPIO_Port,R2_MP_GPIO_Port,R3_MP_GPIO_Port};
uint16_t BtnMtxPinR[3] = {R1_MP_Pin,R2_MP_Pin,R3_MP_Pin}; // PB13 PB14 PB15
GPIO_TypeDef* BtnMtxPortL[3] = {L1_MP_GPIO_Port, L2_MP_GPIO_Port, L4_MP_GPIO_Port};
uint16_t BtnMtxPinL[3] = {L1_MP_Pin,L2_MP_Pin,L4_MP_Pin}; // PB12 PA11 PA12

void BtnMtxRd(){// Read 4x4 Matrix button
	static uint8_t CurntL = 0; // L pos that read L1 L2 L3 L4
	static uint8_t nextL = 0;
	if(micros() - BtnMtxtimestp >= 100000){ //call every x millisec = x 000 usec
	 BtnMtxtimestp = micros(); // stamp time
		 for(int i = 0; i < 3 ; i++){
			 // read R
			 if(HAL_GPIO_ReadPin(BtnMtxPortR[i], BtnMtxPinR[i])==GPIO_PIN_RESET){ // button pressed

				 BtnState |= 1 << (i+(CurntL*3)); //ex i = 3, L = 2 => 0b0000 0100 0000 0000
				 MtxRdTgr = 1; // start matrix read and decode trigger
				}
			 else{
				 //// force 0 after unpressed 1 & x = x
				 BtnState &= ~(1 << (i+(CurntL*3)));
				 ////ex i =   0b 0100 0100 0110 0000
				 ////  ~a =   0b 1111 1011 1111 1011 // 0 & x = 0
				 //// ans =   0b 0100 0000 0110 0000
			 }
		 }
	 ////set CurrentL  L High for read//
	 HAL_GPIO_WritePin(BtnMtxPortL[CurntL], BtnMtxPinL[CurntL], GPIO_PIN_SET);
	 //// set next port Low, prepare to be read in next loop (open drain)
	 nextL = (CurntL + 1) % 3; // % 3 prevent overflow
	 HAL_GPIO_WritePin(BtnMtxPortL[nextL], BtnMtxPinL[nextL], GPIO_PIN_RESET);
	 CurntL = nextL;
	}
}
void NumpadManager(){
	if(MtxRdTgr == 1){
		BMtxShift();
		// Codepos = (CPH * 100) + (CPT * 10) + (CPO * 1);
	switch (ShiftPos){
	case 0:
		CodePos += 100; break;
	case 3:
		CodePos -= 100; break;
	case 1:
		CodePos += 10; break;
	case 4:
		CodePos -= 10; break;
	case 2:
		CodePos += 1; break;
	case 5:
		CodePos -= 1; break;

	case 6: // OK
		TargetOkPos[1] = TargetOkPos[0]; // shift old history to 1
		TargetOkPos[0] = CodePos; break; // Get new val

	case 7: // Set Zero
		TargetOkPos[0] = 0; break;
	case 8: // Cancel
		CodePos = 0; break;
	}

	// saturation [0,720]
	if (CodePos > 720){CodePos = 720;}
	if (CodePos < 0){CodePos = 0;}

	//End btnmanager
	MtxRdTgr = 0;
	}
}
void BMtxShift() // shift to the needed position
{
	//return (BtnState >> x) & 0x01;
	for(int i = 0; i < 12 ; i++)
	{
		uint8_t shifflog = (BtnState >> i) & 0x01;
		if (shifflog == 1)
		{
			//return i;
			ShiftPos = i;
			break;
		}
	}
}

////////////////////////// (5) [TIM] Microsec set using TIM11 //////////////////////
uint64_t micros()
{return _micros + htim11.Instance->CNT;}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   // Tim interrupt
	if(htim == &htim11)
 {_micros += 65535;}
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

