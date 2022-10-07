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
uint16_t BtnState = 0;
static uint64_t CodeCC = 0;
uint8_t DecoNum; // for debgg
static uint16_t counter = 0;
static uint16_t PeriodOfCnter = 100; // counter trigg& reset
static uint8_t RiseTgr = 0; // std_logic boolean
static uint8_t OkToggle = 0; // std_logic boolean
//////mega challenga
static uint64_t SaveCC = 63340500060;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void BtnMtxRd();
int DecoToNum(); // int
void CounterSet();
int ShiftTest(int x);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	CounterSet();
	BtnMtxRd();
	
	// Add number in CodeCC
	if(RiseTgr == 1 && counter == PeriodOfCnter){
		  DecoNum = DecoToNum(); // debgg
		  if ( 0 <= DecoNum && DecoNum <= 9){  // non number is unallow
			  //CodeCC = (CodeCC << 4) + DecoNum;  // hex algor
			  CodeCC = (CodeCC * 10) + DecoNum;  // dec algor
			  } 
		  RiseTgr = 0;
		}
		
	// OkToggle
	if (ShiftTest(15) == 1){OkToggle = 1;}
	else if (RiseTgr == 1 && ShiftTest(15) == 0){OkToggle = 0;}
	else{OkToggle = OkToggle;}
	
	// detect code	
	if (CodeCC == SaveCC && OkToggle == 1){ 
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  }
	else{HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);}

		// Clear
	if( ShiftTest(3) == 1)
		{ 
		  CodeCC = 0x00;
		}
		// <x-- backspace
	if(ShiftTest(7) == 1 && RiseTgr == 1)
		{ 
		  //CodeCC = CodeCC >> 4 ; // hex algor
		  CodeCC = CodeCC / 10;  // dec algor
		  RiseTgr = 0;
		}
	// Change password	
	if(ShiftTest(12) == 1 && RiseTgr == 1)
		{ 
		  SaveCC = CodeCC;
		  CodeCC = 0x00;
		  RiseTgr = 0;
		}
	} // end while

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L4_Pin|L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : L4_Pin L1_Pin */
  GPIO_InitStruct.Pin = L4_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L2_Pin */
  GPIO_InitStruct.Pin = L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R4_Pin R3_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R4_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L3_Pin */
  GPIO_InitStruct.Pin = L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* ////  4x4 Matrix table numberpad
	7	8	9	Clr	|	0	1	2	3
	4	5	6	<x- |	4	5	6	7
	1	2	3	_	|	8	9	10	11
	_	0	_	ok 	|	12	13	14	15
*/
GPIO_TypeDef* BtnMtxPortR[4] = {R1_GPIO_Port,R2_GPIO_Port,R3_GPIO_Port,R4_GPIO_Port};
uint16_t BtnMtxPinR[4] = {R1_Pin,R2_Pin,R3_Pin,R4_Pin};

GPIO_TypeDef* BtnMtxPortL[4] = {L1_GPIO_Port,L2_GPIO_Port,L3_GPIO_Port,L4_GPIO_Port};
uint16_t BtnMtxPinL[4] = {L1_Pin,L2_Pin,L3_Pin,L4_Pin};
void BtnMtxRd(){// Read 4x4 Matrix button
	static uint32_t timestp = 0;
	static uint8_t CurntL = 0; // L that read L1 L2 L3 L4
	if(HAL_GetTick() - timestp >= 50){ //call every 100 m sec
	 timestp = HAL_GetTick(); // stamp time
	 for(int i = 0; i<4 ; i++){
		 // read R
		 if(HAL_GPIO_ReadPin(BtnMtxPortR[i], BtnMtxPinR[i])==GPIO_PIN_RESET){ // button pressed

			 BtnState |= 1 << (i+(CurntL*4)); //ex i = 3, L = 2 => 0b0000 0100 0000 0000
			 RiseTgr = 1;
		 }
		 else{
			 //// force 0 after unpressed 1 & x = x
			 BtnState &= ~(1 << (i+(CurntL*4)));
			 ////ex i =   0b 0100 0100 0110 0000
			 ////  ~a =   0b 1111 1011 1111 1011 // 0 & x = 0
			 //// ans =   0b 0100 0000 0110 0000
		 }
	 }
	  	 ////set CurrentL  L High//
	 HAL_GPIO_WritePin(BtnMtxPortL[CurntL], BtnMtxPinL[CurntL], GPIO_PIN_SET);
	 uint8_t nextL = (CurntL + 1) % 4; // % 4 prevent overflow
	 //// set next port Low, prepare to be read in next loop (open drain)
	 HAL_GPIO_WritePin(BtnMtxPortL[nextL], BtnMtxPinL[nextL], GPIO_PIN_RESET);
	 CurntL = nextL;
	}
}

int DecoToNum()
{
	for(int j = 0;j < 14; j++){
		int8_t Chkrr =( BtnState >> j )&0x01;

		if(Chkrr == 1){
			switch(j){
			case 0:
				return 7;
			case 1:
				return 8;
			case 2:
				return 9;
			case 4:
				return 4;
			case 5:
				return 5;
			case 6:
				return 6;
			case 8:
				return 1;
			case 9:
				return 2;
			case 10:
				return 3;
			case 13:
			default:
				return 0;
			case 15:// ok-> send wrong digit
				return 555;
			}
		}
	}
}

void CounterSet()
	{
	// counter circuit
	  static uint32_t timestp = 0;
	  //static uint32_t counter = 0;
	  if(HAL_GetTick() - timestp >= 1){ //every 1 m sec
		  timestp = HAL_GetTick(); // stamp time
		  if (counter >= PeriodOfCnter){
			  counter = 0;
		  }
		  else{counter++;}
	  }
	}

int ShiftTest(int x)
{
	return (BtnState >> x) & 0x01;
} 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

