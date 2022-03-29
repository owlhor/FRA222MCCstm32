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
#define CAPTURENUM 16 // sample data array size for velo
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint8_t cno = 0b0,cnock = 0b0,enbounce = 0;    //input code for check

uint8_t SHLD, SERCLK, INHCLK, LATCHER, SERIN;    // for parraregis drive
uint8_t encResbit = 10;
 //     continuous shift /finished shift/ binary position
uint16_t EnBitlog = 0b0, GrayCBit = 0b0, BinPos = 0b0;
// debug
uint32_t timeStampSR =0;
static enum {INIT,SHLDER,SFTER,RESFTER,LATCHING} PRState = INIT;
uint32_t time;
uint64_t timestampve;
static uint8_t SegDigit = 0;

// velo dma input capture
//DMA Buffer
uint32_t capturedata[CAPTURENUM] = { 0 };
//diff time of capture data
int64_t DiffTime[CAPTURENUM-1] = { 0 };
//Mean difftime
float MeanTime =0;
float RoundNum = 0; // num of pulse in 1 min


// velo pulse counter
//float Velo2nd = 0;
//uint8_t fedge[2] = {0}; // RiseFall edge detection
//uint16_t pulseCnt = 0;

uint64_t _micros = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void ParraRegisDriveAbsENC(uint8_t encbit);    //
void GraytoBinario(uint16_t grayx,uint8_t numbit);
void encoderSpeedReaderCycle();
void SpeedShaft();
uint64_t micros();
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &capturedata,
  			CAPTURENUM);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // parraregis driving act as DigitalOUT       // A0 as ser in
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SHLD);  // D2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, SERCLK); // D3
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, INHCLK); // D4

	  time = HAL_GetTick();
	  if(time-timeStampSR > 3)      // don't use 1
	          {
	              timeStampSR = time;           //set new time stamp
	              ParraRegisDriveAbsENC(encResbit);   // 166 x 595 Driver
	              if(PRState == LATCHING){
	            	  GraytoBinario(GrayCBit, encResbit);}
	          }

	  encoderSpeedReaderCycle();
	  SpeedShaft();
	  /*
	  // foolish edge detect Counter
	  fedge[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7); // D9
	  if (fedge[0] - fedge[1] == 1)//rising edge
		{
		  pulseCnt++;

		}
	  fedge[1] = fedge[0]; // shift
	*/

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D3_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : D9_velodetect_Pin */
  GPIO_InitStruct.Pin = D9_velodetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D9_velodetect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ParraRegisDriveAbsENC(uint8_t encbit){   // sn74hc166x595 drive
    //static enum {INIT,SHLDER,SFTER,RESFTER,LATCHING} PRState = INIT;
    //static uint8_t SegDigit = 0; // count digits that serial data sent
    switch (PRState)
    {
        case INIT: // preparation
        default:
        SHLD = 1; //bar disable
        SERCLK = 0;
        INHCLK = 0;
        LATCHER = 0;

        SegDigit = 0;
        EnBitlog = 0b0;
        PRState = SHLDER; // go to latcher after preparation
        break;

        case SHLDER:  //7seg = inp; // get parallel out in i66
        SHLD = 0;     // trig parallel get 1 times
        SERCLK = 0;
        INHCLK = 1;   // use inhibit clk to trig / no distrub ser clk
        LATCHER = 0;

        PRState = SFTER;
        break;

        case SFTER: // send serial
        SHLD = 1;
        INHCLK = 0;
        SERCLK = 1;
        LATCHER = 0;
        // load data
        SERIN = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
        //EnBitlog = (EnBitlog << 1)+ SERIN; // shift first bit 2 left
        EnBitlog = (SERIN << SegDigit)+EnBitlog;// shift first bit from right

        SegDigit++;
        if(SegDigit >= encbit + 1){
            PRState = LATCHING; // finish and reshift next dataset
            }
        else{
            PRState = RESFTER; // down clock to shift next serial digit
            }
        break;

        case RESFTER: // down clock before send next digit
        SHLD = 1;
        SERCLK = 0; // down clock
        INHCLK = 0;
        LATCHER = 0;

        PRState = SFTER;
        break;

        case LATCHING:  // latch for 595
        SHLD = 1;
        SERCLK = 0; // down clock
        INHCLK = 0;
        LATCHER = 1;

        // send complete gray
        // >>1 to cut the first duplicate bit(How tf it comes ?)
        GrayCBit = EnBitlog >> 1;
        PRState = INIT;
        break;

        }//end switch
    }//end void prll

void GraytoBinario(uint16_t grayx,uint8_t numbit){ // numbit=10
//int qd;
//uint16_t gray = 0b0110110100;// gc 01 1011 0100 = 0b 01 0010 0111
uint16_t binaryout = 0b0;

    binaryout = (grayx >> (numbit-1))&0x01;
    //std::cout << binaryout << std::endl;

    for (int i = 1; i < numbit ;i++){
        /* XOR operation */
        int cp1 = binaryout&0x01 ;//>> (numbit-i)
        int cp2 = grayx >> (numbit-(i+1))&0x01; //

        if (cp1 == cp2)
            {binaryout = (binaryout << 1) + 0; } //qd = 0;
        else
            {binaryout = (binaryout << 1) + 1; } //qd = 1;
        //std::cout << "cp" << i << " " << cp1 << cp2 << " " << qd << "  "<<binaryout << std::endl;
    }
    BinPos = binaryout;
    //return binaryout;
}

void encoderSpeedReaderCycle() {
	//get DMA Position form number of data
	uint32_t CapPos =CAPTURENUM -  __HAL_DMA_GET_COUNTER(htim2.hdma[TIM_DMA_ID_CC1]);
	uint32_t sum = 0 ;

	//calculate diff from all buffer except current dma
	for(register int i=2 ;i < CAPTURENUM-1;i++)
	{
		DiffTime[i]  = capturedata[(CapPos+1+i)%CAPTURENUM]-capturedata[(CapPos+i)%CAPTURENUM];
		//Sum all  Diff
		sum += DiffTime[i];
	}

	//mean all Diff
	MeanTime =sum / (float)(CAPTURENUM-3);
}

void SpeedShaft(){

	// Meantime = 1 time period for 1 pulse
	// 60000000 / MeanTime = amount of pulse in 1 minute with that time period
	// 1024 pulse per round

	RoundNum = (60000000.0 / MeanTime)/1024.0; // round per min detect by 1024 clk

	// x pulse detect in 1 msec
	// x/1024 round per 1 x 1000 x 60 min
	/*
	if(micros() - timestampve >= 10000) // 1msec
			{
				timestampve = micros();
				Velo2nd = pulseCnt * (60000.0/2048.0);
				pulseCnt = 0;
			}
	 */
}

// micro timer
uint64_t micros()
{
	return _micros + htim11.Instance->CNT;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim == &htim11)
 {
	 _micros += 65535;
 }
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

