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
/*
  //----------------------------------
  // FRA262 Robotics Studio III, 1 DOF
  // G6, OWL'S OFFICE
  // ----------------------------------
  // Pin for
  // - Absolute Encoder read
  //	[Position, Velocity] (State Machine read with OF-STUIII-SEN011 Circuit)
  // - Motor Control
  // 	[PWM, direction] (Using Cytron MD10C)
  // - End Effector I2C
  // - Voltage sense at power bar
   *     X
   * - EMERGENCY Interrupt
   * - I/O Button
   *
  //-----------------------------------   */
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
#define ADDR_EFFT 0x23 // End Effector Addr 0x23 0010 0011
#define ADDR_IOXT 0b01000000 // datasheet p15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
////////////////Abs Encoder State Machine//////////////////////////////////
uint8_t SHLD, SERCLK, INHCLK, SERIN;    // clk for parraregis drive

 //     continuous shift /finished shift/ binary position
uint16_t EnBitlog = 0b0, GrayCBit = 0b0, BinPos = 0b0;
// ParraRegis Shifter
static enum {INIT,SHLDER,SFTER,RESFTER,LATCHING} PRState = INIT;
static uint8_t SegDigit = 0;

//////////////Abs Encoder 10 bit I2C
uint8_t encResbit = 10;
uint32_t timeStampSR =0;
uint8_t RawEnBitA = 0b0, RawEnBitB = 0b0;
uint16_t GrayCBitx = 0b0, GrayCBitXI = 0b0, BinPosx = 0b0, BinPosXI = 0b0;
static uint8_t flag_absenc = 0;
////////////// velo dma input capture////////////////////
//DMA Buffer
uint32_t capturedata[CAPTURENUM] = { 0 };
//diff time of capture data
int64_t DiffTime[CAPTURENUM-1] = { 0 };
uint64_t timestampve;
//Mean difftime
float MeanTime =0;
float RoundNum = 0; // num of pulse in 1 min


////////////PWM test/////////////////////////////////////////
uint16_t PWMOut = 3000; // dytycycle = x/10000 % ,TIM4 PB6
uint32_t TimeLoopPWM = 0;
uint64_t _micros = 0;

////////////End Effector////////////////////////////////////
// ADDR 0x23 ACK Register 0x45 : Laser On
// ADDR 0x23 ACK Register 0x23 : Laser data read request
// ADDR 0x23 ACK Register ____ : Laser read (request before every read)

// Regis 0x12 : Start (1sec)
// Regis 0x34 : Work (31sec)
// Regis 0x56 : Stop (1sec)
// Regis 0x78 : wait new (- sec)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void ParraRegisDriveAbsENC(uint8_t encbit);    //
uint16_t GraytoBinario(uint16_t grayx,uint8_t numbit);
void encoderSpeedReaderCycle();

void IOExpenderInit();
void AbsEncI2CRead(uint8_t *RawRA, uint8_t *RawRB);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start_IT(&htim11); // micro timer
    HAL_TIM_Base_Start(&htim2); // Speed
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &capturedata,
    			CAPTURENUM);

    //PWM Test
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    //MCP23017 setting init
    HAL_Delay(100);
    IOExpenderInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // parraregis driving act as DigitalOUT       // A0 as ser in


	  	  if (micros()-timeStampSR > 3000)      // don't use 1
	  	          {
	  	              timeStampSR = micros();           //set new time stamp
	  	              flag_absenc = 1;
	  	          }

	  	  AbsEncI2CRead(&RawEnBitA,&RawEnBitB);
	  	  encoderSpeedReaderCycle();

	  	  ///////////////////// 2KHz change PWM PB6////////////////////
	  	  if(micros() - TimeLoopPWM > 500){
	  	  		  TimeLoopPWM = micros(); // stamp
	  	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWMOut);
	  	  		  //ADC_Target = ADCFeedx[1].datt;
	  	  	  }
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Mot_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, En_SHLD_Pin|En_SERCLK_Pin|En_INHCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Mot_dir_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Mot_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pwr_Sense_Pin */
  GPIO_InitStruct.Pin = Pwr_Sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pwr_Sense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : En_SHLD_Pin En_SERCLK_Pin En_INHCLK_Pin */
  GPIO_InitStruct.Pin = En_SHLD_Pin|En_SERCLK_Pin|En_INHCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI10_Emer_Pin */
  GPIO_InitStruct.Pin = EXTI10_Emer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI10_Emer_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////////// Absolute Encoder ////////////////////////////////////////////
void ParraRegisDriveAbsENC(uint8_t encbit){

    switch (PRState)
    {
        case INIT: // preparation
        default:
        SHLD = 1; //bar disable
        SERCLK = 0;
        INHCLK = 0;
        //LATCHER = 0;

        SegDigit = 0;
        EnBitlog = 0b0;
        PRState = SHLDER; // go to latcher after preparation
        break;

        case SHLDER:  //7seg = inp; // get parallel out in i66
        SHLD = 0;     // trig parallel get 1 times
        SERCLK = 0;
        INHCLK = 1;   // use inhibit clk to trig / no distrub ser clk


        PRState = SFTER;
        break;

        case SFTER: // send serial
        SHLD = 1;
        INHCLK = 0;
        SERCLK = 1;

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
        //LATCHER = 0;

        PRState = SFTER;
        break;

        case LATCHING:  // confirm data
        SHLD = 1;
        SERCLK = 0; // down clock
        INHCLK = 0;
        //LATCHER = 1;

        // send complete gray
        // >>1 to cut the first duplicate bit(How tf it comes ?)
        GrayCBit = EnBitlog >> 1;
        PRState = INIT;
        break;

        }//end switch
    }//end void prll

uint16_t GraytoBinario(uint16_t grayx,uint8_t numbit){ // numbit=10

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
    //BinPos = binaryout;
    return binaryout;
}

void encoderSpeedReaderCycle() {
	//
	//
	//
	//
	// re code using position dif time
	//
	//
	//
	//
	//
	//
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
	// Meantime = 1 time period for 1 pulse
	// 60000000 / MeanTime = amount of pulse in 1 minute with that time period
	// 1024 pulse per round

	RoundNum = (60000000.0 / MeanTime)/1024.0; // round per min detect by 1024 clk

}
/////////////////////Abs Encoder I2C////////////////////////////////////////////
void IOExpenderInit() {// call when start system
	//Init All// setting from datasheet p17 table 3.5
	static uint8_t Setting[0x16] = {
			0b11111111, // IODIRA 1 = in/2=0ut
			0b11111111, // IODIRB
			0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, // GPPUA
			0b00111111, // GPPUB Pull up 100k R
			0x00, 0x00, 0x00, 0x00,
			0x00, // 0x12 GPIOA
			0x00, // 0x13 GPIOB
			0x00, 0x00 };
	// OLATB -> Out data for pinB
	HAL_I2C_Mem_Write(&hi2c1, ADDR_IOXT, 0x00, I2C_MEMADD_SIZE_8BIT, Setting,
			0x16, 100);
}

void AbsEncI2CRead(uint8_t *RawRA, uint8_t *RawRB){

	if(flag_absenc != 0 && hi2c1.State == HAL_I2C_STATE_READY){
		//static uint8_t absencStep = 0;
		switch(flag_absenc){
		default:
			break;

		case 1:
			//HAL_I2C_Master_Receive(&hi2c1, ADDR_IOXT, GrayCBitx, 1, 100);
			//HAL_I2C_Master_Seq_Receive_DMA(hi2c, DevAddress, pData, Size, XferOptions);
			HAL_I2C_Mem_Read(&hi2c1, ADDR_IOXT, 0x12, I2C_MEMADD_SIZE_8BIT,
						RawRA, 1, 100);
			flag_absenc = 2;
		break;

		case 2:
			HAL_I2C_Mem_Read(&hi2c1, ADDR_IOXT, 0x13, I2C_MEMADD_SIZE_8BIT,
								RawRB, 1, 100);
			flag_absenc = 3;
		break;

		case 3:
			GrayCBitx = (RawEnBitB << 8) | RawEnBitA;
			//GrayCBitXI = ~GrayCBitx - 0b1111110000000000; // invert and clear 6 high
			GrayCBitXI = ~GrayCBitx & 0b0000001111111111;
			//BinPosx = GraytoBinario(GrayCBitx, 10);
			BinPosXI = GraytoBinario(GrayCBitXI, 10);
			flag_absenc = 0;
		break;
		}




	}
}
///////////////////////////////////// micro timer////////////////////////////////////
uint64_t micros()
{return _micros + htim11.Instance->CNT;}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
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

