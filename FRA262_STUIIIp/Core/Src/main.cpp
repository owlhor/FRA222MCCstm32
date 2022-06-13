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
#define POSOFFSET  -442 // angle zero offset abs enc
#define ADDR_EFFT 0b01000110 // End Effector Addr 0x23 0010 0011
#define ADDR_IOXT 0b01000000 // datasheet p15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
///////////////////////////////////////////////////////////////////////////
/////////////////Grand State///////////////////////////////////////////////
static enum {Ready, work, stop, emer , stopnd} grandState = Ready;
uint8_t pwr_sense = 0;
uint8_t stop_sense = 0;
uint32_t TimeStampGrand = 0;
uint64_t _micros = 0;

uint8_t counter_e = 0;

//////////////Abs Encoder 10 bit I2C /////////////////////////////
//uint8_t encResbit = 10;
uint32_t timeStampSR =0;
//uint8_t RawEnBitA = 0b0, RawEnBitB = 0b0;
uint8_t RawEnBitAB[2] = {0b0};
uint16_t GrayCBitXI = 0b0, BinPosXI = 0b0;
static uint8_t flag_absenc = 0;
////////////// velo dma input capture////////////////////
//DMA Buffer
uint32_t capturedata[CAPTURENUM] = { 0 };
//diff time of capture data
int64_t DiffTime[CAPTURENUM-1] = { 0 };

//Mean difftime
float MeanTime =0;
float RoundNum = 0; // num of pulse in 1 min

uint16_t posSpeedlog[3] = {0};
float speedsmoothlog[CAPTURENUM] = {0};
float deltaar = 0;
float RoundNumnd = 0;
float RoundNumnd_sm = 0;
uint64_t timestampve = 0;
////////////PWM & Motor Driver/////////////////////////////////////////
uint16_t PWMOut = 5000; // dytycycle = x/10000 % ,TIM4 PB6
uint32_t timestampPWM = 0;
uint8_t mot_dirctn = 0;

///////// PID cat cat /////////////////////////////////////////////
uint8_t bluecounter = 0;

float TargetDeg = 0; // target degree pid
float ErrPos[2] = {0};  // error
float u_contr = 0;
float PreviTime = 0; // find delta T
float DeltaTime = 0;
float CrrntTime = 0;
uint8_t ErrPosx = 0;

float K_P = 15;
float K_I = 0.02;
float K_D = 0;

float Propo;
float Integral;
float Derivate;

////////////End Effector////////////////////////////////////
// ADDR 0x23 ACK Register 0x45 : Laser On
// ADDR 0x23 ACK Register 0x23 : Laser data read request
// ADDR 0x23 ACK Register ____ : Laser read (request before every read)

// Regis 0x12 : Start (1sec)
// Regis 0x34 : Work (31sec)
// Regis 0x56 : Stop (1sec)
// Regis 0x78 : wait new (- sec)
//static enum {ef_INIT,ef_addr,ef_addr_wait,} efftState = ef_INIT;
//uint8_t addr_effct = ADDR_EFFT;
uint8_t efft_status = 0;
uint8_t flag_efftActi = 0;
uint8_t flag_efftRead = 0;
uint8_t trig_efftRead = 0;
uint64_t timestamp_efft = 0;
//static enum {ef_INIT,ef_addreq,ef_addreq_wait,ef_data, ef_data_wait} efrdState = ef_INIT;
//uint16_t counter_efft = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void IOExpenderInit();
uint16_t GraytoBinario(uint16_t grayx,uint8_t numbit);
void AbsEncI2CRead(uint8_t *RawRA, uint8_t *RawRB);
void AbsEncI2CReadx(uint8_t *RawRAB);
void encoderSpeedReaderCycle();

void GrandStatumix();
void Speedsmoothfunc(float inpdat);
void PIDzero();
void MotDrvCytron();
void Efft_activate();
//void Efft_activate(uint8_t devADDR, uint8_t *cmmd);
void Efft_read(uint8_t *Rddata);
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
  MX_TIM11_Init();
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

	  ///// GrandState ///////////////////
	  	  	  //if(micros() - TimeStampGrand >= 1000){
	  	  		//TimeStampGrand = micros();

	  	  	  //}
	  	  	  // Encoder I2CRead
	  	  	  if (micros()-timeStampSR >= 1000)      // don't use 1
	  	  	          {
	  	  	              timeStampSR = micros();           //set new time stamp
	  	  	              flag_absenc = 1;
	  	  	              GrandStatumix();
	  	  	          }
	  	  	  AbsEncI2CReadx(RawEnBitAB);
	  	  	  encoderSpeedReaderCycle();
	  	  	  pwr_sense = HAL_GPIO_ReadPin(Pwr_Sense_GPIO_Port, Pwr_Sense_Pin);

	  	  	  ///////////////////////// speed measyre////////
	  	  	if(micros() - timestampve >= 10000){
	  	  			  timestampve = micros();
	  	  			  posSpeedlog[1] = posSpeedlog[0];
	  	  			  posSpeedlog[0] = BinPosXI;
	  	  			  deltaar = (fabsf(posSpeedlog[1]-posSpeedlog[0])) / 1024.0;
	  	  			  RoundNumnd = deltaar*100.0*60.0;
	  	  			  Speedsmoothfunc(RoundNumnd);
	  	  		 }

	  	  	  ///////////////////// 2KHz change PWM PB6////////////////////
	  	  	  if(micros() - timestampPWM >= 500){
	  	  		  	  timestampPWM = micros(); // stamp
	  	  	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWMOut);
	  	  	  		  //ADC_Target = ADCFeedx[1].datt;
	  	  	  		//HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, mot_dirctn);
	  	  	  		if (mot_dirctn == 0){
	  	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_RESET);
	  	  	  		}else{
	  	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_SET);
	  	  	  		}
	  	  	  	  }

	  	  	 if (grandState ==  work){
	  	  		 PIDzero();
	  	  		 MotDrvCytron();}

	  	  	 //////////// End Effector /////////////////////
	  	  	 Efft_activate(); // Activate by flag_efftActi = 1;
	  	  	 Efft_read(&efft_status);
	  	  	 // trig_efftRead up for 10 times afrer shoot / trig at shoot state
	  	  	 if(trig_efftRead != 0 && micros() - timestamp_efft >= 500000){
	  	  		 timestamp_efft = micros();
	  	  		 flag_efftRead = 1;
	  	  		 trig_efftRead++;
	  	  	 }if(trig_efftRead >= 12){trig_efftRead = 0;} // read xx times


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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

}

/* USER CODE BEGIN 4 */
////////// Grand State //////////////////////////////////////////////////////
void GrandStatumix(){
	if (grandState != Ready){
		HAL_GPIO_WritePin(PLamp_Green_GPIO_Port, PLamp_Green_Pin, GPIO_PIN_RESET);
	}
	if (grandState != work){
			HAL_GPIO_WritePin(PLamp_Blue_GPIO_Port, PLamp_Blue_Pin, GPIO_PIN_RESET);
		}
	if (grandState != stop || grandState != stopnd){
				HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_RESET);
			}

	stop_sense = HAL_GPIO_ReadPin(Stop_Sense_GPIO_Port, Stop_Sense_Pin);


	switch(grandState){
	default:
	case Ready:
		HAL_GPIO_WritePin(PLamp_Green_GPIO_Port, PLamp_Green_Pin, GPIO_PIN_SET);
		PWMOut = 1200;
		if (pwr_sense == 1){grandState = emer;}
		if (stop_sense == 0){grandState = stop;}
		if (bluecounter != 0){grandState = work;} // can go work from ready only
	break;

	case work:
		HAL_GPIO_WritePin(PLamp_Blue_GPIO_Port, PLamp_Blue_Pin, GPIO_PIN_SET);

	// transfer to while

		if (pwr_sense == 1){
			grandState = emer;
			u_contr = 0;}
		if (stop_sense == 0){
			PWMOut = 0;
			grandState = stopnd;
			u_contr = 0;}
	break;

	case stop:
		HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_SET);
		PWMOut = 0;

		if (stop_sense == 1){
			grandState = Ready;

			mot_dirctn++;
			mot_dirctn%=2;
		}
	break;

	case stopnd:
			HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_SET);
			PWMOut = 0;
			Integral = 0;

			if (stop_sense == 1){
				grandState = work;
			}
		break;

	case emer:
		PWMOut = 0;
		if (pwr_sense == 0){
			grandState = Ready;
			HAL_Delay(100);
			IOExpenderInit();
		}
	break;
	}
}
////////// Absolute Encoder ////////////////////////////////////////////
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
	// re code using position dif time
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

void Speedsmoothfunc(float inpdat){
	//static uint8_t scc 0;
	for(int j = CAPTURENUM-1; j >= 0;j--){
		speedsmoothlog[j] = speedsmoothlog[j-1];
	}
	speedsmoothlog[0] = inpdat;

	float summa = 0.0;
	int errcut = 0;
	for (int k = 0; k < CAPTURENUM;k++){
		if (speedsmoothlog[k]>=500){errcut++;}
		else{summa+= speedsmoothlog[k];}

	}
	RoundNumnd_sm =  summa / (CAPTURENUM-errcut);
}

///////////////////// PID Zero /////////////////////////////
void PIDzero(){
	CrrntTime = micros();
	DeltaTime = (CrrntTime - PreviTime) / 1000000.0; // seconds
	PreviTime = CrrntTime; // log previ here for next loop

	ErrPos[0] = TargetDeg - BinPosXI;

	Propo = K_P * ErrPos[0];

	Integral = Integral + ( ErrPos[0] * DeltaTime ); // Integral -Newton-Leibniz

	Derivate = (ErrPos[0]-ErrPos[1]) / DeltaTime; // d/dt position

	u_contr = Propo + (K_I * Integral) ; // PID u[k] + (K_D * Derivate)

	ErrPos[1] = ErrPos[0]; // log previous error
}

void MotDrvCytron(){

	//   direction chk
	if(u_contr < 0){mot_dirctn= 1;}
	else if(u_contr > 0) {mot_dirctn = 0;}
	else{PWMOut = 0;}

	// speed
	PWMOut= (int)fabsf(u_contr); // Absolute int
	if(PWMOut> 7000){PWMOut = 7000;} // saturate 50% gear 1:6 - 120rpm => 10rpm
	if(PWMOut < 1600 && fabsf(ErrPos[0]) >= 4){PWMOut = 1600;} //pvnt too low pwm that can't drive mot
	//if(ErrPos[0] < 2){PWMOut = 0;}
}

/////////////////////Abs Encoder I2C////////////////////////////////////////////
void IOExpenderInit() {// call when start system
	//Init All// setting from datasheet p17 table 3.5
	static uint8_t Setting[0x16] = {
			0b11111111, // IODIRA 1 = in/2=0ut
			0b11111111, // IODIRB
			0b11111111, // IPOLA  1 = invert / 0  nonINV
			0b11111111, // IPOLB
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
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

void AbsEncI2CReadx(uint8_t *RawRAB){

	if(flag_absenc != 0 && hi2c1.State == HAL_I2C_STATE_READY){
		switch(flag_absenc){
		default:
			break;

		case 1:
			//HAL_I2C_Master_Receive(&hi2c1, ADDR_IOXT, GrayCBitx, 1, 100);
			//HAL_I2C_Master_Seq_Receive_DMA(hi2c, DevAddress, pData, Size, XferOptions);
			HAL_I2C_Mem_Read(&hi2c1, ADDR_IOXT, 0x12, I2C_MEMADD_SIZE_8BIT,
						RawRAB, 2, 100);
			flag_absenc = 2;
		break;

		case 2:
			//invert in IPOL
			GrayCBitXI = (RawEnBitAB[1] << 8) | RawEnBitAB[0]; // GrayCBitx

			//GrayCBitXI = ~GrayCBitx - 0b1111110000000000; // invert and clear 6 high
			//GrayCBitXI = ~GrayCBitx & 0b0000001111111111;
			BinPosXI = GraytoBinario(GrayCBitXI, 10) + POSOFFSET;  //
			if (BinPosXI >= 1024){BinPosXI = BinPosXI % 1024;}
			flag_absenc = 0;
		break;
		}
	}
}
//////////////// End Effector /////////////////////////////////
void Efft_activate(){
	uint8_t cmmd = 0x45;

	static enum{ef_INIT, ef_shoot} efshoot = ef_INIT;
	switch(efshoot){
	default:
	case ef_INIT:
		if(flag_efftActi != 0 && hi2c1.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, ADDR_EFFT, &cmmd, 0, I2C_FIRST_FRAME);
			efshoot = ef_shoot;
		}
	break;

	case ef_shoot:
		if (hi2c1.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, ADDR_EFFT, &cmmd, 1, I2C_LAST_FRAME);
			efshoot = ef_INIT;
			flag_efftActi = 0;
			trig_efftRead = 1;
		}
	break;
	}

}
void Efft_read(uint8_t *Rddata){

	static uint8_t readRQ = 0x23;
	//static uint8_t readrqq[2] = {0x23,0x23};
	static uint8_t efrdStatus = 0;
	/*
	if (flag_efftRead != 0 && hi2c1.State == HAL_I2C_STATE_READY){
		//HAL_I2C_Mem_Read_IT(&hi2c1, ADDR_EFFT, 0x23, I2C_MEMADD_SIZE_8BIT, Rddata, 1);
		//HAL_I2C_Mem_Read(&hi2c1, ADDR_EFFT, 0x23, I2C_MEMADD_SIZE_8BIT, Rddata, 1, 100);
		//flag_efftRead = 0;
	}*/
	switch(efrdStatus){
	default:
	case 0:
		if (flag_efftRead != 0 && hi2c1.State == HAL_I2C_STATE_READY){
			//HAL_I2C_Master_Transmit_IT(&hi2c1, ADDR_EFFT, 0x23, 1);
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, ADDR_EFFT, &readRQ, 1, I2C_FIRST_AND_NEXT_FRAME);
			efrdStatus = 1;
		}
	break;

	case 1:

		if (hi2c1.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Receive_IT(&hi2c1, ADDR_EFFT, Rddata, 1, I2C_LAST_FRAME);
			//HAL_I2C_Master_Receive_IT(&hi2c1, ADDR_EFFT, Rddata, 1);
		flag_efftRead = 0;
		efrdStatus = 0;

			}
	break;
	}

}


/////////////// Emer Interrupt /////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//// EMER ////
	if(GPIO_Pin == GPIO_PIN_11){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		counter_e++;
		grandState = emer;
		bluecounter = 0;
		PWMOut = 0;
		// Motor Driver Torque Lock here
	}
	//// Stop ////
	if(GPIO_Pin == GPIO_PIN_10){
		PWMOut = 0;
		bluecounter = 0;
		if(grandState == work){grandState = stopnd;}
		//else{grandState = stop;}

		}
	//// work Blue button////
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
		flag_efftActi = 1;
		trig_efftRead = 1;
		//flag_efftRead = 1;
	}

	//// setzero ////
		if(GPIO_Pin == GPIO_PIN_2){
			TargetDeg = 0;
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