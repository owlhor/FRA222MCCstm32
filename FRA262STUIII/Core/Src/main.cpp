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
  //==========================================================
  // FRA262 Robotics Studio III, 1 DOF
  // G6, OWL'S OFFICE
   * ---------------------------------------------------------
  // P.WIPOP => Grand State machine, Hardware Interface(Buttons, Encoder, End Effector)
  // M.JEDSADAPORN => UART UI Interface
  // P.CHAKRAPATH => Motor Control(PID, Trajectory, Kalman)
  // =========================================================
  // Pin for
  // - Absolute Encoder read
  //	[Position, Velocity] (I2C/OF-STUIII-SEN021 Circuit)
  // - Motor Control
  // 	[PWM, direction] (Using Cytron MD10C)
  // - End Effector I2C
   * - Voltage sense at power bar
   *     X
   * - EMERGENCY Interrupt
   * - I/O Button
   * -----------------------------------
   * - UART UI Base System connect
   * - Motor POsition, Velocity Control
   *
  //-----------------------------------   */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define _USER_MATH_DEFINES
#include "math.h"
#include <stdio.h>
#include <string.h>

//#include "arm_math.h"
#include <cstdlib>
#include <Eigen/Dense>

using namespace Eigen;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define CAPTURENUM 16 // sample data array size for velo
#define POSOFFSET  -833 // angle zero offset abs enc
#define ADDR_EFFT 0b01000110 // End Effector Addr 0x23 0010 0011
#define ADDR_IOXT 0b01000000 // datasheet p15

#define RxBuf_SIZE   32
#define MainBuf_SIZE 32

#define Dt 0.001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//===============================================================================
///////////////// [Grand State] ///////////// [Grand State] //////////////////////
static enum {Ready, work, stop, emer , stopnd} grandState = Ready;
uint8_t pwr_sense = 0;
uint8_t stop_sense = 0;
//uint32_t TimeStampGrand = 0;
uint64_t _micros = 0;

uint8_t counter_e = 0;
static uint8_t flag_LasxTraj = 0; // traject and lasershoot state & Flag
int8_t positionlog[10] = {-1};   // position target logger, send to -> TargetDeg
static uint8_t position_order = 0; // order of positionlog
////////////// [Abs Encoder 10 bit I2C] /////////////////////////////
uint32_t timeStampSR =0;
//uint8_t RawEnBitA = 0b0, RawEnBitB = 0b0;
uint8_t RawEnBitAB[2] = {0b0};
uint16_t GrayCBitXI = 0b0, BinPosXI = 0b0;
uint8_t flag_absenc = 0;
////////////// [velo dma input capture] ////////////////////
/*
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
*/
//////////// [PWM & Motor Driver] /////////////////////////////////////////
uint16_t PWMOut = 5000; // dytycycle = x/10000 % ,TIM4 PB6
uint32_t timestampPWM = 0;
uint8_t mot_dirctn = 0;
//uint16_t pscalr = 0;

///////////////////////==================================
///////////////// [Trajectory cat cat] /////////////////
float Finalposition = 3.141*2;
float Velocity = 1.04719755 ;
float Acceleration = 0.5;
float OutPosition = 0;
float OutVelocity = 0;
float OutAcceleration = 0;
float Tb = 0;
float timeFinal = 0;
float TimeinS = 0;
float Currentpos = 0;
int flagNewpos = 0;
uint32_t TimeStampTraject = 0;

/////////////////// [Kalman cat cat] //////////////////////////////

uint32_t TimeStampKalman = 0;
uint64_t runtime = 0;
float Q1=15 ;

Matrix <float,3,3> A ;
Matrix <float,3,3> P ;
Matrix <float,3,3> O ;
Matrix <float,3,3> I ;
Matrix <float,1,1> S ;
Matrix <float,1,1> R ;
Matrix <float,1,1> D ;
Matrix <float,1,1> Z ;
Matrix <float,3,1> K ;
Matrix <float,1,3> C ;
Matrix <float,3,1> G ;
Matrix <float,3,3> Q ;

Vector3f X;
Vector3f X1;
Vector3f B;

float KalP;
float KalV;
float KalA;

float DiffVelo = 0;
float Pos1 = 0;
float Pos2 = 0;

/////////////////// [Unwrap testKalman] /////////////////////////////////
float P0=0;
float P_0=0;
float Pn=0;
float P_n=0;
float RawData=0;
float P_max=1024*0.006136;
float e = 0.65*1024*0.006136;
float OutUnwrap = 0;
float CurrentEn = 0;

uint32_t TimeUnwrap=0;

/////////// [PID cat cat] /////////////////////////////////////////////

uint32_t TimeStampPID_P=0;

uint8_t ErrPosx = 0;
uint8_t bluecounter = 0;

float TargetDeg = 3.141*2; // target degree pid
float ErrPos[2] = {0};  // error

float ufromposit = 0 ;
float sumError = 0 ;

float K_P = 4;
float K_I = 0.005;
float K_D = 2;

float Propo;
float Integral;
float Derivate;
/////////////////////////// [PID Velo] //////////////////////////
float ErrVelo[3] = {0};  // error
float K_P_V = 4.57;
float K_I_V = 0.025;
float K_D_V = 20;
float Vcontr[2] = {0};
float SumAll = 0;
uint32_t TimeStampPID_V=0;

float u_contr = 0;

uint32_t TimeStampdiff=0;
uint32_t TimeDrive = 0;
uint8_t ch;
uint8_t check=0;
//////////////===================================================

////////////  [End Effector]  ////////////////////////////////////
// ADDR 0x23 ACK Register 0x45 : Laser On
// ADDR 0x23 ACK Register 0x23 : Laser data read request
// ADDR 0x23 ACK Register ____ : Laser read (request before every read)

// Regis 0x12 : Start (1sec)
// Regis 0x34 : Work (31sec)
// Regis 0x56 : Stop (1sec)
// Regis 0x78 : wait new (- sec)

uint8_t efft_status = 0; // hex status of end effector
uint8_t flag_efftActi = 0;
uint8_t flag_efftRead = 0;
uint8_t trig_efftRead = 0;
uint64_t timestamp_efft = 0;

//////////////// [UART UI Base System] ////////////////////////////

uint8_t RxBuf[RxBuf_SIZE];      // input buffer
uint8_t MainBuf[MainBuf_SIZE];  // collect from RxBuf, Checking use

uint16_t oldPos_uart = 0;		//  UART data position
uint16_t newPos_uart = 0;		//  UART data position

uint8_t temp_s_ack1[2] = {0x58, 0x75};  // Acknowledge#1 "Xu" 0x5875
uint8_t temp_f_ack2[2] = {0x46, 0x6E};  // Acknowledge#2 "Fn" 0x466E
//////[UART UI] transmit parameter (dummy)
uint8_t Req_sta[4] = {153,0,5,97}; //  station 5
uint8_t Req_AngPosi[4] = {154,61,18,22};// 15634 , 1.5634 rad 90 degree
//uint8_t Req_AngPosi[4] = {154,122,183,52};// 31415 , 3.1415 rad 180 degree
uint8_t Req_MaxVelo[4] = {155,0,127,229};// 5 rpm
//// [UART UI] Receive parameter
uint8_t Set_AngVelo[2];
uint8_t Set_AngPosi[2];
uint8_t Set_Goal_1Sta[2];
uint8_t Set_Goal_nSta[10];
//// [UART UI] process param
uint16_t datasize_uart = 0;
uint8_t chkStart;
uint8_t chkStart2;
uint8_t NameM;
uint8_t StartM;
uint8_t chkM = 0;
uint8_t chksum;
uint8_t chksum1;
uint8_t chksum2;
uint8_t chksum3;
uint8_t dataF1;
uint8_t dataF2;
uint8_t M_state;
uint8_t Posdata;
uint8_t Nstation;
uint8_t dataFSum;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
///////////// [Abs Encoder]

void IOExpenderInit();
uint16_t GraytoBinario(uint16_t grayx,uint8_t numbit);
void AbsEncI2CReadx(uint8_t *RawRAB);

void GrandStatumix();
void LaserTrajex_workflow();
void ResetParam();


//void PIDzero();
void PIDPosition();
void PIDVelocity();
void MotDrvCytron();

void Kalmanfilter();
void Trajectory();
void Unwrapping();
void diffvelo();
void controlloop();

void Efft_activate();
void Efft_read(uint8_t *Rddata);

void All_mode_UARTUI();

uint64_t micros();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////// [UART UI Base System] /////////////////////
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		oldPos_uart = newPos_uart;  // Update the last position before copying new data
		datasize_uart = Size;
		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos_uart+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MainBuf_SIZE - oldPos_uart;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf + oldPos_uart, RxBuf, datatocopy);  // copy data in that remaining space

			oldPos_uart = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos_uart = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)MainBuf + oldPos_uart, RxBuf, Size);
			newPos_uart = Size + oldPos_uart;
		}
		StartM = MainBuf[newPos_uart - datasize_uart]; // 0b1001xxxx for check start & mode
		chkStart = StartM >> 4; // for check "start" or nor
		NameM = (StartM & 15); // Check mode

		if (chkStart == 9){ // 0b1001
			if (NameM >= 1 && NameM <= 14){ // 15 work state or not
				All_mode_UARTUI();
			}
		}

		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	} //huart USART2
}

void All_mode_UARTUI()
{
	// NameM => 15 mode
		switch (NameM){
			case 1: // 10010001 01000000 00000000 00101110
				chksum = MainBuf[newPos_uart-1];
				dataF2 = MainBuf[newPos_uart-2];
				dataF1 = MainBuf[newPos_uart-3];
				chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 1;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 2: //10010010 01101101
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 2;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 3: //10010011 01101100
					chksum = MainBuf[newPos_uart-1];
					chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 3;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 4:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_AngVelo[0] = dataF1;
					Set_AngVelo[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 4;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 5:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_AngPosi[0] = dataF1;
					Set_AngPosi[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 5;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 6:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_Goal_1Sta[0] = dataF1;
					Set_Goal_1Sta[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 6;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 7:
				Nstation = MainBuf[(newPos_uart - datasize_uart)+1];
				for(int i=2; i < Nstation+2; i++ ){
					dataFSum += MainBuf[newPos_uart-i];
					Set_Goal_nSta[i-2] = MainBuf[newPos_uart-i];
				}
				chksum = MainBuf[newPos_uart-1];
				chksum3 = ~(StartM + Nstation + dataFSum);
				if (chksum == chksum3){
					M_state = 7;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					dataFSum = 0;
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 8: //10011000 01100111
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 8;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_f_ack2, 2 ,1000);
				}
				break;
			case 9: //10011001 01100110
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 9;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_sta, 4 ,1000);
				}
				break;
			case 10: //10011010 01100101
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 10;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_AngPosi, 4 ,1000);
				}
				break;
			case 11: //10011011 01100100
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 11;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_MaxVelo, 4 ,1000);
				}
				break;
			case 12: //// 12 Enable end effector 10011100 01100011
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 12;
					flag_efftActi = 1;

					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 13: // 13 Disable end effector 10011101 01100010
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 13;

					trig_efftRead = 0;
					flag_efftRead = 0;

					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			case 14: //10011110 01100001
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 14;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 ,1000); //Xu
				}
				break;
			}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/////assign  Matrix  element value  //////////
		A << 1 ,   Dt    ,   (Dt*Dt)/2 ,
		     0 ,    1    ,      Dt    ,
			 0 ,    0    ,      1     ;

		X << 0 ,    0    ,      0     ;

		X1 << 0 ,    0    ,      0     ;


		P << 0.000001 , 			0 	 , 			0     ,
		     0 		 ,    0.000001    ,  		0     ,
			 0 		 ,    		0    ,      0.000001     ;

		O << 0 , 	0 	 , 		0     ,
		     0 ,    0    ,  	0     ,
			 0 ,    0    ,      0     ;

		I << 1 , 0 , 0 ,
			 0 , 1 , 0 ,
			 0 , 0 , 1 ;

		R << pow(10,1); ;

		D << 0 ;

		B << 0 , 0 , 0 ;

		C << 1 , 0 , 0 ;

		G << (Dt*Dt)/2 , Dt , 1 ;

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start_IT(&htim11); // micro timer
    HAL_TIM_Base_Start(&htim2); // Speed
    //HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) &capturedata,CAPTURENUM);

    //PWM Test
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    //UART UI
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    //MCP23017 setting init
    HAL_Delay(50);
    IOExpenderInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	      ///// IT test
	 //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
/*
	  	  ///// GrandState ///////////////////
	  	  //if(micros() - TimeStampGrand >= 1000){
	  		//TimeStampGrand = micros();
	  	  //}
*/
	  	  // Encoder I2CRead
	  	  if (micros()-timeStampSR >= 1000)      // don't use 1
	  	          {
	  	              timeStampSR = micros();           //set new time stamp
	  	              flag_absenc = 1;
	  	              GrandStatumix();
	  	          }
	  	  AbsEncI2CReadx(RawEnBitAB);

	  	  ///////////////////// 2KHz change PWM PB6////////////////////

	  	  if(micros() - timestampPWM >= 500){
	  		  	  timestampPWM = micros(); // stamp
	  	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWMOut); // dutycycle
	  	  		  //__HAL_TIM_SET_PRESCALER(&htim4, pscalr); // Set freq of pwm
	  	  		  //ADC_Target = ADCFeedx[1].datt;
	  	  		if (mot_dirctn == 0){
	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_RESET);
	  	  		}else{
	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_SET);
	  	  		}

	  	  	  }

	  	 ////////////// Motor Control//////////////////////
	  	 if (grandState ==  work){

	  		 if(flagNewpos==0){
	  		    Currentpos = CurrentEn;
	  		    flagNewpos = 1;
	  		 }
	  		 Unwrapping();
	  		 Trajectory();

	  		 Kalmanfilter();
	  		 controlloop();
	  		/* PIDPosition();
	  		 PIDVelocity();
	  	//	 PIDzero();
	  		 MotDrvCytron();
	  		 */
	  	 }

	  	 //////////// End Effector /////////////////////
	  	 Efft_activate(); // Activate by flag_efftActi = 1;
	  	 Efft_read(&efft_status);
	  	 //// trig_efftRead up for 10 times afrer shoot / trig at shoot state
	  	 if(trig_efftRead != 0 && micros() - timestamp_efft >= 200000){
	  		 timestamp_efft = micros();
	  		 flag_efftRead = 1;
	  		 trig_efftRead++;

	  	 }
	  	 //// disable this when run with laserwork
	  	 if(trig_efftRead >= 40 || efft_status == 0x78 ){//
	  		 trig_efftRead = 0;
	  	 	 //flag_efftRead = 0;
	  		 } // read xx times

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Mot_dir_Pin|PLamp_Green_Pin|PLamp_Blue_Pin|PLamp_Yellow_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin EXTI11_EMER_Pin */
  GPIO_InitStruct.Pin = B1_Pin|EXTI11_EMER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pwr_Sense_Pin */
  GPIO_InitStruct.Pin = Pwr_Sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pwr_Sense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mot_dir_Pin PLamp_Green_Pin PLamp_Blue_Pin PLamp_Yellow_Pin */
  GPIO_InitStruct.Pin = Mot_dir_Pin|PLamp_Green_Pin|PLamp_Blue_Pin|PLamp_Yellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI10_Stop_Pin */
  GPIO_InitStruct.Pin = EXTI10_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI10_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Stop_Sense_Pin */
  GPIO_InitStruct.Pin = Stop_Sense_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Stop_Sense_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI2_SetZero_Pin */
  GPIO_InitStruct.Pin = EXTI2_SetZero_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI2_SetZero_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
////////// [Grand State] //////////////////////// [Grand State] ////////////////////////
//======== [Grand State] ======================== [Grand State] ========================
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
	pwr_sense = HAL_GPIO_ReadPin(Pwr_Sense_GPIO_Port, Pwr_Sense_Pin);

	switch(grandState){
	default:
	case Ready:
		HAL_GPIO_WritePin(PLamp_Green_GPIO_Port, PLamp_Green_Pin, GPIO_PIN_SET);
		//PWMOut = 1200;
		Unwrapping();
		Kalmanfilter();
		//diffvelo();

		if (pwr_sense == 1){grandState = emer;}
		if (stop_sense == 0){grandState = stop;}
		if (bluecounter != 0){grandState = work;} // can go work from ready only
	break;

	case work:
		HAL_GPIO_WritePin(PLamp_Blue_GPIO_Port, PLamp_Blue_Pin, GPIO_PIN_SET);
		//LaserTrajex_workflow();

		if (pwr_sense == 1){
			grandState = emer;
			u_contr = 0;
		}
		if (stop_sense == 0){
			PWMOut = 0;
			grandState = stopnd;
			u_contr = 0;
		}
	break;

	case stop:
		HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_SET);
		PWMOut = 0;
		u_contr = 0;

		X(1,0)=0;
		KalV = X(1,0);

		if (stop_sense == 1){
			grandState = Ready;

			// rotation change for dummy test
			//mot_dirctn++;
			//mot_dirctn%=2;
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
		ResetParam();
		// Reset every variables at control
		if (pwr_sense == 0){
			grandState = Ready;
			HAL_Delay(100);
			IOExpenderInit();
		}
	break;
	}
}

void LaserTrajex_workflow(){ // 1, n loop go to shoot laser run
	// -1 means no position value
	if (TargetDeg == -1){flag_LasxTraj = 0;}

	switch(flag_LasxTraj){
	default:
	case 0: // wait go flag
		////////// raise flag to 1 if need to drive the state

		if (grandState == work){ //flag_LasxTraj != 0 ||
			flag_LasxTraj = 1;
			TargetDeg = positionlog[position_order];
		}
		break;
	case 1: // traject

		MotDrvCytron();
		//////// raise flag to 2 and flag_efftActi = 1; if reach the target the position
		if(ErrPos[0] < 2){
			flag_efftActi = 1;
			PWMOut = 0;
			flag_LasxTraj = 2;
		}
		break;
	case 2: // Laserwork
		trig_efftRead = 1;

		if(efft_status == 0x78){ // if laser finished work
			trig_efftRead = 0;
			position_order++; // go to next obtained position

			if (positionlog[position_order] == -1){
				//Real end, reset all position parameter
				// back to ready
				flag_LasxTraj = 0;
				ResetParam();
				grandState = Ready;
				}
			else {
				TargetDeg = positionlog[position_order];
				flag_LasxTraj = 1;
			} // continue next pos if have
		}
		break;
	}
}

void ResetParam(){
	// Reset every variables at control
	// reset position buffer
	PWMOut = 0;
	for(int i = 0; i <= position_order; i++){
		positionlog[position_order] = -1;
	}
	position_order = 0;
	Integral = 0;
	u_contr = 0;
}

//////////////////// [Trajectory Path] //////////////////////
void Trajectory(){
	//0.01 -> 0.1s
	if(micros() - TimeStampTraject >= 1000){
		TimeStampTraject = micros();

		if (Finalposition/Velocity > Velocity/Acceleration){
			Tb = Velocity/Acceleration;
		}
		else {
			Tb = sqrt(2*Finalposition);
			Velocity = sqrt(Finalposition/2);
		}
		//TimeinS = _micros/10^6;
		timeFinal = (4*Velocity) + ((Finalposition-(2*Velocity*Velocity))/Velocity);

		if (TimeinS < Tb){
			OutPosition = (0.5*Acceleration*TimeinS*TimeinS)+Currentpos;
			OutVelocity = Acceleration*TimeinS;
			OutAcceleration = Acceleration;
			ch = 1;
		}
		else if(TimeinS < (timeFinal-Tb)){
			OutPosition = (0.5*Acceleration*(Tb*Tb)) + (Velocity*(TimeinS-Tb))+Currentpos;
			OutVelocity = Velocity;
			OutAcceleration = 0;
			ch = 2;
		}
		else if(((timeFinal-Tb) <= TimeinS) && (TimeinS <= timeFinal)){
			OutPosition = (0.5*Acceleration*(Tb*Tb))+ (Velocity*(timeFinal-(2*Tb)))  + (Velocity*(TimeinS-(timeFinal-Tb))) - (0.5*Acceleration*((TimeinS-(timeFinal-Tb))*(TimeinS-(timeFinal-Tb))))+Currentpos;
			OutVelocity = Velocity-(Acceleration*(TimeinS-(timeFinal-Tb)));
			OutAcceleration = -Acceleration;
			ch = 3;
		}
		else if(TimeinS > timeFinal){
			OutPosition = Finalposition+Currentpos;
			OutAcceleration = 0;
			ch = 4;
		}

		TimeinS = TimeinS + Dt;

	 }

	//OutVelocity = 0.523598775 ;
}

//////////////////////// [Unwrapping] ///////////////////////
void Unwrapping(){

	if(micros() - TimeUnwrap >= 1000){
		TimeUnwrap = micros();
		Pn = BinPosXI*0.006136;
		if(Pn-P_n <= -1*e){
			P0 = P_0 + P_max;
		}
		else if(Pn - P_n >= e){
			P0 = P_0 - P_max;
		}
		else{
			P0 = P_0;
		}

		OutUnwrap = Pn + P0;
		CurrentEn = BinPosXI * 0.006136;
		P_n = Pn;
		P_0 = P0;
	}
}

void diffvelo(){
	Pos1 = OutUnwrap;
	if(micros() - TimeStampdiff >= 1000){
		TimeStampdiff = micros();
		if(Pos1 != Pos2){
			DiffVelo = (Pos1-Pos2)/Dt;
		}
		else{
			DiffVelo = DiffVelo;
		}
	//	DiffVelo=(Pos1-Pos2)/Dt;
		Pos2 = Pos1;

	}
}


/////////////////////////// [Kalman Filter] ///////////////////////
void Kalmanfilter(){

	 if(micros() - TimeStampKalman >= 1000){
		 TimeStampKalman = micros();
		 ////////// Predict ////////////////////
		 Q = G*Q1*G.transpose();
		 X = A*X1 ;
		 P = A*O*A.transpose()+Q;

		 //////////////// Update /////////////////////
	     Z << OutUnwrap ;
	//     Z << DiffVelo ;
		 K = (P*C.transpose())*(C*P*C.transpose()+R).inverse();
		 X1 = X+K*(Z-C*X);
		 O = (I-(K*C))*P;

		 KalP = X(0,0);
		 KalV = X(1,0);
		 KalA = X(2,0);
		 runtime = micros()-TimeStampKalman;

	     //ErrPos[0] = TargetDeg - BinPosXI*0.006136;
	 }

}

///////////////////// [PID cat cat Zero] /////////////////////////////
void PIDPosition(){
	/*CrrntTime = micros();
	//DeltaTime = (CrrntTime - PreviTime) / 1000000.0; // seconds
	PreviTime = CrrntTime; // log previ here for next loop
	*/
	if(micros() - TimeStampPID_P >= 1000){
		TimeStampPID_P = micros();

		ErrPos[0] = OutPosition - KalP;
	//	ErrPos[0] = OutVelocity - KalV;
		sumError = sumError + ErrPos[0];

		Propo = K_P * ErrPos[0];

		Integral = K_I * sumError; // Integral -Newton-Leibniz

		Derivate = K_D * (ErrPos[0]-ErrPos[1]); // d/dt position

		ufromposit = Propo + Integral + Derivate;

		//u_contr = ufromposit ;
		ErrPos[1] = ErrPos[0]; // log previous error
	}
}

void PIDVelocity(){

	if(micros() - TimeStampPID_V >= 1000){
		TimeStampPID_V = micros();
		/*
		ErrVelo[0] = OutVelocity - KalV ;
		SumAll = (K_P_V + K_D_V + K_I_V)*ErrVelo[0]-(K_P_V+2*K_D_V)*ErrVelo[1]+K_D_V*ErrVelo[2] ;
		Vcontr[0] = Vcontr[1] + SumAll ;
		u_contr = Vcontr[0] ; //Out motor
		Vcontr[1] = Vcontr[0] ;
		ErrVelo[2] = ErrVelo[1] ;
		ErrVelo[1] = ErrVelo[0] ;
		*/

		ErrVelo[0] = OutVelocity + ufromposit - KalV;
		SumAll = SumAll + ErrVelo[0];

		u_contr = (K_P_V * ErrVelo[0])+(K_I_V * SumAll)+(K_D * (ErrVelo[0]-ErrVelo[1])) ;
		ErrVelo[1] = ErrVelo[0]; // log previous error
		}
}

void controlloop(){

	if( abs( OutPosition - KalP) < 0.01 && KalV < 0.0005){
		PWMOut = 0;
		check = 8;
	}
	else{
		PIDPosition();
		PIDVelocity();
		MotDrvCytron();
	}
}


void MotDrvCytron(){

	//   direction chk
	if(u_contr < 0){mot_dirctn= 0;}
	else if(u_contr > 0) {mot_dirctn = 1;}
	else{PWMOut = 0;}


	// speed
	PWMOut= (int)fabsf(u_contr); // Absolute int
	if(PWMOut > 5000){PWMOut = 5000;} // saturate 50% gear 1:6 - 120rpm => 10rpm
	if(PWMOut < 1600 && fabsf(ErrPos[0]) >= 4){PWMOut = 1600;} //pvnt too low pwm that can't drive mot
	if(ErrPos[0] < 2){PWMOut = 0;}
}

///////////////////// [Abs Encoder I2C] ////////////////////////////////////////////
////////// [Absolute Encoder] ////////////////////////////////////////////
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

/////////////////////// [End Effector] //////////////////////////////////
void Efft_activate(){
	uint8_t cmmd = 0x45;

	static enum{ef_INIT, ef_shoot} efshoot = ef_INIT;
	switch(efshoot){
	default:
	case ef_INIT:
		if(flag_efftActi != 0 && hi2c3.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, ADDR_EFFT, &cmmd, 0, I2C_FIRST_FRAME);
			//HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, ADDR_EFFT, &cmmd, 1, I2C_FIRST_AND_LAST_FRAME);
			//HAL_I2C_Master_Transmit(&hi2c3, ADDR_EFFT, &cmmd, 2, 100);
			efshoot = ef_shoot;
		}
	break;

	case ef_shoot:
		if (hi2c3.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, ADDR_EFFT, &cmmd, 1, I2C_LAST_FRAME);
			efshoot = ef_INIT;
			flag_efftActi = 0;
			trig_efftRead = 1;
		}
	break;
	}

}
void Efft_read(uint8_t *Rddata){


	uint8_t readRQ = 0x23;
/*
	if (flag_efftRead != 0 && hi2c3.State == HAL_I2C_STATE_READY){
		HAL_I2C_Mem_Read_IT(&hi2c3, ADDR_EFFT, readRQ, I2C_MEMADD_SIZE_8BIT, Rddata, 1);
		//HAL_I2C_Mem_Read(&hi2c1, ADDR_EFFT, &readRQ, I2C_MEMADD_SIZE_8BIT, Rddata, 1, 100);
		flag_efftRead = 0;
	}
	*/
	static uint8_t efrdStatus = 0;
	switch(efrdStatus){
	default:
	case 0:
		if (flag_efftRead != 0 && hi2c3.State == HAL_I2C_STATE_READY){
			//HAL_I2C_Master_Transmit_IT(&hi2c3, ADDR_EFFT, 0x23, 1);
			HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, ADDR_EFFT, &readRQ, 1, I2C_FIRST_AND_NEXT_FRAME);
			//HAL_I2C_Master_Transmit(&hi2c3, ADDR_EFFT, &readRQ, 2, 100);
			//HAL_I2C_Master_Seq_Transmit_DMA(&hi2c3, ADDR_EFFT, &readRQ, 1, I2C_FIRST_AND_NEXT_FRAME);
			efrdStatus = 1;
		}
	break;

	case 1:

		if (hi2c3.State == HAL_I2C_STATE_READY){
			HAL_I2C_Master_Seq_Receive_IT(&hi2c3, ADDR_EFFT, Rddata, 1, I2C_LAST_FRAME);
			//HAL_I2C_Master_Seq_Receive_DMA(&hi2c3, ADDR_EFFT, Rddata, 1, I2C_LAST_FRAME);
			//HAL_I2C_Master_Receive(&hi2c3, ADDR_EFFT ,Rddata, 1, 100);
		flag_efftRead = 0;
		efrdStatus = 0;
			}
	break;
	}

}
/////////////////UART UI Base System ////////////////////////////
///////// Obsolete, new at User Begin 0////////////////////
/*
void All_mode_UARTUI()
{
	// DataIn = 1 byte Data from UART Recieve
	switch (chkM){				// Check mode State
		default:
		case 0: 				// Check start INIT
			StartM = DataIn;    // 	Use in Checksum Frame 3
			uint8_t chkStart = DataIn >> 4;
			if (chkStart == 9){ // 9 0b1001
				chkM = 1;
			}else{chkM = 0;}
			break;

		case 1:					// Check if mode 1 - 14 or not
			NameM = (DataIn & 15); // 0b00001111
			if (NameM >= 1 && NameM <= 14){
				chkM = 2;
			}
			else{chkM = 0;}
			break;
		///////////////////////// // 14Mode work State //////////////////////////
		case 2:
		switch (NameM){			// 14Mode work State
				case 1:			// Check
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						//M_state = 1;
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 2:			// MCU Connect ,2 byte DataFrame 1
					chksum = DataIn;
					chksum1 = ~(StartM);	// Check condition from manual
					if (chksum == chksum1){	// Transmit back ack1
						//M_state = 2;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}

					break;
				case 3:			// MCU DisConnect ,2 byte DataFrame 1
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 3;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 4:			// Set Angular Velocity
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						//M_state = 4;
						/// Add work here///////////
												///////////////////////////
					HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 5:			// Set Angular Position
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						//M_state = 5;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 6:			// Set goal single station
					if (dataFN == 2){
						dataF1 = DataIn;
					}
					if(dataFN == 3){
						dataF2 = DataIn;
					}
					chksum = DataIn;
					chksum2 = ~(StartM + dataF1 + dataF2);
					if (chksum == chksum2){
						//M_state = 6;
						/// Add work here///////////
												///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 7:			//set Goal multiple station
					if (dataFN == 2){
						Nstation = DataIn;
					}
					if (dataFN < Nstation + 3){
						if (dataFN == countN + 3){
							dataFSum += DataIn;
							countN += 1;
						}
					}
					chksum = DataIn;
					chksum3 = ~(StartM + Nstation + dataFSum);
					if (chksum == chksum3){
						//M_state = 7;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100);
						chkM = 0;
						dataFN = 0;
						countN = 0;
					}
					break;
				case 8:			// Order Go to that position
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 8;
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						//// simulate workload
						HAL_Delay(1000);
						grandState = work;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_f, 2 ,100);//Fn
						chkM = 0;
						dataFN = 0;
					}
					break;

				////////////// Frame 2 ////////////////////////////////////////
				case 9:			// Request Current Station
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 9;
						/// Add work here///////////
						///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 10:	// Request angular position
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 10;
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						/// Add work here///////////
						///////////////////////////

						//uint16_t angu = BinPosXI * 360 / 1024;
						//HAL_UART_Transmit(&huart2, (uint16_t*)angu, 2 ,100);
						chkM = 0;
						dataFN = 0;
					}
					break;
				case 11:		// Request Max Velo
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 11;
						/// Add work here///////////
												///////////////////////////
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
				case 12:	// 12 Enable end effector
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 12;
						flag_efftActi = 1;

						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;

				case 13:	// 13  Disable end effector
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 13;

						trig_efftRead = 0;
						flag_efftRead = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;

				case 14:	// 14 Set Home
					chksum = DataIn;
					chksum1 = ~(StartM);
					if (chksum == chksum1){
						//M_state = 14;
						/// Add work here///////////
						TargetDeg = 0;
						//// PID and MotDrv to 0
						HAL_UART_Transmit(&huart2, (uint8_t*)temp_s, 2 ,100); //Xu
						chkM = 0;
						dataFN = 0;
					}
					break;
					} /// End switch NameM
			break;
	}//end sw chkM

}
*/
/////////////// Emer Interrupt /////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//// EMER ////
	if(GPIO_Pin == GPIO_PIN_11){
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
		//trig_efftRead = 1;
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
