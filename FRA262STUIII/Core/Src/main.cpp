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
  // G6, OWL'S OFFICE 2022
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

#define POSOFFSET 52 // angle zero offset abs enc
#define NumPosDataSetDef 1  // DataSet Select

#define ADDR_EFFT 0b01000110 // End Effector Addr 0x23 0010 0011
#define ADDR_IOXT 0b01000000 // datasheet p15

#define RxBuf_SIZE   32
#define MainBuf_SIZE 32
#define PosBufSize 20


#define Dt 0.01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//===============================================================================
///////////// [[DATASET]] /////////////////////////////
/*
 * Set 1 => 45, 90, 135, 180, 225, 270, 315
 * Set 2 => 0, 355 // worst case 2
 * Set 3 => n/a
 * Set 4 => 0, 5, 10, 15, 20, 25, 30, 35, 40  // worst case #1
 * */
static uint8_t NumPosDataSetx  = 0 + NumPosDataSetDef;
float rawPossw_1[PosBufSize] = {0.0, 0.78, 1.57, 2.35, 3.14 ,3.92 ,4.712, 5.497};
float rawPossw_2[PosBufSize] = {0.0, 0.0, 6.195};
float rawPossw_3[PosBufSize] = {0.0, 1.0, 0.5, 1.2, 2.4, 1.1, 2.3, 1.3, 0.8, 8.5};
float rawPossw_4[PosBufSize] = {0.0, 0.00, 0.087, 0.174, 0.261 , 0.349, 0.436, 0.523};

uint16_t PosoffsetMon = 0; // send to CubeMonitor
///////////////// [Grand State] ///////////// [Grand State] //////////////////////
static enum {Ready, work, stop, emer ,stopnd, SetZeroGr} grandState = Ready;
uint8_t pwr_sense = 0;
uint8_t stop_sense = 0;
uint32_t TimeStampGrand = 0;
uint8_t flag_finishTra = 0;

uint64_t _micros = 0;
uint32_t _millis = 0;

uint8_t counter_e = 0;
static uint8_t flag_LasxTraj = 0; // traject and lasershoot state & Flag
static uint8_t position_order = 0; // order of positionlog
// position target logger, send to -> FinalPosition
// Put Queue position here in rad
float positionlog[PosBufSize] = {-1.0, -1.0 , -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0
		, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};

float PosDataSet[PosBufSize] = {0};  // Dataset in rad , Select at switch<->Define

////////////// [Abs Encoder 10 bit I2C] /////////////////////////////
uint32_t timeStampSR = 0;
uint8_t RawEnBitAB[2] = {0b0};
uint16_t GrayCBitXI = 0b0, BinPosXI = 0b0;
uint8_t flag_absenc = 0;
//uint16_t encdummbuf = 0;

//////////// [PWM & Motor Driver] /////////////////////////////////////////
uint16_t PWMOut = 0; // dytycycle = x/10000 % ,TIM4 PB6
uint32_t timestampPWM = 0;
uint8_t mot_dirctn = 0;

///////////////////////==================================
///////////////// [Trajectory cat cat] /////////////////
float Finalposition = 0;      // [From UART] Put goal position here in rad
float Distance = 0;
float Velocity = 0;			 // [From UART] Put Max Velo here
float Acceleration = 0;
float OutPosition = 0;
float OutVelocity = 0;
float OutAcceleration = 0;
float Tb = 0;
float timeFinal = 0;
float TimeinS = 0;
float Currentpos = 0;
int flagNewpos = 0;
uint32_t TimeStampTraject = 0;

float a0 = 0;
float a3 = 0;
float a4 = 0;
float a5 = 0;

/////////////////// [Kalman cat cat] //////////////////////////////

uint32_t TimeStampKalman = 0;
uint64_t runtime = 0;
float Q1 = 4000;

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

float KalP;  		// [To UART] Send position in rad
float KalV;			// [To UART] Send Velocity
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
float P_max = 1024*0.006136;
float e = 0.6*1024*0.006136;
float OutUnwrap = 0;
float CurrentEn = 0;  // Position in rad (conv from BinPosXI)

uint32_t TimeUnwrap = 0;

/////////// [PID cat cat] /////////////////////////////////////////////

uint32_t TimeStampPID_P = 0;

uint8_t ErrPosx = 0;
uint8_t bluecounter = 0;

float TargetDeg = 3.141*2; // target degree pid
float ErrPos[2] = {0};  // error

float ufromposit = 0 ;
float sumError = 0 ;

////// 1/7/65
float K_P = 0.00175;
float K_I = 0.0000; // 0.0
float K_D = 0.0002;    // 0.0

//float K_P = 4;
//float K_I = 0.006;
//float K_D = 2;


float Propo;
float Integral;
float Derivate;
/////////////////////////// [PID Velo] //////////////////////////
float ErrVelo[3] = {0};  // error

//float K_P_V = 2.1225;
//float K_I_V = 0.32222225;
//float K_D_V = 40;

////// 1/7/65
//float K_P_V = 2.0;
//float K_I_V = 0.215;
//float K_D_V = 1.65;

////// 1/7/65 18.16
float K_P_V = 2.000;
float K_I_V = 0.222;
float K_D_V = 1.811;

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

uint64_t timeout_efft = 0;
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
int DataProtocol_AngPosi;
//uint8_t Req_AngPosi[4] = {154,122,183,52};// 31415 , 3.1415 rad 180 degree
uint8_t Req_MaxVelo[4] = {155,0,127,229};// 5 rpm
float DataProtocol_Velo;
//// [UART UI] Receive parameter
uint8_t Set_AngVelo[2];
float DataProtocol_SetVelo;
uint8_t Set_AngPosi[2];
int DataProtocol_SetAngPosi;
uint8_t Set_Goal_1Sta[2];				// raw index from UART single
uint8_t Goal_nSta[8];                   // raw index (uncompressed)
uint8_t Set_Goal_nSta[PosBufSize];		// raw index from UART multi
int Set1_Sta;

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
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
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
void xu_Uart();

uint64_t micros();
uint32_t millis();

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
		// If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		// This is to maintain the circular buffer
		// The old data in the main buffer will be overlapped

		if (oldPos_uart+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			oldPos_uart = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf + oldPos_uart, RxBuf, Size);
			newPos_uart = Size + oldPos_uart;
		}
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
		else if (StartM == 88) {
			StartM = MainBuf[newPos_uart-2];
		    chkStart = StartM >> 4;
		    NameM = (StartM & 15);
		    if (chkStart == 9){
		    	if (NameM >= 1 && NameM <= 14){
		    		All_mode_UARTUI();
		        }
		    }
		}

		// start the DMA again
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	} //huart USART2
}

void All_mode_UARTUI()
{
	// mode 7 operator
	int yur;
	int zur;
	int kur; // counter

	// NameM => 15 mode
		switch (NameM){
		////==============[Test Command]===========
			case 1: // 10010001 01000000 00000000 00101110
				chksum = MainBuf[newPos_uart-1];
				dataF2 = MainBuf[newPos_uart-2];
				dataF1 = MainBuf[newPos_uart-3];
				chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 1;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
				}
				break;
		////==============[Connect MCU]===========
			case 2: //10010010 01101101
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 2;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
				}
				break;
		////==============[Disconnect MCU]===========
			case 3: //10010011 01101100
					chksum = MainBuf[newPos_uart-1];
					chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 3;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);

					ResetParam();
					grandState = Ready;
					PWMOut = 0;

					xu_Uart();
				}
				break;
		////==============[Angular Velo Set]===========
			case 4:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_AngVelo[0] = dataF1;
					Set_AngVelo[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 4;
					DataProtocol_SetVelo = Set_AngVelo[1];
					Velocity = (DataProtocol_SetVelo * 10)/255;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
				}
				break;
		////==============[Angular Position Set]===========
			case 5:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_AngPosi[0] = dataF1;
					Set_AngPosi[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 5;

					DataProtocol_SetAngPosi = (Set_AngPosi[0]*256) + Set_AngPosi[1];
					//positionlog[0] = (DataProtocol_SetAngPosi / (3.14 * 10000) * 180);
					positionlog[0] = (float)(DataProtocol_SetAngPosi / 10000.0);
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
				}
				break;
		////==============[Goal 1 station]===========
			case 6:
					chksum = MainBuf[newPos_uart-1];
					dataF2 = MainBuf[newPos_uart-2];
					dataF1 = MainBuf[newPos_uart-3];
					Set_Goal_1Sta[0] = dataF1;
					Set_Goal_1Sta[1] = dataF2;
					chksum2 = ~(StartM + dataF1 + dataF2);
				if (chksum == chksum2){
					M_state = 6;

					///////// Load 1 station Data /////////////
					positionlog[0] = PosDataSet[Set_Goal_1Sta[1]];
					Set1_Sta = Set_Goal_1Sta[1];
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);

					xu_Uart();
				}
				break;
		////==============[Goal n station]===========
			case 7:
				Nstation = MainBuf[(newPos_uart - datasize_uart)+1];
				yur = Nstation;
				zur = ( yur / 2 ) + yur % 2;
				kur = 0; // counter

				for(int i = 0; i < zur; i++ ){
					dataFSum += MainBuf[oldPos_uart + (i+2)];
					Goal_nSta[i] = MainBuf[oldPos_uart + (i+2)];
				}
				for(int x = 1; x < Nstation+1; x++){
                    if(x % 2 == 1){
                        Set_Goal_nSta[x-1] = (Goal_nSta[kur] % 16);
                    }else if(x % 2 == 0){
                        Set_Goal_nSta[x-1] = (Goal_nSta[kur] / 16);
                        kur++;
                    }
                }

				chksum = MainBuf[newPos_uart-1];
				chksum3 = ~(StartM + Nstation + dataFSum);
				if (chksum == chksum3){
					M_state = 7;

					/////////// Load n station-> positionlog  //////////
					//for(int j = 0; j <= len(PosDataSet); j++){
					for(int j = 0; j <= Nstation; j++){
						positionlog[j] = PosDataSet[Set_Goal_nSta[j]];
						//Set_Goal_nSta -> raw index from UART
					}

					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					dataFSum = 0;
					xu_Uart();
				}
				break;
		////==============[RUN Go to Goal station Order to Work]===========
			case 8: //10011000 01100111
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 8;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();

		///////// Order to Work !!! ///
					grandState = work;
					//if(grandState == Ready){grandState = work;}
					//HAL_UART_Transmit(&huart2, (uint8_t*)temp_f_ack2, 2 ,1000);
				}
				break;
		////==============[Request Current station]===========
			case 9: //10011001 01100110
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 9;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
					//Req_sta[1] = 0;
					//Req_sta[2] = x;
					//Req_sta[3] = ~(Req_sta[0] + Req_sta[1] + Req_sta[2]);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_sta, 4 ,100);
				}
				break;
		////==============[Request Angular Position]===========
			case 10: //10011010 01100101
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 10;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
					DataProtocol_AngPosi = (CurrentEn * 10000); //KalP
					Req_AngPosi[1] = (DataProtocol_AngPosi / 256);
					Req_AngPosi[2] = (DataProtocol_AngPosi % 256);
					Req_AngPosi[3] = ~(Req_AngPosi[0] + Req_AngPosi[1] + Req_AngPosi[2]);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_AngPosi, 4 ,100);
				}
				break;
		////==============[Request MAX Angular velo]===========
			case 11: //10011011 01100100
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 11;
					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
					DataProtocol_Velo = (KalV/(2 * 3.14)) * 60;
					Req_MaxVelo[1] = 0;
					Req_MaxVelo[2] = (DataProtocol_Velo * 255) / 10;
					Req_MaxVelo[3] = ~(Req_MaxVelo[0] + Req_MaxVelo[1] + Req_MaxVelo[2]);
					HAL_UART_Transmit(&huart2, (uint8_t*)Req_MaxVelo, 4 ,1000);
				}
				break;
		////==============[Enable end effector]================
			case 12: //// 12  10011100 01100011
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 12;
					flag_efftActi = 1;

					//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
					xu_Uart();
				}
				break;
		////==============[Disable end effector]================
			case 13: // 13 10011101 01100010
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 13;
					//// End Effector Kill
					trig_efftRead = 0;
					flag_efftRead = 0;
					flag_efftActi = 0;

					xu_Uart();
				}
				break;
		////==============[Set Home Zero]================
			case 14: //10011110 01100001
				chksum = MainBuf[newPos_uart-1];
				chksum1 = ~(StartM);
				if (chksum == chksum1){
					M_state = 14;
					//act as set zero interrupt
					position_order = 0;
					positionlog[position_order] = 0.0;
					Velocity = 9.0;
					grandState = SetZeroGr;

					xu_Uart();
				}
				break;
			}
}

void xu_Uart(){
	//HAL_UART_Transmit_DMA(&huart2, (uint8_t*)temp_s, 2);
	HAL_UART_Transmit(&huart2, (uint8_t*)temp_s_ack1, 2 , 200); //Xu
}

void fn_Uart(){
	HAL_UART_Transmit(&huart2, (uint8_t*)temp_f_ack2, 2, 100);
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

		R << pow(10, 0);

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
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim10); // milli timer
  	HAL_TIM_Base_Start_IT(&htim11); // micro timer

    //PWM Test
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    //UART UI
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    ////// ===== Select DataPosition Dataset =================
    switch(NumPosDataSetx){

    case 1:
    	for(int t = 0;t <= PosBufSize ;t++){
		PosDataSet[t] = rawPossw_1[t];
		} break;

    case 2:
    	for(int t = 0;t <= PosBufSize ;t++){
		PosDataSet[t] = rawPossw_2[t];
		} break;

    case 3:
    	for(int t = 0;t <= PosBufSize ;t++){
    	PosDataSet[t] = rawPossw_3[t];
    	} break;

    case 4:
		for(int t = 0;t <= PosBufSize ;t++){
		PosDataSet[t] = rawPossw_4[t];
		} break;
    }

    PosoffsetMon = 0 + POSOFFSET; // for send to cubeMon
    ////============position buffer Set Default===================
    //for(int f = 0; f <= PosBufSize; f++){
    //		positionlog[f] = -1.0;
    //	}
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

	  	  ///// GrandState ///////////////////

	  	  if(micros() - TimeStampGrand >= 1000){
	  		TimeStampGrand = micros();
	  		GrandStatumix();
	  		////// encoder dummy///////
	  		//encdummbuf++;
	  		//encdummbuf %= 1023;
	  		//BinPosXI = encdummbuf;
	  		//CurrentEn = BinPosXI * 0.006136;
	  	  }

	  	  // Encoder I2CRead

	  	  if (micros()-timeStampSR >= 10000)      // don't use 1
	  	          {
	  	              timeStampSR = micros();           //set new time stamp
	  	              flag_absenc = 1;
	  	          }
	  	  AbsEncI2CReadx(RawEnBitAB);

	  	  //Unwrapping();
	  	  //Kalmanfilter();

	  	  ///////////////////// 2KHz change PWM PB6////////////////////

	  	  if(micros() - timestampPWM >= 500){
	  		  	  timestampPWM = micros(); // stamp
	  	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWMOut); // dutycycle
	  	  		  //__HAL_TIM_SET_PRESCALER(&htim4, pscalr); // Set freq of pwm
	  	  		  //ADC_Target = ADCFeedx[1].datt;

	  	  		///// Mot Driver ///////////////////
	  	  		if (mot_dirctn == 0){
	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_SET);
	  	  		}else{
	  	  		HAL_GPIO_WritePin(Mot_dir_GPIO_Port, Mot_dir_Pin, GPIO_PIN_RESET);
	  	  		}

	  	  	  }

	  	 ////////////// Motor Control 1 round position //////////////////////
	  	  //LaserTrajex_workflow();
/*
	  	    if (flag_LasxTraj == 1){
	  		 Unwrapping();

			 if(flagNewpos==0){
				Currentpos = CurrentEn;
				Finalposition = 300*0.006136; // Put goal position here in rad
				Distance = Finalposition - Currentpos;
				flagNewpos = 1;
			 }
			Trajectory();
			Kalmanfilter();
			controlloop();
	  	 }

	  	  if (grandState == SetZeroGr){
				 Unwrapping();

				 if(flagNewpos==0){
					Currentpos = CurrentEn;
					Finalposition = 0; // Put goal position here in rad
					Distance = Finalposition - Currentpos;
					flagNewpos = 1;
				 }
				Trajectory();
				Kalmanfilter();
				controlloop();
			 }
	  	    */

	  	 //////////// End Effector /////////////////////
	  	 Efft_activate(); // Activate by flag_efftActi = 1;
	  	 Efft_read(&efft_status);
	  	 //// trig_efftRead up for 10 times afrer shoot / trig at shoot state

	  	 if(trig_efftRead != 0 && millis() - timestamp_efft >= 500){
	  		 timestamp_efft = millis();
	  		 flag_efftRead = 1;
	  		 trig_efftRead++;
	  	 }

	  	//// disable this when run with laserwork
	  	// if(efft_status == 0x78 ){//trig_efftRead >= 40 ||
	  	//	 trig_efftRead = 0;
	  	//	 } // read xx times

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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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

  /*Configure GPIO pin : TIM2_CH1_VINCp_Pin */
  GPIO_InitStruct.Pin = TIM2_CH1_VINCp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(TIM2_CH1_VINCp_GPIO_Port, &GPIO_InitStruct);

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
		/// [From UART] run when get {HOME} , {RUN}
		//if (bluecounter != 0){grandState = work;} // can go work from ready only
	break;

	case work:
		HAL_GPIO_WritePin(PLamp_Blue_GPIO_Port, PLamp_Blue_Pin, GPIO_PIN_SET);
		LaserTrajex_workflow();

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

	case SetZeroGr:
		HAL_GPIO_WritePin(PLamp_Blue_GPIO_Port, PLamp_Blue_Pin, GPIO_PIN_SET);

		Unwrapping();

		 if(flagNewpos==0){
			Currentpos = CurrentEn;
			//Finalposition = 0; // Put goal position here in rad
			Distance = Finalposition - Currentpos;
			flagNewpos = 1;
		 }
		Trajectory();
		Kalmanfilter();
		controlloop();
//		HAL_Delay(4000); // Simulate workload
//		flag_finishTra = 1;


		if (flag_finishTra == 1){
			flag_finishTra = 0;
			//HAL_UART_Transmit(&huart2, (uint8_t*)temp_f_ack2, 2, 100);
			fn_Uart();
			grandState = Ready;
			ResetParam();
		}
		if (pwr_sense == 1){grandState = emer;}
		if (stop_sense == 0){
					PWMOut = 0;
					grandState = stopnd;
					u_contr = 0;
				}
	break;

	case stop:
		HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_SET);
		PWMOut = 0;

		X(1,0)=0;
		KalV = X(1,0);

		if (stop_sense == 1){
			grandState = Ready;
			IOExpenderInit();
			HAL_Delay(100);
			//== rotation change for dummy test
			//mot_dirctn++; mot_dirctn%=2;
		}
	break;

	case stopnd:
			HAL_GPIO_WritePin(PLamp_Yellow_GPIO_Port, PLamp_Yellow_Pin, GPIO_PIN_SET);
			PWMOut = 0;
			//Integral = 0;
			X(1,0)=0;
			KalV = X(1,0);


			if (stop_sense == 1){
				grandState = work;
			}
		break;

	case emer:
		ResetParam();
		// Reset every variables at control
		if (pwr_sense == 0){
			grandState = Ready;
			HAL_Delay(250); // wait for emer release shock power
			IOExpenderInit();

		}
	break;
	}
}

void LaserTrajex_workflow(){ // 1, n loop go to shoot laser run
	// -1 means no position value
	//if (Finalposition == -1){flag_LasxTraj = 0;}

	switch(flag_LasxTraj){
	default:
	case 0: // wait go flag
		////////// raise flag to 1 if need to drive the state

		if (grandState == work){ //flag_LasxTraj != 0 ||
			flag_LasxTraj = 1;
			Finalposition = positionlog[position_order]; // receive in rad
		}
		break;
	case 1: //-------------traject-----
		//====flag_LasxTraj will trig trajex in while;
		//////// raise flag to 2 and flag_efftActi = 1; if reach the target the position

		Unwrapping();
		 if(flagNewpos == 0){
			Currentpos = CurrentEn;
			//Finalposition = 300*0.006136; // Put goal position here in rad
			Distance = Finalposition - Currentpos;
			flagNewpos = 1;
			check = 0;
		 }
		Trajectory();
		Kalmanfilter();
		controlloop();

		//HAL_Delay(2500); // Simulate workload
		//flag_finishTra = 1;

		if(flag_finishTra == 1){
			flag_finishTra = 0;
			flag_LasxTraj = 2;
			flag_efftActi = 1;

			timeout_efft = millis();
		}
		break;
	case 2: //---------------Laserwork--------------
		trig_efftRead = 1;

		// if laser finished work or tomeout and not too fast shift state
		//if((efft_status == 0x78 || millis() - timeout_efft >= 6000) && millis() - timeout_efft >= 1500){
		// force encoder to work
		if(efft_status == 0x78 && millis() - timeout_efft >= 5050){
			efft_status = 0x00;
			trig_efftRead = 0;
			position_order++; // go to next obtained position

			if (positionlog[position_order] == -1){
				//Real end, reset all position parameter
				// back to ready
				//HAL_UART_Transmit(&huart2, (uint8_t*)temp_f_ack2, 2, 100);
				fn_Uart();
				flag_LasxTraj = 0;
				ResetParam();
				grandState = Ready;
				}
			else {
				Finalposition = positionlog[position_order];
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
			positionlog[i] = -1.0;
		}
	position_order = 0;
	flag_LasxTraj = 0;
	trig_efftRead = 0;
	bluecounter = 0;
	Distance = 0;
	Currentpos = 0;
	efft_status = 0x00;
	IOExpenderInit();
	//Integral = 0;
	//u_contr = 0;
}

//////////////////// [Trajectory Path] //////////////////////
void Trajectory(){

	if(micros() - TimeStampTraject >= 10000){
		TimeStampTraject = micros();

		Acceleration = 0.5;


		if (Distance/Velocity > Velocity/Acceleration){
			Tb = Velocity/Acceleration;
		}
		else {
			Tb = sqrt(2*abs(Distance));
			Velocity = sqrt(abs(Distance)/2);
		}
		//TimeinS = _micros/10^6;
		timeFinal = (4*abs(Velocity)) + ((abs(Distance)-(2*abs(Velocity)*abs(Velocity)))/abs(Velocity));

		a0 = Currentpos;
		a3 = (1/(2*pow(timeFinal,3)))*(20*Distance);
		a4 = (1/(2*pow(timeFinal,4)))*(30*(Currentpos-Finalposition));
		a5 = (1/(2*pow(timeFinal,5)))*(12*Distance);

		OutPosition = a0+(a3*pow(TimeinS,3))+(a4*pow(TimeinS,4))+(a5*pow(TimeinS,5));
		OutVelocity = (3*a3*pow(TimeinS,2))+(4*a4*pow(TimeinS,3))+(5*a5*pow(TimeinS,4));

//		if (TimeinS < Tb){
//			OutPosition = (0.5*Acceleration*TimeinS*TimeinS)+Currentpos;
//			OutVelocity = Acceleration*TimeinS;
//			OutAcceleration = Acceleration;
//			ch = 1;
//			}
//		else if(TimeinS < (timeFinal-Tb)){
//			OutPosition = (0.5*Acceleration*(Tb*Tb)) + (Velocity*(TimeinS-Tb))+Currentpos;
//			OutVelocity = Velocity;
//			OutAcceleration = 0;
//			ch = 2;
//			}
//		else if(((timeFinal-Tb) <= TimeinS) && (TimeinS <= timeFinal)){
//			OutPosition = (0.5*Acceleration*(Tb*Tb))+ (Velocity*(timeFinal-(2*Tb)))  + (Velocity*(TimeinS-(timeFinal-Tb))) - (0.5*Acceleration*((TimeinS-(timeFinal-Tb))*(TimeinS-(timeFinal-Tb))))+Currentpos;
//			OutVelocity = Velocity-(Acceleration*(TimeinS-(timeFinal-Tb)));
//			OutAcceleration = -Acceleration;
//			ch = 3;
//			}
//		else if(TimeinS > timeFinal){
//			OutPosition = Distance+Currentpos;
//			OutAcceleration = 0;
//			ch = 4;
//			}
//
//		if (Distance > 0){
//			//Velocity = 1.04719755; // [From UART] Put Max Velo here
//			//Acceleration= 0.5;   // recieve frol UART
//			check = 50;
//		}
//		else if(Distance < 0){
//			//Velocity=-1.04719755; // [From UART] Put Max Velo here  (negative)
//			//Velocity= -1 * Velocity;
//			OutVelocity = OutVelocity * -1.0;
//			OutPosition = OutPosition * -1.0;
//		    //Acceleration= -0.5;   // recieve frol UART (negative)
//		    check = 100;
//		}


		TimeinS = TimeinS + Dt;

		//OutVelocity = 0.523598775 ;
		}
}

//////////////////////// [Unwrapping] ///////////////////////
void Unwrapping(){

	if(micros() - TimeUnwrap >= 10000){
		TimeUnwrap = micros();
		Pn = BinPosXI * 0.006136;
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

	if(micros() - TimeStampKalman >= 10000){
		 TimeStampKalman = micros();
	//if(millis() - TimeStampKalman >= 10){
	//	TimeStampKalman = millis();
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
		 //runtime = micros()-TimeStampKalman;

	     //ErrPos[0] = TargetDeg - BinPosXI*0.006136;
	 }

}

///////////////////// [PID cat cat Zero] /////////////////////////////
void PIDPosition(){
	/*CrrntTime = micros();
	//DeltaTime = (CrrntTime - PreviTime) / 1000000.0; // seconds
	PreviTime = CrrntTime; // log previ here for next loop
	*/
	if(micros() - TimeStampPID_P >= 10000){
		TimeStampPID_P = micros();

	//if(millis() - TimeStampPID_P >= 10){
	//	TimeStampPID_P = millis();

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

	if(micros() - TimeStampPID_V >= 10000){
		TimeStampPID_V = micros();
	//if(millis() - TimeStampPID_V >= 10){
	//	TimeStampPID_V = millis();
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

	if( abs( Finalposition - KalP) < 0.005 && abs(KalV) < 0.0005){
		PWMOut = 0;
		check = 8;

		flagNewpos = 0;
		flag_finishTra = 1;
		TimeinS = 0;
	}
	else{
		PIDPosition();
		PIDVelocity();
		MotDrvCytron();
	}
}


void MotDrvCytron(){

	//   direction chk
	//if(u_contr > 0){mot_dirctn= 0;}
	//else if(u_contr < 0) {mot_dirctn = 1;}
	//else{PWMOut = 0;}

	if(micros() - TimeDrive >= 10000){
			TimeDrive = micros();
			u_contr = u_contr * 833.3;
			// u_contr = 2500;
			if(u_contr > 0){
				mot_dirctn= 1;
			}
			else if(u_contr < 0) {
				mot_dirctn = 0;
			}
			else{
				PWMOut = 0;
			}
	// speed
	PWMOut= (int)fabsf(u_contr); // Absolute int
	if(PWMOut > 5000){PWMOut = 5000;} // saturate 50% gear 1:6 - 120rpm => 10rpm
	//if(PWMOut < 1600 ){PWMOut = 1600;} //pvnt too low pwm that can't drive mot
	//&& fabsf(ErrPos[0]) >= 4
	//if(flag_finishTra == 1){PWMOut = 0;}
	}
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
	static uint8_t Xetting[0x16] = {
			0b11111111, // IODIRA 1 = in/2=0ut
			0b11111111, // IODIRB
			0b11111111, // IPOLA  1 = invert / 0  nonINV
			0b11111111, // IPOLB
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, // GPPUA
			0x00, // GPPUB Pull up 100k R
			0x00, 0x00, 0x00, 0x00,
			0x00, // 0x12 GPIOA
			0x00, // 0x13 GPIOB
			0x00, 0x00 };
	// OLATB -> Out data for pinB
	HAL_I2C_Mem_Write(&hi2c1, ADDR_IOXT, 0x00, I2C_MEMADD_SIZE_8BIT, Xetting,
			0x16, 100);

}

void AbsEncI2CReadx(uint8_t *RawRAB){

	if(flag_absenc != 0 && hi2c1.State == HAL_I2C_STATE_READY){

		HAL_I2C_Mem_Read(&hi2c1, ADDR_IOXT, 0x12, I2C_MEMADD_SIZE_8BIT,
								RawRAB, 2, 100);

		GrayCBitXI = (RawEnBitAB[1] << 8) | RawEnBitAB[0]; // GrayCBitx
		// 1023- to reverse
		BinPosXI = 1023 - (GraytoBinario(GrayCBitXI, 10) + POSOFFSET) ;//
		if (BinPosXI >= 1024){BinPosXI %= 1024;}

		flag_absenc = 0;

//		switch(flag_absenc){
//		default:
//			break;
//
//		case 1:
//			HAL_I2C_Mem_Read(&hi2c1, ADDR_IOXT, 0x12, I2C_MEMADD_SIZE_8BIT,
//						RawRAB, 2, 100);
//			flag_absenc = 2;
//		break;
//
//		case 2:
//			//invert in IPOL
//			GrayCBitXI = (RawEnBitAB[1] << 8) | RawEnBitAB[0]; // GrayCBitx
//
//			BinPosXI = 1023 - (GraytoBinario(GrayCBitXI, 10) + POSOFFSET);  //
//			if (BinPosXI >= 1024){BinPosXI = BinPosXI % 1024;}
//			flag_absenc = 0;
//		break;
//
//		}
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


	static uint8_t readRQ = 0x23;
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

/////////////// Emer Interrupt /////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//============// EMER ////=====================
	if(GPIO_Pin == GPIO_PIN_11){
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		grandState = emer;
		bluecounter = 0;
		PWMOut = 0;
		// Motor Driver Torque Lock here
	}
	//=============// Stop ////======================
	if(GPIO_Pin == GPIO_PIN_10){
		PWMOut = 0;
		bluecounter = 0;
		if(grandState == work){grandState = stopnd;}
		else{grandState = stop;}

		}
	//=========// work Blue button //=========//
	if(GPIO_Pin == GPIO_PIN_13){
		bluecounter++;
//		for(uint8_t i = 0; i <= sizeof(PosDataSet); i++){
//		    		positionlog[i] = PosDataSet[i];
//		    	}
		//if(grandState == Ready){}

		////===== Zero Calibrate Func /////////////
		if (bluecounter % 2 == 0){
			PWMOut = 1100;
			mot_dirctn++; mot_dirctn%=2;

		}else{PWMOut = 0;}

		/// == Manual Work Order ==========
		//grandState = work;

		//flag_efftActi = 1;
		//trig_efftRead = 1;
	}

	//=============// setzero //================//
		if(GPIO_Pin == GPIO_PIN_2){
			position_order = 0;
			positionlog[position_order] = 0.0122; // 0.000613 - 0.0122 => 1-2/1024
			Velocity = 7.0;
			grandState = SetZeroGr;
		}
}

///////////////////////////////////// micro timer////////////////////////////////////
uint64_t micros()
{return _micros + htim11.Instance->CNT;}
uint32_t millis()
{return _millis;}// + htim10.Instance->CNT

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim == &htim11)
 {_micros += 65535;}
 if(htim == &htim10)
 {_millis++;}
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
