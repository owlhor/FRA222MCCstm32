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
#include <stdio.h>
#include <string.h>
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
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char UIBuffex[50] =
{ 0 };

uint16_t LEDHz = 1;
uint16_t LEDtimePer = 1;
uint8_t LEDONOFF = 1; // on 1 default
uint32_t LEDTimeStamp = 0;

uint8_t BTNActivate = 0;
uint32_t BTNTimestamp = 0;
uint8_t BTNVal[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void LED_UART_UI();
int16_t UARTRecieveIT();
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
  {
  char temp[]="HELLOS TERRA\r\n please type something to test UART\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10); // strlen = length of str -> config length of data
  char temp2[]="Lobby: Press [0] to LED, Press [1] to button\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)temp2, strlen(temp2),10);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*Method 1 Polling Mode*/

	  //		UARTRecieveAndResponsePolling();

	  		/*Method 2 Interrupt Mode  print every 32 byte*/

	  // IT, no timeout// recieved from console
	  		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

	  		/*Method 2 W/ 1 Char Received*/
	  		int16_t inputchar = UARTRecieveIT();
	  		if(inputchar!=-1) // -1 defined means no data
	  		{
	  			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
	  			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);

	  			//-------------LED_UI_Session---------------------------------
	  			static enum {Lobby,BTNNON,LEDCF} LED_UI_Page = Lobby;
	  			switch(LED_UI_Page){
	  			case Lobby:
	  				//sprintf("Lobby: Press [0] to LED, Press [1] to button\n");

	  				if(inputchar == 49){ //ascii 49 = chr1
	  					LED_UI_Page = BTNNON;
	  					BTNActivate = 1;
	  					sprintf(UIBuffex, "Page:[BTN] type [x] to get back\r\n");}
	  				else if(inputchar == 48){ // chr0
	  					LED_UI_Page = LEDCF;
	  					sprintf(UIBuffex, "Page:[LED] type [x] to get back\r\n");
	  					HAL_UART_Transmit(&huart2, (uint8_t*)UIBuffex, strlen(UIBuffex), 100);
	  					sprintf(UIBuffex, "[a] up Hz\r\n[s] down Hz \r\n[d] ON/OFF \r\n");
	  					}
	  				else{sprintf(UIBuffex, "N/A, Retype\r\n");}
	  			break;

	  			case LEDCF:
	  				if(inputchar == 97){ // a // Speed up Hz + 1
	  					LEDHz++;
	  					sprintf(UIBuffex, "LED Hz:[%d]\r\n", LEDHz);
	  				}
	  				else if(inputchar == 115){ // s // Speed down Hz - 1
	  					if(LEDHz <= 1){LEDHz = 1;}
	  					else{LEDHz = LEDHz - 1;}

	  					sprintf(UIBuffex, "LED Hz:[%d]\r\n", LEDHz);
	  				}
	  				else if(inputchar == 100){ // d  // ON OFF
	  					LEDONOFF = (LEDONOFF+1) % 2; // toggle on off
	  					if (LEDONOFF == 0){sprintf(UIBuffex, "LED: OFF\r\n");}
	  					else{sprintf(UIBuffex, "LED: ON\r\n");}
	  				}
	  				else if(inputchar == 120){ // x
	  					LED_UI_Page = Lobby;
	  					sprintf(UIBuffex, "Lobby: Press [0] to LED, Press [1] to button\r\n");}
	  				else{sprintf(UIBuffex, "N/A, Retype\r\n");}
	  			break;

	  			case BTNNON:
	  				if(inputchar == 120){ // x
	  					LED_UI_Page = Lobby;
	  					BTNActivate = 0;
	  					sprintf(UIBuffex, "Lobby: Press [0] to LED, Press [1] to button\r\n");}
	  				else{sprintf(UIBuffex, "N/A, Retype\r\n");}
	  			break;
	  			}

	  			HAL_UART_Transmit(&huart2, (uint8_t*)UIBuffex, strlen(UIBuffex), 100);
	  		}


	  /*This section just simmulate Work Load*/
	//HAL_Delay(100);
	LEDtimePer = 1000 / LEDHz; // 1000 in HALgett / 1000000 in micros
	if (HAL_GetTick() - LEDTimeStamp >= LEDtimePer){
		LEDTimeStamp = HAL_GetTick();
		if(LEDONOFF == 0){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		}else{  // not 0 = ON
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}

	}
	//Button loop
	if (HAL_GetTick() - BTNTimestamp >= 100){
		BTNTimestamp = HAL_GetTick();
		if(BTNActivate == 1){
			BTNVal[1]=BTNVal[0];
			BTNVal[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			if(BTNVal[0] != BTNVal[1]){ // edge detection
				if(BTNVal[0] == 0){sprintf(UIBuffex, "BTN: Pressed\r\n");}
				else{sprintf(UIBuffex, "BTN: UnP\r\n");}
				HAL_UART_Transmit(&huart2, (uint8_t*)UIBuffex, strlen(UIBuffex), 100);
			}
		}
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
//void UARTRecieveAndResponsePolling()
//{
//	char Recieve[32]={0}; // UART Data recieved here

//	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 32, 1000);
// string print func
//	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve);
//	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
//}


int16_t UARTRecieveIT() // find last update position and return data in that position
{
	static uint32_t dataPos = 0;
	int16_t data=-1; // return -1 if no data
	// xfer size = 32, xfercount will down every time text typed in

	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos) // update when datapos is changed
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1) % huart2.RxXferSize; // update and % 32(overflow)
	}
	return data;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
//	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
//}

void LED_UART_UI(){

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

