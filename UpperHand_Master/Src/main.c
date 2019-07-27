/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define   SINE_RES         2048			// Waveform resolution
#define	MASTER	1		//Define the role before compiling

#include "string.h"

const uint16_t function[SINE_RES] = { 2048, 2145, 2242, 2339, 2435, 2530, 2624, 2717, 2808, 2897,
                                      2984, 3069, 3151, 3230, 3307, 3381, 3451, 3518, 3581, 3640,
                                      3696, 3748, 3795, 3838, 3877, 3911, 3941, 3966, 3986, 4002,
                                      4013, 4019, 4020, 4016, 4008, 3995, 3977, 3954, 3926, 3894,
                                      3858, 3817, 3772, 3722, 3669, 3611, 3550, 3485, 3416, 3344,
                                      3269, 3191, 3110, 3027, 2941, 2853, 2763, 2671, 2578, 2483,
                                      2387, 2291, 2194, 2096, 1999, 1901, 1804, 1708, 1612, 1517,
                                      1424, 1332, 1242, 1154, 1068, 985, 904, 826, 751, 679,
                                      610, 545, 484, 426, 373, 323, 278, 237, 201, 169,
                                      141, 118, 100, 87, 79, 75, 76, 82, 93, 109,
                                      129, 154, 184, 218, 257, 300, 347, 399, 455, 514,
                                      577, 644, 714, 788, 865, 944, 1026, 1111, 1198, 1287,
                                      1378, 1471, 1565, 1660, 1756, 1853, 1950, 2047};

const uint16_t function2[64] = { 0x7fd,0x8c5,0x98c,0xa4f,0xb0c,0xbc1,0xc6d,0xd0e,
										0xda3,0xe2a,0xea1,0xf09,0xf5e,0xfa2,0xfd3,0xff0,
										0xffa,0xff0,0xfd3,0xfa2,0xf5e,0xf09,0xea1,0xe2a,
										0xda3,0xd0e,0xc6d,0xbc1,0xb0c,0xa4f,0x98c,0x8c5,
										0x7fd,0x735,0x66e,0x5ab,0x4ee,0x439,0x38d,0x2ec,
										0x257,0x1d0,0x159,0xf1,0x9c,0x58,0x27,0xa,
										0x0,0xa,0x27,0x58,0x9c,0xf1,0x159,0x1d0,
										0x257,0x2ec,0x38d,0x439,0x4ee,0x5ab,0x66e,0x735};

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
DAC_HandleTypeDef hdac1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Rx_data[2];
uint8_t Rx_Buffer[20], Rx_indx, Transfer_cplt;
uint8_t RxTx_Response[] = "Gotcha!!!\r\n";

float voltage = 2.2;
uint8_t valByte;

//Variables for DAC
uint8_t indexx=0;
uint8_t delay=0;
uint8_t setpoint=200;
uint8_t outerloop=100;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void playSound(void)
{
	while(indexx<127)
		  {
		  	 HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_8B_R,function[indexx]);
		  	 indexx++;
		  	 while(delay<setpoint)
		  	 {
		  		  delay++;
		  	 }
		  	 delay=0;
		  }
		  indexx=0;
		  setpoint++;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
	//HAL_UART_Transmit(&huart1, (uint8_t *) RxTx_Response, 10,0xFFFF);
	//HAL_GPIO_WritePin (GPIOB, R17_Orange_Pin, GPIO_PIN_SET);
	//HAL_Delay(500);
	//HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_bufferdata, 18);
	//HAL_GPIO_WritePin (GPIOB, R17_Orange_Pin, GPIO_PIN_RESET);
	//__NOP();

	//Interrupt callback routine
	    uint8_t i;
	    if (huart->Instance == USART1)  //current UART
	        {
	        if (Rx_indx==0) {for (i=0;i<100;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data

	        if (Rx_data[0]!=10) //if received data different from ascii 10 (enter)
	            {
	            Rx_Buffer[Rx_indx++]=Rx_data[0];    //add data to Rx_Buffer
	            }
	        else            //if received data = 13
	            {
	            Rx_indx=0;
	            Transfer_cplt=1;//transfer complete, data is ready to read
	            }

	        HAL_UART_Receive_IT(&huart1, Rx_data, 1);   //activate UART receive interrupt every time
	        }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin (GPIOB, R17_Orange_Pin, GPIO_PIN_SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	void debugPrint(UART_HandleTypeDef *huart1, char _out[])
	    {
	     HAL_UART_Transmit(huart1, (uint8_t *) _out, strlen(_out), 15);
	     Error_Handler();
	    }
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
  MX_USART1_UART_Init();
  if (MASTER==1) {MX_DAC1_Init();}
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_GPIO_WritePin (GPIOB, R17_Orange_Pin, GPIO_PIN_SET);  //Start up indicator
  HAL_Delay(1500);
  HAL_GPIO_WritePin (GPIOB, R17_Orange_Pin, GPIO_PIN_RESET);  //Turn off for other testing....
  HAL_GPIO_WritePin (GPIOA, HC05_KEY_Pin, GPIO_PIN_RESET); //Set this to HIGH to put HC05 in Program Mode
  HAL_Delay(1000);
  HAL_GPIO_WritePin (GPIOA, HC05_Switch_Pin, GPIO_PIN_SET); //Power up the HC05
  HAL_Delay(1000);

//********THIS CODE IS USED TO SET THE ROLE OF THE HC05*********************
  /*debugPrint(&huart1, "AT+ROLE=1");
  debugPrint(&huart1, "\r\n");
  HAL_Delay(1000);
  debugPrint(&huart1, "AT+CMODE=0");
  debugPrint(&huart1, "\r\n");
  HAL_Delay(1000);
  debugPrint(&huart1, "AT+BIND=18,e4,34d7c7");// Slave address
  debugPrint(&huart1, "\r\n");*/
//************************************************************************

  //if (MASTER==1) {HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);}

  valByte = (uint8_t)((voltage/3.3)*255);

  while (1)
  {
	  HAL_UART_Receive_IT(&huart1, (uint8_t *) Rx_data, 1);
	  debugPrint(&huart1, "Master Talking Here!!!");
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */



  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC05_Switch_GPIO_Port, HC05_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC05_KEY_GPIO_Port, HC05_KEY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R17_Orange_GPIO_Port, R17_Orange_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HC05_Switch_Pin */
  GPIO_InitStruct.Pin = HC05_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC05_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HC05_KEY_Pin */
  GPIO_InitStruct.Pin = HC05_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC05_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Metal_Detected_Pin_Pin */
  GPIO_InitStruct.Pin = Metal_Detected_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Metal_Detected_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R17_Orange_Pin */
  GPIO_InitStruct.Pin = R17_Orange_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R17_Orange_GPIO_Port, &GPIO_InitStruct);

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
