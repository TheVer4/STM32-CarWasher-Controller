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

#include "max_matrix_stm32.h"
#include "rc522.h"
#include <stdio.h>
#include <string.h>

#define DEVICE_ID "STM32"
#define UART_BUFFER_SIZE 1
#define UART_MESSAGE_SIZE 17

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t UART1_rxBuffer[UART_BUFFER_SIZE] = {0};
uint8_t UART1_rxMessageBuffer[UART_MESSAGE_SIZE] = {0};
uint16_t userBalance = 0;
uint16_t userTime = 0;
uint8_t cardPassed = 0;
uint8_t relayStates = 0;
uint8_t lastPressedButton = 0;
uint8_t pressCount = 0;
uint8_t status;
uint8_t sn[4] = {0, 0, 0, 0};
uint8_t	str[MFRC522_MAX_LEN];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void ProcessUARTCommand(char * command);
char * GetSTMuID(void);
void TurnRelayEnabled(uint8_t relayNumber);
void TurnRelayDisabled(uint8_t relayNumber);
void InteractWithRC522(void);

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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  char SERIAL_MESSAGE[] = "RDY00000000000000";
  char MATRIX_MESSAGE[40] = " 0:00 ";

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart1, (uint8_t*)SERIAL_MESSAGE, strlen(SERIAL_MESSAGE), 1000);
  HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, UART_BUFFER_SIZE);

  MFRC522_Init();
  max_init(0x1);

  // Initialize Relays
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t even = 0;


  while (1)
  {

	  InteractWithRC522();

	  if(sn[0] == 0 && sn[1] == 0 && sn[2] == 0 && sn[3] == 0)
		  cardPassed = 0;
	  else
		  cardPassed = 1;

	  HAL_Delay(2 * 1000);
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1);
      if(cardPassed) {
    	  if(even++ % 2 == 0)
    		  snprintf(MATRIX_MESSAGE, sizeof(MATRIX_MESSAGE), " %d:%02d ", userTime / 60, userTime % 60);
		  else
			  snprintf(MATRIX_MESSAGE, sizeof(MATRIX_MESSAGE), " %02d P ", userBalance); //userBalance

//    	  scroll_string((uint8_t*)MATRIX_MESSAGE, 100, left);
      } else {
    	  strcpy(MATRIX_MESSAGE, "BCTABbTE KAPTY      WELCOME!        ");
      }

      scroll_string((uint8_t*)MATRIX_MESSAGE, 25, left);
      lastPressedButton = 0;
      pressCount = 0;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Marquee_CS_Pin|Marquee_CLS_Pin|Marquee_DIN_Pin|Relay_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Relay_4_Pin|Relay_7_Pin|Relay_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay_6_Pin|Relay_2_Pin|Relay_5_Pin|Relay_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_Tim_Pin|LED_Balance_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Marquee_CS_Pin Marquee_CLS_Pin Marquee_DIN_Pin Relay_8_Pin */
  GPIO_InitStruct.Pin = Marquee_CS_Pin|Marquee_CLS_Pin|Marquee_DIN_Pin|Relay_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_1_Pin Button_5_Pin Button_2_Pin Button_6_Pin
                           Button_3_Pin Button_7_Pin Button_4_Pin Button_8_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin|Button_5_Pin|Button_2_Pin|Button_6_Pin
                          |Button_3_Pin|Button_7_Pin|Button_4_Pin|Button_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_4_Pin Relay_7_Pin Relay_3_Pin */
  GPIO_InitStruct.Pin = Relay_4_Pin|Relay_7_Pin|Relay_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_6_Pin Relay_2_Pin Relay_5_Pin Relay_1_Pin */
  GPIO_InitStruct.Pin = Relay_6_Pin|Relay_2_Pin|Relay_5_Pin|Relay_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Tim_Pin LED_Balance_Pin */
  GPIO_InitStruct.Pin = LED_Tim_Pin|LED_Balance_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t DEBUG_MODE = 0;
    if(DEBUG_MODE) HAL_UART_Transmit(&huart1, UART1_rxBuffer, UART_BUFFER_SIZE, 100); //DEBUG LINE DELETE IN PROD


    if(UART1_rxBuffer[0] == 64) { //Service Mode, if sent @ symbol, the buffer will be cleared
    	for (int i=0; i<UART_MESSAGE_SIZE; i++) UART1_rxMessageBuffer[i] = 0;
    	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, UART_BUFFER_SIZE);
    	return;
    }


    if(strlen(UART1_rxMessageBuffer) == 0) {
    	strcpy(UART1_rxMessageBuffer, UART1_rxBuffer);
    } else {
    	strcat(UART1_rxMessageBuffer, UART1_rxBuffer);
    }

    if(strlen(UART1_rxMessageBuffer) == UART_MESSAGE_SIZE) {
    	if(DEBUG_MODE) 	HAL_UART_Transmit(&huart1, "\r\n", 2, 100); //DEBUG LINE DELETE IN PROD

    	ProcessUARTCommand(UART1_rxMessageBuffer);

		for (int i=0; i<UART_MESSAGE_SIZE; i++) UART1_rxMessageBuffer[i] = 0;
    }

    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, UART_BUFFER_SIZE);
}

void ProcessUARTCommand(char * command) {
	char * cmd[3] = {0};
	char * arg[14] = {0};
	char response[17] = {0};
	memcpy(cmd, &command[0], 3);
	memcpy(arg, &command[3], 14);
//	HAL_UART_Transmit(&huart1, "Command: ", 9, 100);
//	HAL_UART_Transmit(&huart1, cmd, 10, 100);
//	HAL_UART_Transmit(&huart1, " Length: ", 9, 100);
//	char * len[100];
//	sprintf(len, "%d", strlen(cmd));
//	HAL_UART_Transmit(&huart1, len, strlen(len), 100);
//	HAL_UART_Transmit(&huart1, "\r\n", 2, 100);
//
//	HAL_UART_Transmit(&huart1, "Argument: ", 10, 100);
//	HAL_UART_Transmit(&huart1, arg, 14, 100);
//	HAL_UART_Transmit(&huart1, "\r\n", 2, 100);

	if(strcmp(cmd, "GYN") == 0) { //Give Your Name
		strcpy(response, "DUN");
		char * uid = GetSTMuID();
		sprintf(arg, "%014s", uid);
		free(uid);
		strcat(response, arg);
	} else if(strcmp(cmd, "GCI") == 0) { //Give Card UID
		strcpy(response, "CID");
		sprintf(arg, "NCP%08x%02x%02x%02x", sn[0],sn[1],sn[2],sn[3]);
		strcat(response, arg);
	} else if(strcmp(cmd, "GRS") == 0) { // Get Relay Statuses
		strcpy(response, "RES");
		sprintf(arg, "%014X", relayStates);
		strcat(response, arg);
	} else if(strcmp(cmd, "TRE") == 0) { // Turn Relay Enabled
		strcpy(response, "REO");
		uint8_t state = strtol(arg, NULL, 16);
		TurnRelayEnabled(state);
		strcat(response, arg);
	} else if(strcmp(cmd, "TRD") == 0) { // Turn Relay Disabled
		strcpy(response, "RDO");
		uint8_t state = strtol(arg, NULL, 16);
		TurnRelayDisabled(state);
		strcat(response, arg);
	} else if(strcmp(cmd, "SRS") == 0) { // Switch Relay Status DEPRECATED
		strcpy(response, "RSO");
		sprintf(arg, "000000DEADBEEF");
		strcat(response, arg);
	} else if(strcmp(cmd, "CBV") == 0) { // Change Balance Value
//		HAL_UART_Transmit(&huart1, "Hit CBV\r\n", 9, 100);
		strcpy(response, "VBO");
		uint16_t parsedArgument = strtol(arg, NULL, 16);
		userBalance = (parsedArgument <= 9999) ? parsedArgument : 9999;
		strcat(response, arg);
	} else if(strcmp(cmd, "CTV") == 0) { // Change Time Value
//		HAL_UART_Transmit(&huart1, "Hit CBV\r\n", 9, 100);
		strcpy(response, "VTO");
		uint16_t parsedArgument = strtol(arg, NULL, 16);
		userTime = (parsedArgument <= 3599) ? parsedArgument : 3599;;
		strcat(response, arg);
	} else {
		strcpy(response, "00000000000000000");
	}


	HAL_UART_Transmit(&huart1, response, 17, 100);
//	HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
	return;
}

char * GetSTMuID(void) {
	uint16_t *idBase0 = (uint16_t*)(UID_BASE);
	uint16_t *idBase1 = (uint16_t*)(UID_BASE + 0x02);
//	uint32_t *idBase2 = (uint32_t*)(UID_BASE + 0x04); // sorry, not enough place in proto :(
	uint32_t *idBase3 = (uint32_t*)(UID_BASE + 0x08);
	char * result = malloc(sizeof(char)*16);
	sprintf(result, "%x%x%x", *idBase0, *idBase1, *idBase3);
	return result;
}

void TurnRelayEnabled(uint8_t relayNumber) {
	if((relayNumber & 1) == 1)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // Relay 1
	if((relayNumber & 2) == 2)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // Relay 2
	if((relayNumber & 4) == 4)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); // Relay 3
	if((relayNumber & 8) == 8)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // Relay 4
	if((relayNumber & 16) == 16)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // Relay 5
	if((relayNumber & 32) == 32)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // Relay 6
	if((relayNumber & 64) == 64)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // Relay 7
	if((relayNumber & 128) == 128)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Relay 8

	relayStates |= relayNumber;
}

void TurnRelayDisabled(uint8_t relayNumber) {
	if((relayNumber & 1) == 1)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // Relay 1
	if((relayNumber & 2) == 2)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // Relay 2
	if((relayNumber & 4) == 4)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET); // Relay 3
	if((relayNumber & 8) == 8)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // Relay 4
	if((relayNumber & 16) == 16)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // Relay 5
	if((relayNumber & 32) == 32)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); // Relay 6
	if((relayNumber & 64) == 64)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // Relay 7
	if((relayNumber & 128) == 128)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Relay 8

	relayStates ^= (relayStates&relayNumber);
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	if (pressCount != 1) {
		uint8_t DEBUG_MODE = 0;
		char response[17] = {0};
		if(GPIO_Pin == GPIO_PIN_7 && lastPressedButton != 1) {
			pressCount++;
			lastPressedButton = 1;
			sprintf(response, "ABP00000000000001");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_9 && lastPressedButton != 2) {
			pressCount++;
			lastPressedButton = 2;
			sprintf(response, "ABP00000000000002");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_11 && lastPressedButton != 3) {
			pressCount++;
			lastPressedButton = 3;
			sprintf(response, "ABP00000000000004");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_13 && lastPressedButton != 4) {
			pressCount++;
			lastPressedButton = 4;
			sprintf(response, "ABP00000000000010");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_8 && lastPressedButton != 5) {
			pressCount++;
			lastPressedButton = 5;
			sprintf(response, "ABP00000000000020");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_10 && lastPressedButton != 6) {
			pressCount++;
			lastPressedButton = 6;
			sprintf(response, "ABP00000000000040");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_12 && lastPressedButton != 7) {
			pressCount++;
			lastPressedButton = 7;
			sprintf(response, "ABP00000000000080");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else if(GPIO_Pin == GPIO_PIN_14 && lastPressedButton != 8) {
			pressCount++;
			lastPressedButton = 8;
			sprintf(response, "ABP00000000000100");
			HAL_UART_Transmit(&huart1, response, 17, 100);
//			if(DEBUG == 1) HAL_UART_Transmit(&huart1, "\r\n", 2, 100); // DEBUG LINE REMOVE IN RELEASE
		} else {
			//Do not do anything when else.
//			__NOP();
			return;
		}
	}
}

int cardPresented = 0;

void InteractWithRC522(void) {
	char buff[17];
	MFRC522_Init();

	HAL_Delay(20);

	status = MI_ERR;

	status = MFRC522_Request(PICC_REQIDL, str);

	status = MFRC522_Anticoll(sn);
	MFRC522_Halt();
	MFRC522_AntennaOff();
	if (status == MI_OK) {
		if(cardPresented++ == 0) {
			sprintf(buff, "NCP%08x%02x%02x%02x", sn[0],sn[1],sn[2],sn[3]);
			HAL_UART_Transmit(&huart1, buff, 17, 100);
		}
	}
	else {
		if(sn[0] != 0 && sn[1] != 0 && sn[2] != 0 && sn[3] != 0) {
			sprintf(buff, "WCL%08x%02x%02x%02x", sn[0],sn[1],sn[2],sn[3]);
			HAL_UART_Transmit(&huart1, buff, 17, 100);
		}
		cardPresented = 0;
	  sn[0] = 0;
	  sn[1] = 0;
	  sn[2] = 0;
	  sn[3] = 0;
	}

	MFRC522_Halt();
	MFRC522_AntennaOff();
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
