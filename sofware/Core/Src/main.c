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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_SIZE 1
#define YES 1
#define NO 0
#define FORWARD 1
#define BACKWARD -1
#define STAY (uint16_t) 0
#define TIMEOFSHOT 2000
#define SPEED (uint16_t) 255
#define DELAY 50
#define SMALL_DELAY 5
#define ANGLE (uint16_t) 50
#define MIDDLE_POS (uint16_t) 700
#define MAX_ANGLE (uint16_t) 1250
#define VER_DELTA (uint16_t) 350
#define VER_NULL (uint16_t) 200
#define DIV (uint16_t) 20
#define DIV_VER (uint16_t) 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buff[BUFF_SIZE];
uint8_t rx_buff;
uint8_t buff_after;
uint16_t ver_angle = MIDDLE_POS - VER_NULL;
uint16_t hor_angle = MIDDLE_POS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

int MOTORS_MOVE(int8_t condition_motor_left, int8_t condition_motor_right);
void SERVO_MOVE(uint8_t is_upper, uint8_t position);
void GUNSHOOT();
void MUSIC_PLAY();
void rx_parse();
void check_continue();
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t times = 0;
  uint8_t flag = 1;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);

  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ver_angle);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, hor_angle);
  HAL_Delay(DELAY);

  HAL_UART_Receive_IT(&huart1, &rx_buff, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    if((HAL_GetTick() - times) > 1000) // интервал 1000мс = 1сек
    { 
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, flag);

      if (flag == 1)
        flag = 0;
      else
        flag = 1;
      times = HAL_GetTick();
    }

    switch (buff[0]) 
    {
      case 'F': // motor forward
        MOTORS_MOVE(FORWARD, FORWARD);
        if (buff_after == buff[0]) {
          buff_after = 0;
          MOTORS_MOVE (STAY, STAY);
        } else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'B': // motor backward
        MOTORS_MOVE(BACKWARD, BACKWARD);
        if (buff_after == buff[0]){
          buff_after = 0;
          MOTORS_MOVE (STAY, STAY);
        } else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'R': // motor right
        MOTORS_MOVE(FORWARD, BACKWARD);
        if (buff_after == buff[0]){
          buff_after = 0;
          MOTORS_MOVE (STAY, STAY);
        } else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'L': // motor left
        MOTORS_MOVE(BACKWARD, FORWARD);
        if (buff_after == buff[0]){
          buff_after = 0;
          MOTORS_MOVE (STAY, STAY);
        } else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'U': // servo up
        SERVO_MOVE(YES, YES);
        if (buff_after == buff[0])
          buff_after = 0;
        else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'D': // servo down
        SERVO_MOVE(YES, NO);
        if (buff_after == buff[0])
          buff_after = 0;
        else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'S': // servo left
        SERVO_MOVE(NO, YES);
        if (buff_after == buff[0])
          buff_after = 0;
        else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'H': // servo right
        SERVO_MOVE(NO, NO);
        if (buff_after == buff[0])
          buff_after = 0;
        else
          buff_after = buff[0];
        buff[0] = 0;
        break;
      case 'G': // GUNSHOT
        GUNSHOOT();
        buff[0] = 0;
        break;
      case 'M': // MUSIC
        MUSIC_PLAY();
        buff[0] = 0;
        break;
      default:
        check_continue();
        break;
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1151;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Laser_Pin */
  GPIO_InitStruct.Pin = Laser_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Laser_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function for check to continue work.
  * @param  None
  * @retval None
  */
void check_continue() {
  switch (buff_after) {
    case 'F': // motor forward
      MOTORS_MOVE(FORWARD, FORWARD);
      break;
    case 'B': // motor backward
      MOTORS_MOVE(BACKWARD, BACKWARD);
      break;
    case 'R': // motor right
      MOTORS_MOVE(FORWARD, BACKWARD);
      break;
    case 'L': // motor left
      MOTORS_MOVE(BACKWARD, FORWARD);
      break;
    case 'U': // servo up
      SERVO_MOVE(YES, YES);
      break;
    case 'D': // servo down
      SERVO_MOVE(YES, NO);
      break;
    case 'S': // servo left
      SERVO_MOVE(NO, YES);
      break;
    case 'H': // servo right
      SERVO_MOVE(NO, NO);
      break;
    case 'G': // GUNSHOT
      GUNSHOOT();
      break;
    case 'M': // MUSIC
      MUSIC_PLAY();
      break;
    default:
      break;
  }
}

/**
  * @brief  This function for moving motors.
  * @param  2 params - type of moving for each motor
  * @retval None
  */
 int MOTORS_MOVE(int8_t condition_motor_left, int8_t condition_motor_right) 
 {

  ver_angle = MIDDLE_POS - VER_NULL;
  hor_angle = MIDDLE_POS;
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ver_angle);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, hor_angle);
  HAL_Delay(DELAY);

  if (condition_motor_left == FORWARD && condition_motor_right == FORWARD) {
    // if (TIM2->CCR1 == STAY && TIM2->CCR2 == (SPEED - 1) && TIM2->CCR3 == STAY && TIM2->CCR4 == (SPEED - 1)) {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (SPEED - 1));
      HAL_Delay(SMALL_DELAY);
    // } else {
    //   for (uint16_t i = 0; i < SPEED; i += 5) {
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (SPEED - 1));
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (SPEED - 1));
    //     HAL_Delay(SMALL_DELAY);
    //   }
    // }
  } else if (condition_motor_left == BACKWARD && condition_motor_right == FORWARD) {
    // if (TIM2->CCR1 == (SPEED - 1) && TIM2->CCR2 == STAY && TIM2->CCR3 == STAY && TIM2->CCR4 == (SPEED - 1)) {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (SPEED - 1));
      HAL_Delay(SMALL_DELAY);
    // } else {
    //   for (uint16_t i = 0; i < SPEED; i += 5) {
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, i);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, i);
    //     HAL_Delay(SMALL_DELAY);
    //   }
    // }
  } else if (condition_motor_left == FORWARD && condition_motor_right == BACKWARD) {
    // if (TIM2->CCR1 == STAY && TIM2->CCR2 == (SPEED - 1) && TIM2->CCR3 == (SPEED - 1) && TIM2->CCR4 == STAY) {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
      HAL_Delay(SMALL_DELAY);
    // } else {
    //   for (uint16_t i = 0; i < SPEED; i += 5) {
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, i);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, i);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
    //     HAL_Delay(SMALL_DELAY);
    //   }
    // }
  } else if (condition_motor_left == BACKWARD && condition_motor_right == BACKWARD) {
    // if (TIM2->CCR1 == (SPEED - 1) && TIM2->CCR2 == STAY && TIM2->CCR3 == (SPEED - 1) && TIM2->CCR4 == STAY) {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, (SPEED - 1));
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
      HAL_Delay(SMALL_DELAY);
    // } else {
    //   for (uint16_t i = 0; i < SPEED; i += 5) {
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, i);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, i);
    //     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
    //     HAL_Delay(SMALL_DELAY);
    //   }
    // }
  }

  if (condition_motor_left == STAY && condition_motor_right == STAY) {
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
  }
 }

 /**
  * @brief  This function for moving servo.
  * @param is this upper servo or not
  * @retval None
  */
 void SERVO_MOVE(uint8_t is_upper, uint8_t position) 
 {

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, STAY);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, STAY);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, STAY);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, STAY);
  
  if (is_upper) 
  {
    if (position && (ver_angle > ANGLE + VER_DELTA)) 
    {
      for (uint16_t i = 0; i < ANGLE / DIV_VER; i++) {
        ver_angle -= 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ver_angle);
        HAL_Delay(SMALL_DELAY);
      }
    }
    else if (ver_angle < MAX_ANGLE - VER_DELTA)
    {
      for (uint16_t i = 0; i < ANGLE / DIV_VER; i++) {
        ver_angle += 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ver_angle);
        HAL_Delay(SMALL_DELAY);
      }
    }
  }
  else
  {
    if (position && (hor_angle < MAX_ANGLE))
    {
      for (uint16_t i = 0; i < ANGLE / DIV; i++) {
        hor_angle += 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, hor_angle);
        HAL_Delay(SMALL_DELAY);
      }
    }
    else if (hor_angle > ANGLE)
    {
      for (uint16_t i = 0; i < ANGLE / DIV; i++) {
        hor_angle -= 1;
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, hor_angle);
        HAL_Delay(SMALL_DELAY);
      }
    }
  }
 }

/**
  * @brief  This function for shouting.
  * @param None
  * @retval None
  */
 void GUNSHOOT() 
 {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);

  HAL_Delay(TIMEOFSHOT);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
 }

 /**
  * @brief  This function for playing music.
  * @param None
  * @retval None
  */
 void MUSIC_PLAY() 
 {
  uint8_t i = 10;
  for (; i < 30; i++) 
  {
    __HAL_TIM_SET_AUTORELOAD(&htim3, i*2);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, i);
    HAL_Delay(DELAY*2);
  }
  __HAL_TIM_SET_AUTORELOAD(&htim3, 1);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);

 }

/**
  * @brief  This function is for parse data from BlueTooth.
  * @param  None
  * @retval None
  */
 void rx_parse() 
 {
   buff[0] = rx_buff;
 }

/**
  * @brief  This function is for interrupting by BlueTooth.
  * @param UART number
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    rx_parse();
    HAL_UART_Receive_IT(&huart1,&rx_buff,1); // Enabling interrupt receive again
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
