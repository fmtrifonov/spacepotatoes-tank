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
#include<stdio.h>
#include<math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define horizontalSticCenter 0 // значение "0" по горизонтальной оси в приведенных значениях
#define verticalSticCenter 0 // значение "0" по вертикальной оси в приведенных значениях
#define k 20                 // коэффициент отклонения от центра
#define max_speed 300         // максимальное значение скорости. Используется для кодировки скорости
#define min_speed -300
#define PI 3.14159
#define YES 1
#define NO 0
#define FORWARD 1
#define BACKWARD -1
#define STAY 0
#define DELAY 50
#define speed_border 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //PA1 TIM2 CH2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //PA0 TIM2 CH1
  //int speed_right; // переменные, хранящие в вычислениях скорости левого и правого двигателей
  //int speed_left;
  //int speed, horizontal_value;  // переменные, хранящие среднюю скорость и скорость поворота
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);  //правый мотор
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0); //левый мотор
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*printf("Введите значение скорости правого мотора: \n");
    scanf("%d",speed_right);
    printf("Введите значение скорости левого мотора: \n");
    scanf("%d",speed_left);
    printf("Введите уровень скорости от 1 до 6\n");
    scanf("%c",control);
    printf("Введите действие:\nW - вперед\nS - назад\nA - влево\nD - вправо\nQ - влево вперед\nE - вправо вперед\nZ - влево назад\nC - вправо назад\n");
    scanf("%c",choice);

    if (control == '1') {     // Скорость 1.
      speed_left = 20;
      speed_right = 20;
    }
    if (control == '2') {     // Скорость 2
      speed_left = 50;
      speed_right = 50;
    }
    if (control == '3') {     // Скорость 3.
      speed_left = 80;
      speed_right = 80;
    }
    if (control == '4') {     // Скорость 4.
      speed_left = 120;
      speed_right = 120;
    }
    if (control == '5') {     // Скорость 5
      speed_left = 170;
      speed_right = 170;
    }
    if (control == '6') {     // Скорость 6.
      speed_left = 250;
      speed_right = 250;
    }

    if (choice == 'W') {   
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);  
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    }
    else if(choice == 'S'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    }
    else if(choice == 'A'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    }
    else if(choice == 'D'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    }
    else if(choice == 'Q'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

      speed_left = speed_left / 2;
    }
    else if(choice == 'E'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

      speed_right = speed_right / 2;
    }
    else if(choice == 'Z'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

      speed_left = speed_left / 2;
    }
    else if(choice == 'C'){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

      speed_right = speed_right / 2;
    }
    else{
      speed_left = 0;
      speed_right = 0;
    }

    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, speed_right);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, speed_left);*/
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
  htim2.Init.Prescaler = 127;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*--------------------функции получения значений координатам-----------------*/
int getXaxisData(uint8_t value)
{
	uint16_t angle=((value >> 3)*15);
	uint8_t radius=value&0x07;
	int x_value = (int)radius*((float)(cos((float)(angle*PI/180))));
	return x_value;
}

int getYaxisData(uint8_t value)
{
	uint16_t angle=((value >> 3)*15);
	uint8_t radius=value&0x07;
	int y_value= (int)radius*((float)(sin((float)(angle*PI/180))));
	return y_value;
}
/*-------------------- функция "зашифровки" значений скорости ---------------*/
void getValue(int x_value, int y_value, int *speed_left,int *speed_right){
  int speed = y_value * 35;
  if (speed < k && speed > -1*k) speed = 0;
  int horizontal_value = x_value * 35;
  if(horizontal_value < horizontalSticCenter - k){
    *speed_left = speed - (horizontalSticCenter - horizontal_value);
    *speed_right = speed + (horizontalSticCenter - horizontal_value);  
  }
  else if(horizontal_value > horizontalSticCenter + k){
    *speed_left = speed + (horizontalSticCenter - horizontal_value);
    *speed_right = speed - (horizontalSticCenter - horizontal_value);  
  }
  else{
    *speed_left = speed;
    *speed_right = speed;
  }
  if (*speed_left > max_speed) *speed_left = max_speed;
  if (*speed_right > max_speed) *speed_right = max_speed;
  if (*speed_left < min_speed) *speed_left = min_speed;
  if (*speed_right < min_speed) *speed_right = min_speed;
}
/*-------------------- функция управления двигателями ---------------*/
int MOTORS_MOVE(int speed_left, int speed_right) 
 {
  if (speed_left > speed_border) 
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, STAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, FORWARD);
  } else if (speed_left < speed_border) 
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, FORWARD);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, STAY);
  }
  if (speed_right > speed_border) 
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, STAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, FORWARD);
  } else if (speed_right < speed_border) 
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, FORWARD);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, STAY);
  }

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, speed_left);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, speed_right);

  uint32_t times = HAL_GetTick();
  while (HAL_GetTick() - times < DELAY);

 }
/*------------------ function for interrupting by BlueTooth -------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    switch (rx_buff) {
      /*case 'F': // motor forward
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
        break;*/
      case 'U': // servo up
        SERVO_MOVE(YES);
        break;
      case 'D': // servo down
        SERVO_MOVE(YES);
        break;
      case 'S': // servo left
        SERVO_MOVE(NO);
        break;
      case 'H': // servo right
        SERVO_MOVE(NO);
        break;
      case 'G': // GUNSHOT
        GUNSHOOT();
        break;
      case 'M': // MUSIC
        MUSIC_PLAY();
        break;
      default:
        int x_value = getXaxisData(rx_buff);
        int y_value = getYaxisData(rx_buff);
        int speed_left, speed_right;
        getValue(x_value,y_value,&speed_left,&speed_right);
        MOTORS_MOVE(speed_left,speed_right);
        break;
    }
    HAL_UART_Receive_IT(&huart1,rx_buff,BUFF_SIZE); // Enabling interrupt receive again
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
