/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define DUTY_CYCLE 2000 //unused, can prolly delete
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//feedback variables, used with interrupt handler to get incremental feedback from motor(s)
int duration;
int Direction;
int YellowsigLast;

int errorterm; //The difference between the current position and the desired position.

//PID Control terms, adjust as needed
int PropTerm = 10; //proportional term (30?)
float IntTerm = 0; //Integral term
float DerivTerm = 0.1; //Derivative term

int FeedbackControlSignal; //final summation of effects of P, I and D controllers

//Proportional control support variables
int Pterm;

//Integral control support variables
int IntArray[30]; //= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int IntegralTerm;
int ITerm;
int sum;

//Derivative control support variables
int DerArray[30]; // = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int DerivativeTerm;
int DTerm;
int numerator;
int denominator;
int slope;


int DesiredPos; //desired final position of the actuator

int GoalTerm = 1; //sometimes overshoots, leading to undefined behavior. This prevents movement once goal is reached.
int i;
int j;
int k;
int f;
int z;

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
  Direction = 1; //Problems with global variables?
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_MspInit(&htim2);
  HAL_TIM_Base_Start(&htim2); //Starts the TIM Base generation

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DUTY_CYCLE);

  HAL_TIM_MspPostInit(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //PA15 motor pwm, PC2 And PC3 motor binary direction.
	  //PA9 and PA10 are incremental motor feedback

	  //MotorDirectionDetermination();

	  DesiredPos = 8669;  //give offset (360 deg ~ 8669)

	  errorterm = duration - DesiredPos;

	  //control (P control)
	  Pterm = PropTerm*errorterm;

	  //Integral control (I control)
	  ITerm = IntegralControl(errorterm);

	  //Derivative control (D control)
	  DTerm = DerivativeControl(errorterm);

	  //Summing effects of PID controller
	  FeedbackControlSignal = Pterm+ITerm+DTerm;

	  if (FeedbackControlSignal > 60000 || FeedbackControlSignal < -60000){
		  FeedbackControlSignal = 60000;
	  }


	  if(errorterm > 120) { //24 counts per degree
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //Forward or CW (Negative feedback)
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(FeedbackControlSignal));
		 HAL_Delay(10);
	  }
	  if(errorterm < -120) {
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //Backwards or CCW (Positive feedback)
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(FeedbackControlSignal));
		 HAL_Delay(10);
	  }else {
	 		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	 		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
		     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	 		 //GoalTerm = 0;
	 		 HAL_Delay(500);
	 	  }


	        /*
	         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //Forward or CW (Negative feedback)
	  		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	  		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 60000);
	  		 HAL_Delay(3000);

	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //Backwards or CCW (Positive feedback)
	  		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	  		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 60000);
	  		  HAL_Delay(1000);

	  		  */







	  //PWM LED CODE****************************************************
	  /*
	  int i;

	  for (i = 0; i < 65535; i=i+500) {

	  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);

	  	HAL_Delay(10);

	  }*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Motorfeedback should read the yellow and white wires, and determine the motor speed,
//incremental position and direction from the 2 square waves.
/*
void motorfeedback(){

	int Yellowsig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//digitalRead(encoder0pinA);
	 if((YellowsigLast == 0) && Yellowsig == 1)
	 {
	   int Whitesig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12); //digitalRead(encoder0pinB);
	   if(Whitesig == 0 && Direction)
	   {
	     Direction = 0; //Reverse/false
	   }
	   else if(Whitesig == 1 && Direction == 0)
	   {
	     Direction = 1;  //Forward/true
	   }
	 }
	 YellowsigLast = Yellowsig;

	 if(Direction == 0)  duration++;
	 else  duration--;
	 printf("%d", duration); //see feedback via console
}
*/

int IntegralControl(int errorterm) {

	for (j = 29; j > 0; j--) { //moving down the array terms, with the oldest being 10 and youngest 0
		IntArray[j]=IntArray[j-1];
	}
	IntArray[0] = errorterm;

	sum = 0;
	for(k = 0; k < 29; k++) { //summing the entire array
		sum = sum + IntArray[k]; // same as sum += arr[i];
	}

	IntegralTerm = IntTerm*sum;

	return IntegralTerm;
}

int DerivativeControl(int errorterm) {

	for (f = 29; f > 0; f--) { //moving down the array terms, with the oldest being 29 and youngest 0
		DerArray[f]=DerArray[f-1];
	}
	DerArray[0] = errorterm;

	//rough approximation of slope by dividing average of first 15 errorterms (numerator)
	//by the average of the last 15 errorterms (denominator).
	numerator = 0;
	denominator = 0;
	for(z = 0; z < 29; z++) {
		if(z < 15) {
			numerator = numerator + DerArray[z];
		}
		numerator = numerator/5;

		if(z >= 15) {
			denominator = denominator + DerArray[z];
		}
		denominator = denominator/5;

		if (denominator == 0) {
			denominator = 1; //preventing possibility of divide by zero error
		}

		slope = numerator/denominator;

	}
	DerivativeTerm = DerivTerm*slope;

	return DerivativeTerm;
}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	//if(GPIO_Pin == GPIO_PIN_11 || GPIO_Pin == GPIO_PIN_12)
    //{

    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); // Toggle The Output (LED) Pin

	/*
		YellowsigLast = Yellowsig;
		Yellowsig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//digitalRead(encoder0pinA);
    	int Whitesig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11); //Moving up measurement in an attempt to stop intermittent incorrect direction.

    	if(YellowsigLast == 0 && Yellowsig == 1)
    	{
    		 if(Whitesig == 0 && Direction == 1) {
    		    Direction = 0; //Reverse/false
    		 }
    		 else if(Whitesig == 1 && Direction == 0) {
    		    Direction = 1;  //Forward/true
    		 }
    	}
    	//YellowsigLast = Yellowsig;
		*/
    	if(Direction == 0) {
    		duration++;
    	}
    	else {
    		duration--; //original was just: else duration--;
    	}


    	//printf("%d", duration); //see feedback via console
    	// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duration); //seeing if changes take effect




    //}

}
/*
void MotorDirectionDetermination() {

	int Yellowsig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//digitalRead(encoder0pinA);
	int Whitesig = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11); //Moving up measurement in an attempt to stop intermittent incorrect direction.

	if(YellowsigLast == 0 && Yellowsig == 1)
	{
	    if(Whitesig == 0 && Direction == 1) {
	    	Direction = 0; //Reverse/false
	    }
	    else if(Whitesig == 1 && Direction == 0) {
	    	Direction = 1;  //Forward/true
	    }
	}

	YellowsigLast = Yellowsig;
}*/
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
