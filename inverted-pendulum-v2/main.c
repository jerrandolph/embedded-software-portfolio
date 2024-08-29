/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// These PWM duty cycle counts correspond to the servo motor arm being in the -90 degree orientation and +90 degree orientation.
#define ARM_NEG_90_DEG (8190)
#define ARM_POS_90_DEG (2335)
#define ARM_RANGE (ARM_NEG_90_DEG - ARM_POS_90_DEG)

// Target rotation corresponds to the pendulum being balanced. It is set once the main blue button is pressed.
float targetRotation = 0;
float lastArmAngle = 0.0;
int isActive = 0;

// convertAngleToCount converts an angle in degrees to a PWM duty cycle clock count. The angle is capped to +/- 45 degrees to
// prevent the motor from damaging the pendulum.
int convertAngleToCount(float theta) {
	lastArmAngle = theta;
	if (theta < -45) {
		theta = -45;
	} else if (theta > 45) {
		theta = 45;
	}

	float scale = (90 - theta)/180;

	return (int)((float)ARM_RANGE * scale + (float)ARM_POS_90_DEG);
}

// setArmAngle moves the servo motor arm by converting the desired arm angle into a PWM duty cycle count.
void setArmAngle(float theta) {
	if (!isActive){
		return;
	}
	int newPulse = convertAngleToCount(theta);
	htim3.Instance->CCR1 = newPulse;
}

// zeroArmAngle resets the servo motor's arm to 0 degrees. This is used when the system is deactivated, so it ignores the 'isActive' flag.
void zeroArmAngle() {
	int newPulse = convertAngleToCount(0);
	htim3.Instance->CCR1 = newPulse;
}

// PID control parameters.
float kp = -10;
float ki = 0.0;
float kd = -20;

// These _comp variables are the last calculated PID components. They are set as global variables so they can be exported
// over UART.
float p_comp = 0.0;
float i_comp = 0.0;
float d_comp = 0.0;
float comp_total = 0.0;

float prevError;
// errorSum is used to calculate the I term.
float errorSum = 0.0;

// d_prev and filter_alpha are used to create a low pass filter for the D term.
//float d_prev = 0.0;
//float filter_alpha = .1;

void pidControlLoop(float value) {
	float error = value - targetRotation;

	errorSum += error;

	p_comp = kp * error;

	float d_raw = (prevError - error);
	//float d_filtered = filter_alpha * d_raw + (1-filter_alpha)*d_prev;
	//if (d_filtered != 0){
	//	int x = 0;
	//	x = x;
	//}
	d_comp = kd * d_raw;

	//d_comp = kd * (prevError - error);

	i_comp = errorSum * ki;

	comp_total = p_comp + i_comp + d_comp;
	setArmAngle(comp_total);
	prevError = error;
	//d_prev = d_filtered;
}

// transmit_uint32 sends an integer over UART.
void transmit_uint32(uint32_t data) {
	if (HAL_UART_Transmit(&huart2, (uint8_t*)&data,  sizeof(uint32_t),  1000) != HAL_OK) {
		Error_Handler();
	}
}

// transmit_float32 sends a float over UART.
void transmit_float32(float data) {
	if (HAL_UART_Transmit(&huart2, (uint8_t*)&data,  sizeof(uint32_t),  1000) != HAL_OK) {
		Error_Handler();
	}
}

// transmit_data sends a data packet over UART. The packet contains the given rotation and angle
void transmit_data(float encoder_rotation) {
	transmit_uint32(0xDEADBEEF);
	transmit_float32(encoder_rotation);
	transmit_float32(lastArmAngle);
	transmit_float32(p_comp);
	transmit_float32(i_comp);
	transmit_float32(d_comp);
	transmit_float32(comp_total);
}


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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // rotation contains the current rotation value of the rotary encoder.
  float rotation = 0;

  // buttonHeld is used to ensure that when the blue button is pressed, it only triggers 1 time.
  // TODO: Replace this with a proper interrupt handler.
  int buttonHeld = 0;

  // By default, set the servo motor's arm to 0 degrees.
  zeroArmAngle();
  int prevRotation = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
  while (1)
  {
	  // Figure out the angle of the pivot arm.
	  // The rotary encoder doesn't tell us an absolute rotation. Instead, it gives us a signal each
	  // time the pivot rotates by 1 click. The order of pins 0 and 1 tell us whether the rotation
	  // was in the positive direction or negative direction.
	  int aRotation = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	  if (aRotation != prevRotation) {
		  int bRotation = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		  if (bRotation == aRotation) {
			  rotation++;
		  } else {
			  rotation--;
		  }
	  }
	  prevRotation = aRotation;


	  // Send a data packet over UART.
	  transmit_data(rotation);

	  // Check to see if the blue button has been pressed. This resets the target rotation value and arms the system.
	  // Pressing the blue button again disables the system.
	  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if (state == GPIO_PIN_RESET && !buttonHeld) {
		  buttonHeld = 1;
		  targetRotation = rotation;
		  // Toggle whether the system si active.
		  isActive = !isActive;
		  if (!isActive) {
			  // If it's not active, reset the servo motor arm to 0 degrees.
			  zeroArmAngle();
		  }
	  } else if (state == GPIO_PIN_SET && buttonHeld){
		  buttonHeld = 0;
	  }


	  pidControlLoop(rotation);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 26;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62234;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
