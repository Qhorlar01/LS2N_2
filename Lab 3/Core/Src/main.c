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
static float measure = 0;
unsigned char obstacle = 0;
volatile int timeout_flag = 0; // Flag to indicate timeout

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void setup()

{

	// GPIO setup

	RCC->AHBENR |= 0x1<<17; //GPIOA enable

	// Pin mode

	GPIOA -> MODER &= ~(0x03<<20); // reset bit PA10

	GPIOA -> MODER |= (0x01<<20); //PA10 as an output

	// Timer setup

	RCC->APB1ENR |= 0x1<<4; //TIM6 timer clock enabled

	//reset peripherals
	RCC->APB1RSTR |= 0x1<<4;
	RCC->APB1RSTR &= ~(ox1<<4);
	__asm("nop");

	// Setting 10Hz for TIM6

	TIM6->ARR = 100-1; // 100 * 1ms = 100ms = 10 Hz
	TIM6->PSC = 64000-1; // prescaler : tick@1ms
	TIM6->CR1 |= 0x1<<0; // control reg : enable timer6  Counter

	// Interrupt setup
	TIM6->DIER |= 0x1<<0;            // Enable update interrupt
	NVIC_EnableIRQ(TIM6_DAC1_IRQn);  // Enable TIM6 interrupt

	// Timer setup TIM7

	RCC->APB1ENR |= 0x1<<5; //enable peripheral clock
	//__asm("nop");
	RCC->APB1RSTR |= 0x1<<5;
	RCC->APB1RSTR &= ~(0x1<<5);
	//__asm("nop");

	// Setting 50ms for TIM7 = 2Hz

	TIM7->ARR = 50000-1; // 50.000 * 1us = 50ms
	TIM7->PSC = 64-1; // prescaler : tick@1u
	TIM7->CR1 |= (1<<3); //Set one-pulse mode

	// Interrupt setup TIM7

	TIM7->DIER |= TIM_DIER_UIE; // Interrupt enable register - UIE = Update Interrupt enable
	NVIC_EnableIRQ(TIM7_DAC2_IRQn); // to reset the timer in case no object is detected

	// Interrupt setup EXTI15

	RCC -> APB2ENR |= 0x1<<0; // enable peripheral clock
	EXTI -> IMR |= (0x1<<10);//enable at least one external line
	SYSCFG -> EXTICR[3] &= ~(0x0F<<8); // assign external interrupt configuration register to Port A
	EXTI -> RTSR |= (1<<10); // Set rising edge sensitivity for pin 10
	EXTI -> FTSR |= (1<<10); // Set falling edge sensitivity for pin 10

	// check pin level after interrupt to check if it was rising or falling edge
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

	// Timer 6 interrupt handler
	void TIM6_DAC1_IRQHandler()
{
    TIM6->SR &= ~TIM_SR_UIF;        // Clear TIM6 update interrupt flag
    EXTI->IMR &= ~EXTI_IMR_MR10;    // Disable EXTI interrupt on PA10

    // Configure PA10 as output
    GPIOA->MODER &= ~(0x03 << 20);
    GPIOA->MODER |= (0x01 << 20);

    // Trigger request at 10 Hz with 10us high state
    GPIOA->ODR |= (0x1 << 10);
    for (volatile int i = 0; i < 20; i++);
    GPIOA->ODR &= ~(0x1 << 10);

    // Configure PA10 as input
    GPIOA->MODER &= ~(0x03 << 20);

    GPIOA->PUPDR &= ~(0x03 << 20); // PA10 as Pull-down to detect rising edge
    GPIOA->PUPDR |= (2 << 20);

    EXTI->IMR |= EXTI_IMR_MR10;     // Enable EXTI interrupt on PA10
}

	// EXTI15_10 interrupt handler
	void EXTI15_10_IRQHandler()
	{
	    EXTI->PR |= (1 << 10);  // Clear pending interrupt on PA10

	    if ((GPIOA->IDR & (0x01 << 10)) == 1)
	    {
	        TIM7->CNT = 0;
	        TIM7->CR1 |= TIM_CR1_CEN;  // Start timer 7

	        // Configure PA10 as Pull-up to detect falling edge
	        GPIOA->PUPDR &= ~(0x03 << 20);
	        GPIOA->PUPDR |= (0x01 << 20);
	    }

	    if ((GPIOA->IDR & (1 << 10)) == 0)
	    {
	        TIM7->CR1 &= ~(1 << 0);  // Stop timer 7
	        measure = TIM7->CNT;     // Store measure value
	        // Calculate the distance between the obstacle and the Ultrasonic Sensor
	        int distance_mm = (int)(measure * 17.24);  // Convert measure to mm
	        obstacle = 1;
	    }
	}

	// Timer 7 interrupt handler
	void TIM7_DAC2_IRQHandler()
	{
	    TIM7->SR &= ~TIM_SR_UIF;   // Clear TIM7 update interrupt flag
	    TIM7->CR1 &= ~TIM_CR1_CEN; // Stop TIM7

	    obstacle = 0;              // Reset obstacle status
	    timeout_flag = 1;          // Set the timeout flag
	}

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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

	  setup();  // Setup function

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      if (timeout_flag)
      {
          printf("Timeout: Sensor not available\n");
          timeout_flag = 0; // Reset the timeout flag
      }
      else
      {
          printf("Distance: %d mm\n", (int)(measure * 17.24));  // Display distance
      }

      // Additional logic or delay as needed
      for (volatile int i = 0; i < 1000000; i++);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
