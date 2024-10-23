

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
void Led_Init(void);
void Led_On(void);
void Led_Off(void);

void Input_A6(void);

void time1(void);

void Timer10_Init(void);
void Timer10_Enable(void);
void Timer10_Disable(void);
void Led_Init_PB8 (void);


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t counter1=0;
uint16_t status1=0;
uint16_t repeat=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  Led_Init();
  Input_A6();
  Timer10_Init();
  Timer10_Enable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  counter1=TIM10->CNT;
	  status1 = GPIOA->IDR & GPIO_IDR_ID6;
	  if(status1 == 64)
	  {
		  GPIOD->ODR |=GPIO_ODR_OD15;
	  }
	  else
	  {
		  GPIOD->ODR &=~(GPIO_ODR_OD15);
	  }
	  //status1=TIM10->SR;
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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Led_Init()
{
	RCC->AHB1ENR |=RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |=GPIO_MODER_MODER15_0;
	GPIOD->MODER &=~(GPIO_MODER_MODE15_1);
	GPIOD->OTYPER &=~(GPIO_OTYPER_OT15);
	GPIOD->OSPEEDR &=~(GPIO_OSPEEDER_OSPEEDR15_0);
	GPIOD->OSPEEDR &=~(GPIO_OSPEEDER_OSPEEDR15_1);
	GPIOD->PUPDR &=~(GPIO_PUPDR_PUPD15_0);
	GPIOD->PUPDR &=~(GPIO_PUPDR_PUPD15_1);

}

void Input_A6(void)
{
	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER &=~(GPIO_MODER_MODER6_0);
	GPIOA->MODER &=~(GPIO_MODER_MODER6_1);
	GPIOA->PUPDR &=~(GPIO_PUPDR_PUPD6_0);
	GPIOA->PUPDR &=~(GPIO_PUPDR_PUPD6_1);
	GPIOA->IDR = 0;

}

void Led_On()
{
	GPIOD->ODR |=GPIO_ODR_OD15;
}

void Led_Off()
{
	GPIOD->ODR &=~(GPIO_ODR_OD15);
}

void time1()
{
	for (int var = 0; var < 47000; ++var) {

	}
}

void Timer10_Init(void){

		Led_Init_PB8();
	//Enable TIM10 Clock
	__HAL_RCC_TIM10_CLK_ENABLE();

	// Timer Clock  48 Mhz / 48.000 = 1000 Hz ( 1ms period )
	TIM10->PSC=47999;
	//Output Compare Mode
	TIM10->CCMR1 &=~(TIM_CCMR1_CC1S_0);
	TIM10->CCMR1 &=~(TIM_CCMR1_CC1S_1);

	// Output Compare Mode - Toggle on match
	TIM10->CCMR1 |=TIM_CCMR1_OC1M_0;
	TIM10->CCMR1 |=TIM_CCMR1_OC1M_1;
	TIM10->CCMR1 &=~(TIM_CCMR1_OC1M_2);

	// Enable OC1REF Output
	TIM10->CCER |=TIM_CCER_CC1E;

	//Output Polarity : Active High
	TIM10->CCER &=~(TIM_CCER_CC1P);


	// Reload/Set in every 500 ms
	TIM10->ARR=4000;
	TIM10->CCR1=4000;

	//TIM10->DIER |=TIM_DIER_UIE;

	// Enable TIM10 Interrupt on NVIC
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		NVIC_SetPriority(TIM1_UP_TIM10_IRQn,2);

}

void Timer10_Enable(void)
{

	TIM10->CR1 |=TIM_CR1_CEN;
}

void Timer10_Disable(void)
{
	TIM10->CR1 &=~(TIM_CR1_CEN);
}

void TIM1_UP_TIM10_IRQHandler()
{
	TIM10->SR &=~(TIM_SR_UIF);
	time1();
	repeat=repeat+1;
	GPIOD->ODR ^=GPIO_ODR_OD15;


}

void Led_Init_PB8 (void)
{
	// Output GPIOB AF3 High AFRH

	// Enable Clock A port
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Select Mode : Alternate Function
	GPIOB->MODER &=~(GPIO_MODER_MODE8_0);
	GPIOB->MODER |=GPIO_MODER_MODE8_1;

	// 0:Output push-pull
	GPIOB->OTYPER &=~(GPIO_OTYPER_OT8);

	// Select Speed 01: Medium speed
	GPIOB->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR8_0;
	GPIOB->OSPEEDR &=~(GPIO_OSPEEDER_OSPEEDR8_1);

	// 00: No pull-up, pull-down
	GPIOB->PUPDR &=~(GPIO_PUPDR_PUPD8_0);
	GPIOB->PUPDR &=~(GPIO_PUPDR_PUPD8_1);

	//  Alternate function selection GPIOB8 için AF3 AFRH selection: 0011: AF3
	GPIOB->AFR[1] |=GPIO_AFRH_AFSEL8_0;
	GPIOB->AFR[1] |=GPIO_AFRH_AFSEL8_1;
	GPIOB->AFR[1] &=~(GPIO_AFRH_AFSEL8_2);
	GPIOB->AFR[1] &=~(GPIO_AFRH_AFSEL8_3);



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
