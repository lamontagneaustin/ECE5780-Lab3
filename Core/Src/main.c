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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR  |= (RCC_AHBENR_GPIOAEN)|(RCC_AHBENR_GPIOCEN); // Enables the GPIOA/GPIOC clock in the RCC.
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN) | (RCC_APB1ENR_TIM3EN); // Enables the TIM2/TIM3 clock in the RCC.
	
	// Configures GPIOC Pins 8 and 9 (ORANGE LED and GREEN LED)
	GPIOC->MODER   |=  (1 << 16) | (1 << 18);
	GPIOC->OTYPER  &= ~((1 << 8) | (1 << 9));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 18));
	GPIOC->PUPDR   &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	
	// Configures GPIOC Pins 6 and 7 (RED LED and BLUE LED)
	GPIOC->MODER   |=  (1 << 13) | (1 << 15);
	GPIOC->MODER   &=  ~((1 << 12) | (1 << 14));
	GPIOC->AFR[0] &= ~((1 << 24)|(1 << 25)|(1 << 26)|(1 << 27));
	GPIOC->AFR[0] &= ~((1 << 28)|(1 << 29)|(1 << 30)|(1 << 31));
	
	// Configures TIM2 Peripheral for Interrupt.
	TIM2->PSC   = (0x4E1F); // 19999 PSC which divides by 20000 on 8MHz Clock.
	TIM2->ARR   = (0x64); // 100 ARR value which makes the interrupt 4.00 Hz.
	TIM2->DIER |= (1 << 0); // Enables Update Interrupt.
	TIM2->CR1  |= (1 << 0); // Counter enable.
	
	// Configures TIM3 Peripheral for PWM.
	TIM3->PSC    =  (0x63); // Sets PSC to 99.
	TIM3->ARR    =  (0x64); // Sets ARR to 100.
	TIM3->CCMR1 &= ~((1 << 0)|(1 << 1));
	TIM3->CCMR1 &= ~((1 << 8)|(1 << 9));
	TIM3->CCMR1 |=  (1 << 4)|(1 << 5)|(1 << 6);
	TIM3->CCMR1 |=  (1 << 13)|(1 << 14);
	TIM3->CCMR1 &= ~(1 << 12);
	TIM3->CCMR1 |=  (1 << 3)|(1 << 11); // Enable Compare 1 and 2 preload.
	
	TIM3->CCER |= (1 << 0)|(1 << 4); // Set output enable bits for channels 1 and 2.
	TIM3->CCR1  = (0x5F);            // 20% of ARR, ARR=100 so CCR1=20
	TIM3->CCR2  = (0x5);            // 20% of ARR, ARR=100 so CCR2=20
	
	TIM3->CR1  |= (1 << 0); // Counter enable.
	
	
	NVIC_EnableIRQ(TIM2_IRQn);
	//NVIC_SetPriority(TIM2_IRQn, 1);
	
	GPIOC->BSRR |= (1 << 8); // Start PC8 (ORANGE) High.
  while (1)
  {
		/*
		HAL_Delay(1000);
		// Toggle Pin PC6 (RED).
		if(GPIOC->IDR & 0x40){
			GPIOC->BSRR |= (1 << 22); // Resets State of PC6.
		}
		else{
			GPIOC->BSRR |= (1 << 6); // Sets State of PC6.
		}
		*/
  }
}

/**
  * @brief  TIM2 Interrupt Handler.
  */
void TIM2_IRQHandler(void) {
	
	// Toggle Pin PC8 (ORANGE).
	if(GPIOC->IDR & 0x100){
		GPIOC->BSRR |= (1 << 24); // Resets State of PC8.
	}
	else{
		GPIOC->BSRR |= (1 << 8); // Sets State of PC8.
	}
	
	// Toggle Pin PC9 (GREEN).
	if(GPIOC->IDR & 0x200){
		GPIOC->BSRR |= (1 << 25); // Resets State of PC9.
	}
	else{
		GPIOC->BSRR |= (1 << 9); // Sets State of PC9.
	}
	
	TIM2->SR &= ~(1 << 0);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
