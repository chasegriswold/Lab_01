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

//	---------- Uncomment for First checkoff ----------
//	
///*blue and red LEDs are attached to pins 6 & 7 respectively*/	
//	
//// (1) Enable the peripheral clock of GPIOC 
//	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */
//	// Input-Output
//	/* Set Red LED On*/
//	GPIOC->MODER |= (1<<12); // Sets the 12th bit = 1
//	GPIOC->MODER &= ~(1<<13); // Set 13th bit = 0
//	/* Set Blue LED On*/
//	GPIOC->MODER |= (1<<14); // Sets the 14th bit = 1
//	GPIOC->MODER &= ~(1<<15); // Set 15th bit = 0
//	
//	// Push-Pull
//	/* Set Red LED push-pull*/
//	GPIOC->OTYPER &= ~(1<<6); // Sets the 6th bit = 0 -- NOTE: 0 sets push pull in OTYPER
//	/* Set Blue LED On*/
//	GPIOC->OTYPER &= ~(1<<7); // Sets the 7th bit = 0 -- NOTE: 0 sets push pull in OTYPER
//	
//	// Set Speed
//	//x0: Low speed (first bit is a don't care)
//		/* Set Red LED Speed */
//	GPIOC->OSPEEDR &= ~(1<<12); // Sets the 12th bit = 0 -- NOTE: Next bit does not matter
//	/* Set Blue LED Speed */
//	GPIOC->OSPEEDR &= ~(1<<14); // Sets the 14th bit = 0 -- NOTE: Next bit does not matter
//	
//		/* Set Red LED no-pullupdown */
//	GPIOC->PUPDR &= ~(1<<12); // Sets the 12th bit = 0
//	GPIOC->PUPDR &= ~(1<<13); // Sets the 13th bit = 0
//	/* Set Blue LED no-pullupdown */
//	GPIOC->PUPDR &= ~(1<<14); // Sets the 14th bit = 0
//	GPIOC->PUPDR &= ~(1<<15); // Sets the 15th bit = 0
//	
//	SystemClock_Config(); //Configure the system clock
//	/* The above is for the HAL_delay function */

//	while (1) {
//		
//		HAL_Delay(400); // Delay 400ms
//		
//			/*set red high and blue low*/
//			/* Set Red LED High */
//			GPIOC->ODR |= (1<<6); // Sets the 6th bit = 1 
//			/* Set Blue LED Low */
//			GPIOC->ODR &= ~(1<<7); // Sets the 7th bit = 0
//		
//		HAL_Delay(400); // Delay 400ms
//		
//		/*toggle*/
//			/*set red low and blue high*/
//			/* Set Red LED High */
//			GPIOC->ODR &= ~(1<<6); // Sets the 6th bit = 0 
//			/* Set Blue LED Low */
//			GPIOC->ODR |= (1<<7); // Sets the 7th bit = 1		
//		
//	}
//	
// ------------ End of first checkoff -----------


// ---------- Checkoff Part 2 -----------

/*blue and red LEDs are attached to pins 6 & 7 respectively*/	
	
// (1) Enable the peripheral clock of GPIOC 
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */
	// Input-Output
	/* Set Red LED On*/
	GPIOC->MODER |= (1<<12); // Sets the 12th bit = 1
	GPIOC->MODER &= ~(1<<13); // Set 13th bit = 0
	/* Set Blue LED On*/
	GPIOC->MODER |= (1<<14); // Sets the 14th bit = 1
	GPIOC->MODER &= ~(1<<15); // Set 15th bit = 0
	
	// Push-Pull
	/* Set Red LED push-pull*/
	GPIOC->OTYPER &= ~(1<<6); // Sets the 6th bit = 0 -- NOTE: 0 sets push pull in OTYPER
	/* Set Blue LED On*/
	GPIOC->OTYPER &= ~(1<<7); // Sets the 7th bit = 0 -- NOTE: 0 sets push pull in OTYPER
	
	// Set Speed
	//x0: Low speed (first bit is a don't care)
		/* Set Red LED Speed */
	GPIOC->OSPEEDR &= ~(1<<12); // Sets the 12th bit = 0 -- NOTE: Next bit does not matter
	/* Set Blue LED Speed */
	GPIOC->OSPEEDR &= ~(1<<14); // Sets the 14th bit = 0 -- NOTE: Next bit does not matter
	
		/* Set Red LED no-pullupdown */
	GPIOC->PUPDR &= ~(1<<12); // Sets the 12th bit = 0
	GPIOC->PUPDR &= ~(1<<13); // Sets the 13th bit = 0
	/* Set Blue LED no-pullupdown */
	GPIOC->PUPDR &= ~(1<<14); // Sets the 14th bit = 0
	GPIOC->PUPDR &= ~(1<<15); // Sets the 15th bit = 0
	
	SystemClock_Config(); //Configure the system clock
	/* The above is for the HAL_delay function */

//This is technically the start of Checkoff part 2, but we needed to initialize the LEDs for this code as well.
// Pushbuttons B1 USER: User and Wake-Up button connected to the I/O PA0 of the
//this is on the GPIOA bus since it's PA0

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //  Analog GPIO configuration
	
	/* Set PA0 to input mode (bits 0 and 1 of GPIOA) On*/
	GPIOA->MODER = 0x28000000; // This is the reset state for GPIOA datasheet page 158
	// the above code is a cleaner way to set it to input mode. The datasheet calls out bits 0 and 1 both be set to 0 for input mode, but above code accomplishes the same thing.
	
	//Set the pins to low speed
	GPIOA->OSPEEDR &= ~(1<<0); // Sets the 0th bit = 0 -- NOTE: Next bit is a don't care
	
	//Enable the pull-down resistor --
	GPIOA->PUPDR |= 2; // Since pull down is 10 for bits 1 and 0 respectively (2 in decimal) we can just set this register to = 2.
	
	GPIOC->ODR |= (1<<6); // 6th bit = 1
	GPIOC->ODR &= ~(1<<7); // 7th bit = 0
	
	uint32_t debouncer = 0;
	while(1) {
		debouncer = (debouncer << 1); // Always shift every loop iteration
		if (GPIOA->IDR & 0x01) { // If input signal is set/high //NOTE: the number 0x01 is literally bit 0 and this checks if it reads a high.
		debouncer |= 0x01; // Set lowest bit of bit-vector
		}
		if (debouncer == 0xFFFFFFFF) {
		// This code triggers repeatedly when button is steady high!
		}
		if (debouncer == 0x00000000) {
		// This code triggers repeatedly when button is steady low!
		}
		if (debouncer == 0x7FFFFFFF) {
		// This code triggers only once when transitioning to steady high!
				GPIOC->ODR ^= (1<<6); // 6th bit = 0
				GPIOC->ODR ^= (1<<7); // 7th bit = 0
		}
		// When button is bouncing the bit-vector value is random since bits are set when
		//the button is high and not when it bounces low.
		HAL_Delay(2); // Delay 2ms to avoid errors, see lab manual
	}
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
