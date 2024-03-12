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

void gpio_init(void)
{
	/* Initialize PB11 */
	// Set to alternate function mode
	GPIOB->MODER &= ~(1<<22);
	GPIOB->MODER |= (1<<23);
	// Set to open-drain output type
	GPIOB->OTYPER |= (1<<11);
	// Set I2C2_SDA as alternate function
	GPIOB->AFR[1] |= (1<<12);
	GPIOB->AFR[1] &= ~(1<<13);
	GPIOB->AFR[1] &= ~(1<<14);
	GPIOB->AFR[1] &= ~(1<<15);
	
	/* Initialize PB13 */
	// Set to alternate function mode
	GPIOB->MODER &= ~(1<<26);
	GPIOB->MODER |= (1<<27);
	// Set to open-drain output type
	GPIOB->OTYPER |= (1<<13);
	// Set I2C2_SCL as alternate function
	GPIOB->AFR[1] |= (1<<20);
	GPIOB->AFR[1] &= ~(1<<21);
	GPIOB->AFR[1] |= (1<<22);
	GPIOB->AFR[1] &= ~(1<<23);
	
	/* Initialize PB14 */
	// Set to output mode
	GPIOB->MODER |= (1<<28);
	GPIOB->MODER &= ~(1<<29);
	// Set to push-pull output type
	GPIOB->OTYPER &= ~(1<<14);
	// Initialize high
	GPIOB->ODR |= (1<<14);
	
	/* Initialize PC0 */
	// Set to output mode
	GPIOC->MODER |= (1<<0);
	GPIOC->MODER &= ~(1<<1);
	// Set to push-pull output type
	GPIOC->OTYPER &= ~(1<<0);
	// Initialize high
	GPIOC->ODR |= (1<<0);
	
	/* Setup Orange LED (PC8) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<16);
	GPIOC->MODER &= ~(1<<17);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<8);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<16);
	GPIOC->OSPEEDR &= ~(1<<17);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	// Initialize to low
	GPIOC->ODR &= ~(1<<8);
	
	/* Setup Green LED (PC9) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<18);
	GPIOC->MODER &= ~(1<<19);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<9);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->OSPEEDR &= ~(1<<19);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
	// Initialize to low
	GPIOC->ODR &= ~(1<<9);
}

void i2c_init(void)
{
	/* Set I2C to use 100kHz standard I2C */
	// Set PRESC to 1
	I2C2->TIMINGR |= (1<<28);
	I2C2->TIMINGR &= ~(1<<29);
	I2C2->TIMINGR &= ~(1<<30);
	I2C2->TIMINGR &= ~(1<<31);
	// Set SCLDEL to 0x4
	I2C2->TIMINGR |= (1<<22);
	// Set SDADEL to 0x2
	I2C2->TIMINGR |= (1<<17);
	// Set SCLH to 0xF
	I2C2->TIMINGR |= (1<<8);
	I2C2->TIMINGR |= (1<<9);
	I2C2->TIMINGR |= (1<<10);
	I2C2->TIMINGR |= (1<<11);
	// Set SCLL to 0x13
	I2C2->TIMINGR |= (1<<4);
	I2C2->TIMINGR |= (1<<1);
	I2C2->TIMINGR |= (1<<0);
	
	/* Enable I2C2 */
	I2C2->CR1 |= (1<<0);
}

void i2c_write(void)
{
	/* Configure I2C to write */
	// Set the slave address to 0x6A
	I2C2->CR2 |= (0x69<<1);
	// Set the number of bytes to transmit to 1
	I2C2->CR2 |= (1<<16);
	// Set to write operation
	I2C2->CR2 &= ~(1<<10);
	// Set the start bit
	I2C2->CR2 |= (1<<13);
}

void i2c_read(void)
{
	/* Configure I2C to read */
	// Set the slave address to 0x6A
	I2C2->CR2 |= (0x69<<1);
	// Set the number of bytes to transmit to 1
	I2C2->CR2 |= (1<<16);
	// Set to write operation
	I2C2->CR2 |= I2C_CR2_RD_WRN;
	// Set the start bit
	I2C2->CR2 |= (1<<13);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	gpio_init();
	i2c_init();
	
	i2c_write();
	
	while (1)
	{
		if (I2C2->ISR & (1<<1))
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {}
	}
	
	I2C2->TXDR |= 0x0F;
	
	while (1)
	{
		if (I2C2->ISR & (1<<6))
		{
			break;
		}
	}
	
	i2c_read();
	
	while (1)
	{
		if (I2C2->ISR & (1<<2))
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {}
	}
	
	while (1)
	{
		if (I2C2->ISR & (1<<6))
		{
			break;
		}
	}
	
	if (I2C2->RXDR == 0xD3)
	{
			I2C2->CR2 |= (1<<14);
	}
	
  while (1) {}
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
