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
#include <stdlib.h>

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
	
	/* Setup Red LED (PC6) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<12);
	GPIOC->MODER &= ~(1<<13);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<6);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<13);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	// Initialize to low
	GPIOC->ODR &= ~(1<<6);

	/* Setup Blue LED (PC7) */
	// Set to general purpose output mode
	GPIOC->MODER |= (1<<14);
	GPIOC->MODER &= ~(1<<15);
	// Set to push-pull mode
	GPIOC->OTYPER &= ~(1<<7);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1<<14);
	GPIOC->OSPEEDR &= ~(1<<15);
	// Set no pull-up, no pull-down
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	// Initialize to low
	GPIOC->ODR &= ~(1<<7);
	
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

void tc_flag_rec(void)
{
	while (1)
	{
		if (I2C2->ISR & (1<<6)) // Wait for TC
		{
			return;
		}
	}
}

void i2c_write(int wr_val)
{
	/* Configure I2C to write */
	I2C2->CR2 = 0;
	// Set the slave address to 0x69
	I2C2->CR2 |= (0x69<<1);
	// Set the number of bytes to transmit to 1
	I2C2->CR2 |= (1<<16);
	// Set to write operation
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;
	// Set the start bit
	I2C2->CR2 |= (1<<13);
	
	while (1)
	{
		if (I2C2->ISR & (1<<1)) // Wait for TXIS
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {} // Check for NACK
	}	
	
	I2C2->TXDR = wr_val; // Write next data byte in TXDR

	tc_flag_rec();
}

int i2c_read(void)
{
	int rec_val = 0;
	/* Configure I2C to read */
	I2C2->CR2 = 0;
	// Set the slave address to 0x69
	I2C2->CR2 |= (0x69<<1);
	// Set the number of bytes to transmit to 1
	I2C2->CR2 |= (1<<16);
	// Set to read operation
	I2C2->CR2 |= I2C_CR2_RD_WRN;
	// Set the start bit
	I2C2->CR2 |= (1<<13);
	
	while (1)
	{
		if (I2C2->ISR & (1<<2)) // Wait for RXNE
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {} // Check for NACK
	}
	
	rec_val = I2C2->RXDR; // Read received data byte
	
	tc_flag_rec();
	
	return rec_val;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	int rec_val = 0;
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	gpio_init();
	
	/* Part 1 */
	//i2c_init();
	
	//i2c_write(0x0F);
	
	//rec_val = i2c_read();
		
	//if (rec_val == 0xD3)
	//{
	//		I2C2->CR2 |= (1<<14); // STOP
	//}
	
	/* Part 2 */
	// Reinitialize I2C
	i2c_init();
	
	/* Configure I2C to write */
	I2C2->CR2 = 0;
	// Set the slave address to 0x69
	I2C2->CR2 |= (0x69<<1);
	// Set the number of bytes to transmit to 1
	I2C2->CR2 |= (2<<16);
	// Set to write operation
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;
	// Set the start bit
	I2C2->CR2 |= (1<<13);
	
	while (1)
	{
		if (I2C2->ISR & (1<<1)) // Wait for TXIS
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {} // Check for NACK
	}	
	
	I2C2->TXDR = 0x20; // Write next data byte in TXDR
	
	while (1)
	{
		if (I2C2->ISR & (1<<1)) // Wait for TXIS
		{
			break;
		}
		else if (I2C2->ISR & (1<<4)) {} // Check for NACK
	}	
	
	I2C2->TXDR = 0x0B;
	
	tc_flag_rec();
	
	/* Initialize values for X and Y */
	int16_t x;
	int16_t y;
	int16_t threshold = 0x0FFF;
	
  while (1) 
	{
		i2c_write(0x28);
		x = i2c_read();
		i2c_write(0x29);
		x = (i2c_read() << 8);
		
		i2c_write(0x2A);
		y = i2c_read();
		i2c_write(0x2B);
		y = (i2c_read() << 8);
		
		// Check which has the larger magnitude
		if (abs(x) > abs(y)) // If x has the larger magnitude
		{
			if (x > 0) // If x is positive
			{
				// x+ is up
				if (abs(x) > threshold)
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR |= (1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
				else
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
			}
			else
			{
				// x- is up
				if (abs(x) > threshold)
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR |= (1<<9);
				}
				else
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
			}
		}
		else // If y has the larger magnitude
		{
			if (y > 0)
			{
				// y+ is up
				if (abs(y) > threshold)
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR |= (1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
				else
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
			}
			else
			{
				// y- is up
				if (abs(y) > threshold)
				{
					GPIOC->ODR |= (1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
				else
				{
					GPIOC->ODR &= ~(1<<6);
					GPIOC->ODR &= ~(1<<7);
					GPIOC->ODR &= ~(1<<8);
					GPIOC->ODR &= ~(1<<9);
				}
			}
		}
		HAL_Delay(100);
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
