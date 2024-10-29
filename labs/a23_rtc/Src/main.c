

#include "main.h"
#include "uart.h"
void SystemClock_Config(void);


int main(void) {
	HAL_Init();
	SystemClock_Config();
	UART_init();

	// GPIO INIT
	RCC->AHB2ENR &= ~(0x1 << 0);
	RCC->AHB2ENR |=  (0x1 << 0);	// enable GPIOA clock

	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |=  (0x1 << 10);	// PA5 output
	GPIOA->ODR &= ~(0x1 << 5);		// output LOW to LED

	// Disable HSE just in case
	RCC->CR &= ~(0x1 << 16);

	// Enable the LSE
	PWR->CR1 |= (0x1 << 8);			// Access RTC and Backup registers enabled
	RCC->BDCR |= (0x1 << 0);		// enable LSE clock
	while(!(RCC->BDCR & (1 << 1)));	// wait for LSE clock to turn on


	// RTC clock select and enable
	RCC->APB1ENR1 |= (0x1 << 10);	// enable general RTC clock
	RCC->BDCR |=  (0x1 << 8);		// set LSE as RTC clock
	RCC->BDCR |=  (0x1 << 15);		// enable RTC clock

	// Disables the write protect as per TRM
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// INITIALIZATION
	RTC->ISR |= (0x1 << 7);				// Set INIT flag
	while(!(RTC->ISR & (0x1 << 6)));	// wait until init is allowed

	/*
	 * Prescaler Configuration from TRM to give RTC 1Hz clock from the LSE
	 */
	RTC->PRER = 0;
	RTC->PRER |=  (0x7FFF << 0);		// RTC Synchronous Prescaler = 32,768 - 1

	RTC->TR = (0x0 << 20) | (0x8 << 16) | (0x3 << 12) | (0x5 << 8 | (0x0 << 4) | (0x0 << 0)); //set time to 08:35:00
	RTC->ISR &= ~(0x1 << 8);

	// ALARM A Register Initialization
	RTC->CR &= ~(0x1 << 8);			// disable ALARM A
	while (!(RTC->ISR & 0x1));		//wait for allowed to write to Alarm A

	RTC->ALRMAR = (0x0 << 20) | (0x8 << 16) | (0x3 << 12) | (0x5 << 8) | (0x1 << 4 | (0x0 << 0)); //set alarm -> 08:35:10
	RTC->ALRMAR |= (0x1 << 31);		// don't care about Date/day
	RTC->ISR &= ~(0x1 << 7);		// clear INIT flag
	RTC->CR |=  (0x1 << 8);			// enable ALARM


	while(!(RTC->ISR & (0x1 << 8)));	// wait for ALARM A to trigger


	while(1) {

		GPIOA->ODR |= (0x1 << 5);		// output HIGH to LED
		HAL_Delay(500);
		GPIOA->ODR &= ~(0x1 << 5);		// output HIGH to LED
		HAL_Delay(500);



	}
}// end main



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  //RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	/* from stm32l4xx_hal_rcc.h==
	#define RCC_MSIRANGE_0                 MSI = 100 KHz
	#define RCC_MSIRANGE_1                 MSI = 200 KHz
	#define RCC_MSIRANGE_2                 MSI = 400 KHz
	#define RCC_MSIRANGE_3                 MSI = 800 KHz
	#define RCC_MSIRANGE_4                 MSI = 1 MHz
	#define RCC_MSIRANGE_5                 MSI = 2 MHz
	#define RCC_MSIRANGE_6                 MSI = 4 MHz
	#define RCC_MSIRANGE_7                 MSI = 8 MHz
	#define RCC_MSIRANGE_8                 MSI = 16 MHz
	#define RCC_MSIRANGE_9                 MSI = 24 MHz
	#define RCC_MSIRANGE_10                MSI = 32 MHz
	#define RCC_MSIRANGE_11                MSI = 48 MHz   dont use this one*/
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
  //RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
