

#include "main.h"
#include "uart.h"
void SystemClock_Config(void);


//TRY WITH 4MHZ SYSTEM CLOCK
int main(void) {
	HAL_Init();
	SystemClock_Config();

	RCC->AHB2ENR &= ~(0x1 << 0);
	RCC->AHB2ENR |=  (0x1 << 0);	// enable GPIOA clock

	UART_init();

	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |=  (0x1 << 10);	// PA5 output

	GPIOA->ODR |= (0x1 << 5);		// output HIGH to LED

	__enable_irq();
	UART_print("System has reset\n\r");

	RCC->CSR |= (0x1 << 0);			// enable LSI clock
	while(!(RCC->CSR & (1 << 1)));	// wait for LSI clock to turn on




	IWDG->KR = 0x0000CCCC;				// enable IWDG
	IWDG->KR = 0x00005555;				// enable write access

	while((IWDG->SR & (0x1 << 0)));		// wait for prescaler to set
	IWDG->PR &= ~(0x7);					// clear prescaler
	while((IWDG->SR & (0x1 << 0)));		// wait for prescaler to set
	IWDG->PR |=  (0x4);					// prescaler of 64: 32kHz / 64

	while((IWDG->SR & (0x1 << 1)));		// wait for reload value to set
	IWDG->RLR &= ~(0xFFF);				// clear reload register
	while((IWDG->SR & (0x1 << 1)));		// wait for reload value to set
	IWDG->RLR |= 0x3E7;					// 3 seconds:
										// Reload value = (Time(ms) * 32000) / (4 * 2^( (prescaler / 4) - 1) * 1000)



	while((IWDG->SR != 0));			// wait for all registers to set

	IWDG->KR = 0x0000AAAA;			// refresh counter


	GPIOA->ODR &= ~(0x1 << 5);		// output LOW to LED



	while(1) {
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
