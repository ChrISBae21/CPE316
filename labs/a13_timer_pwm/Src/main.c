

#include "main.h"


void PWM_GPIO_init();
void PWM_init();
void SystemClock_Config(void);

int main(void) {
	HAL_Init();
	SystemClock_Config();

	PWM_init();

	/*
	 * Page 1041 shows Asymmetric Mode Example
	 * Page 1072 is TIMx_CCMR1 OC1M Register
	 * Look for Table 17 for Alternate Function pins for TIMx_CH1, TIMx_CH2, TIMx_CH3, TIMx_CH4
	 */
	while (1) {

	}

}// end main



void PWM_GPIO_init() {
	// Configure GPIO and AFR
	
	GPIOA->MODER &= ~((0x3 << 0) | (0x3 << 2));		// clear PA0 and PA1
	GPIOA->MODER |=  ((0x2 << 0) | (0x2 << 2));		// set PA0 and PA1 to AF

	GPIOA->AFR[0] &= ~((0xF << 0) | (0xF << 4));	// clear AF
	GPIOA->AFR[0] |=  ((0x1 << 0) | (0x1 << 4));	// set PA0 and PA1 to AF1
}

void PWM_init() {
	RCC->AHB2ENR  |= (1 << 0);		// enables clock to GPIOA
	RCC->APB1ENR1 |= (1 << 0);		// turn on clock to TIM2RCC->AHB2ENR  |= (1 << 0);		// enables clock to GPIOA
	RCC->APB1ENR1 |= (1 << 0);		// turn on clock to TIM2
	PWM_GPIO_init();
	TIM2->CCER	&= ~(0x1 << 1);					// CC1P (Polarity) Active High
	TIM2->CCER 	&= ~(0x1 << 5);					// CC2P (Polarity) Active High

	TIM2->CR1 	|= (0x1 << 7);					// set the Auto-Reload Preload

	// CCMR CONFIGURATION FOR CHANNEL 1
	TIM2->CCMR1 |=  (0x1 << 3);					// set the Preload Register for CCR1
	TIM2->CCMR1 &= ~((0x1 << 16)				// clear bit 16 for OC1M Channel 1 Mode 1
					|(0x1 << 6)					// clear bit 6 for  OC1M Channel 1 Mode 1
					|(0x1 << 5)					// clear bit 5 for  OC1M Channel 1 Mode 1
					|(0x1 << 4)					// clear bit 4 for  OC1M Channel 1 Mode 1
	);
	TIM2->CCMR1 |=  ((1 << 6) | (1 << 5));		// set bits 5 and 6 for OC1M Channel 1 Mode 1
	TIM2->CCMR1 &= ~(0x3 << 0);					// clear to set Channel 1 as output

	// CCMR CONFIGURATION OR CHANNEL 2
	TIM2->CCMR1 |= (0x1 << 11);					// Set the Preload Register for CCR2
	TIM2->CCMR1 &= ~((0x1 << 24)				// clear bit 24 for OC2M Channel 2 Mode 2
					|(0x1 << 14)				// clear bit 14 for OC2M Channel 2 Mode 2
					|(0x1 << 13)				// clear bit 13 for OC2M Channel 2 Mode 2
					|(0x1 << 12)				// clear bit 12 for OC2M Channel 2 Mode 2
	);

	TIM2->CCMR1 |=  ((1 << 14) 					// set bit 14 for OC2M Channel 2 Mode 2
					|(1 << 13) 					// set bit 13 for OC2M Channel 2 Mode 2
					|(1 << 12)					// set bit 12 for OC2M Channel 2 Mode 2
	);
	TIM2->CCMR1 &= ~(0x3 << 8);					// clear to set Channel 2 as output


	TIM2->ARR  = 80000-1;
	TIM2->CCR1 = 20000-1;
	TIM2->CCR2 = 60000-1;

	TIM2->EGR	|= (0x1 << 0);				// set the UG bit
	TIM2->CCER	|= ((0x1 << 0)				// enable Capture Mode for Channel 1
				   |(0x1 << 4)				// enable Capture Mode for Channel 2
	);
	TIM2->CR1	|= (0x1 << 0);				// enable counter

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
