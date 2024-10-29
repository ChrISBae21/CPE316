

#include "main.h"
#include "uart.h"
void SystemClock_Config(void);
void RNG_IRQHandler(void);
void int_to_string(uint32_t num, char* buffer, uint32_t buff_len);


uint8_t valid = 3; // 0 means valid
uint32_t RN;
char RN_STRING[32];

//TRY WITH 4MHZ SYSTEM CLOCK
int main(void) {
	HAL_Init();
	SystemClock_Config();

	RCC->AHB2ENR  |= ( (1 << 0) );		// enable GPIOA clock
	RCC->APB1ENR1 |= (1 << 17);			// enable USART2 clock
	RCC->AHB2ENR  |= ( (1 << 18) );		// enable RNG clock
	UART_init();

	// PLL CLOCK INITIALIZATION

	RCC->PLLCFGR |=  (0x1 << 0);		// set MSI clock as PLL (all PLLs) clock source

	RCC->CR &= ~(0x1 << 24);
	while ( (RCC->CR & (0x1 << 24)) );	// wait until the PLL turns off

	RCC->PLLCFGR |= (0x18 << 8);    // multiply VCO, PLL input clock, by 48 (4 MHz * 48 = 192 MHz)
	RCC->PLLCFGR &= ~(0x1 << 21);	// divide VCO frequency by 4 (192 MHz / 4 = 48 MHz)
	RCC->PLLCFGR |= (0x1 << 20);		// enable clock to the RNG
	RCC->CR |= (0x1 << 24);				// PLL Turned On

	while ( (RCC->CR & (0x1 << 24)) == 0);	// wait until the PLL turns on

	RCC->CCIPR &= ~(0x3 << 26);
	RCC->CCIPR |= (0x2 << 26);
	RNG->CR &= ~(0x1 << 2);			// Turn off RNG


	RNG->CR |= (0x1 << 5);

//	RNG->CR |= (0x1 << 3);			// RNG Interrupt Enable
//
//	NVIC->ISER[RNG_IRQn / 32] = (1 << (RNG_IRQn % 32)); //enable NVIC register
	RNG->CR |= (0x1 << 2);			// Turn on RNG
	HAL_Delay(1000);
	//__enable_irq();


	while( (RNG->SR & (0x1 << 6)) || (RNG->SR & (0x1 << 5)) || !(RNG->SR & (0x1 << 0)));
	RN = RNG->DR;
	RNG->CR &= ~(0x1 << 2);			// Turn off RNG
	int_to_string(RN, RN_STRING, 32);
	UART_print(RN_STRING);
	UART_print("\n\r");


	while(1) {
	}
}// end main

void int_to_string(uint32_t num, char* buffer, uint32_t buff_len) {
	for(uint32_t i = 0; i < buff_len; i++) {
		buffer[i] = 0;
	}

    // Convert integer part
	uint32_t temp = (uint32_t)num;
	uint32_t len = 0;
    while (temp != 0) {
        temp /= 10;
        len++;
    }

    if (len == 0) {
        *buffer++ = '0';
    }
    temp = (uint32_t)num;
    while (len > 0) {
    	uint32_t rem = 1;
        for (int i = 1; i < len; i++) {
            rem *= 10;
        }
        *buffer++ = '0' + (temp / rem);
        temp %= rem;
        len--;
    }

    *buffer = '\0';
}

void RNG_IRQHandler(void) {
	if(RNG->SR & (0x1 << 6)) {
		valid = 1;
		RNG->SR &= ~(0x1 << 6);
	}

	else if(RNG->SR & (0x1 << 5)) {
		valid = 2;
		RNG->SR &= ~(0x1 << 5);
	}

	else if(RNG->SR & (0x1 << 0)) {
		valid = 0;
		RN = RNG->DR;
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
