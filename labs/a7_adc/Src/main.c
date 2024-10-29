

#include "main.h"
#include <math.h>
#include "uart.h"
uint8_t num_conv = 0;
uint8_t flag = 0;

void SystemClock_Config(void);
void ADC_init();
void float_to_string(float num, char* buffer, uint8_t buff_len);
void ADC1_2_IRQHandler(void);
void ADC_init();


//TRY WITH 4MHZ SYSTEM CLOCK
int main(void) {
	HAL_Init();
	SystemClock_Config();
	RCC->AHB2ENR  |= ( (1 << 0) );		// enable GPIOA clock
	RCC->APB1ENR1 |= (1 << 17);			// enable USART2 clock
	RCC->AHB2ENR  |= ( (1 << 13) );		// enable ADC clock

	// PLL CLOCK INITIALIZATION
	RCC->PLLCFGR &= ~(0x3 << 0);		// clear PLL entry clock source
	RCC->PLLCFGR |=  (0x1 << 0);		// set MSI clock as PLL (all PLLs) clock source

	RCC->CR &= ~(0x1 << 26);
	while ( (RCC->CR & (0x1 << 26)) );	// wait until the PLLSAI1 turns off

	RCC->PLLSAI1CFGR &= ~(0x7F << 8);		// clear PLLSA1 Configuration

	// VCO output frequency must be between 64 and 344 MHz
	RCC->PLLSAI1CFGR |= (0x18 << 8);    // multiply VCO, PLL input clock, by 24 (4 MHz * 24 = 96 MHz)
	RCC->PLLSAI1CFGR &= ~(0x3 << 25);
	RCC->PLLSAI1CFGR |= (0x1 << 25);	// divide VCO frequency by 4 (96 MHz / 4 = 24 MHz)


	RCC->PLLSAI1CFGR |= (0x1 << 24);	// enable clock to the ADC
	RCC->CR |= (0x1 << 26);				// PLLSAI1 Turned On

	while ( (RCC->CR & (0x1 << 26)) == 0);				// wait until the PLLSAI1 turns on
	RCC->CCIPR &= ~(0x3 << 28);
	RCC->CCIPR |=  (0x1 << 28);			//ADC Peripheral Clock set to PLL



	UART_init();
	ADC_init();
	//ADC1->IER |= (0x1 << 3);				// enable end of regular sequence conversion interrupt



	// PAGE 511 CONSTRAINT TO THE CLOCK?????????????????????????
	uint32_t data = 0;

	double low = 0;
	double high = 0;
	double avg = 0;

	char lbuff[20];
	char hbuff[20];
	char abuff[20];

	ADC1->CR |= (0x1 << 2);					// ADSTART = 1
	flag = 1;
	while(1) {
		avg += ADC1->DR;

		num_conv+=1;

		if(num_conv == 30) {
			avg = roundf( (avg / 30) * 100 ) / 100;

			float_to_string(avg, hbuff, 20);
			UART_print(hbuff);
			UART_print("\n\r");
			num_conv = 0;
			avg = 0;

		}
		flag = 0;
		ADC1->CR |= (0x1 << 2);					// ADSTART = 1

	}

//	while (1) {
//		//UART_print("testi1\n\r");
//
//		while( (flag == 0) );
//		//UART_print("test");
//		data = ADC1->DR;						// automatically clears the EOC flag
//		//UART_print("testi2\n\r");
//
//		low = (data < low) ? data : low;
//		high = (data > high) ? data : high;
//		avg += data;
//
//		num_conv+=1;
//
//		if(num_conv == 19) {
//			avg = roundf( (avg / 19) * 100 ) / 100;
//
//			// CALIBRATION OF DIGITAL VALUES
//			avg = roundf( ((7.95*pow(10, -4))*avg + 0.0416) * 100 ) / 100;
//			high = roundf( ((7.95*pow(10, -4))*high + 0.0416) * 100 ) / 100;
//			low = roundf( ((7.95*pow(10, -4))*low + 0.0416) * 100 ) / 100;
//
//			float_to_string(avg, abuff, 20);
//			float_to_string(low, lbuff, 20);
//			float_to_string(high, hbuff, 20);
//
//			UART_print("(");
//			UART_print(lbuff);
//			UART_print(", ");
//			UART_print(hbuff);
//			UART_print(", ");
//			UART_print(abuff);
//			UART_print(")");
//			UART_print("\n\r");
//
//			num_conv = 0;
//			avg = low = high = 0;
//
//		}
//		flag = 0;
//		ADC1->CR |= (0x1 << 2);					// ADSTART = 1
//
//
//	}

}// end main
void ADC_init() {

	// GPIO INITIALIZATION
	GPIOA->MODER &= ~(0x3 << 0); 			// clear PA0
	GPIOA->MODER |=  (0x3 << 0);			// set PA0 to Analog Input
	GPIOA->AFR[0] &= ~(0xF);				// Clear PA0
	GPIOA->AFR[0] |= 0x7;					// set PA0 as ADC input pin
	GPIOA->ASCR |= (0x1 << 0);				// Analog Switch Control Register

	//ADC123_COMMON->CCR &= ~(0xF << 18);	// clear ADC prescaler
	//ADC123_COMMON->CCR |=  (0x1 << 0);	// set ADC prescaler divided by 2
	ADC123_COMMON->CCR &= ~(0x3 << 16);		// asynchronous mode; give ADC own clock from PLL


	ADC1->CR &= ~(0x1 << 29);				// get out of deep-power-down
	ADC1->CR |=  (0x1 << 28);				// turn on voltage regulator

	while( !(ADC1->CR & (0x1 << 28)) );		// wait for voltage regulator to turn on*/

	ADC1->CR &= ~(0x1 << 0);				// ADEN = 0; Disable ADC
	while( ADC1->CR & (0x1 << 0));			// wait for ADC to disable

	ADC1->CR |= (0x1 << 30);				// ADCALDIF = 0; single-input calibration
	ADC1->CR |=  (0x1 << 31);				// ADCAL = 1; start calibration
	while( ADC1->CR & (0x1 << 31) );		// wait for calibration to finish

	ADC1->DIFSEL = 0;						// Set all inputs to Single-Input

	ADC1->CFGR &= ~(0x1 << 13);				// CONT = 0; Single Conversion Mode
	ADC1->CFGR &= ~(0x3 << 3);				// 12-bit res

	ADC1->SMPR1 &= ~(0x1 << 31);			// no additional clocks
	ADC1->SMPR1 &= ~(0x7 << 0);				// clear sampling Rate for channel 0
	ADC1->SMPR1 |=  (0x7 << 0);				// set the sampling rate for channel 0

	ADC1->SQR1 &= ~(0xF << 0);				// sequence length of 1
	ADC1->SQR1 &= ~(0x1F << 6);				// clear first sequence select to set to channel 0
	ADC1->SQR1 |= (0x5 << 6);

	//ADC123_COMMON->CCR |= (0x1 << 22);	// VREF enable
	//ADC123_COMMON->CCR |= ~(0x1 << 18);	// VBAT enable

	ADC1->CFGR &= ~(0x3 << 10);				// EXTEN[1:0] = 0; disable hardware trigger detection

	ADC1->CR  |= (0x1 << 0);				// ADEN = 1
	while( !(ADC1->ISR & (0x1 << 0)) );		// wait for ADC ready flag
	ADC1->ISR |= (0x1 << 0);				// clear ADC ready flag

	ADC1->IER |= (0x1 << 2);
	ADC1->ISR |= (0x1 << 2);				// Clear EOC flag
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));		// enable USART2 ISR (bit 18)
	__enable_irq();




}


void ADC1_2_IRQHandler(void) {
	//UART_print("testingirq\n\r");

	//if( (ADC1->ISR & (0x1 << 3)) ) {
		flag = 1;
		ADC1->ISR |= (0x1 << 2);				// Clear EOC flag


	//}

}



void float_to_string(float num, char* buffer, uint8_t buff_len) {
	for(uint8_t i = 0; i < buff_len; i++) {
		buffer[i] = 0;
	}

    // Convert integer part
    int intPart = (int)num;
    int intDigits = 0;
    while (intPart != 0) {
        intPart /= 10;
        intDigits++;
    }

    if (intDigits == 0) {
        *buffer++ = '0';
    }
    intPart = (int)num;
    while (intDigits > 0) {
        int divisor = 1;
        for (int i = 1; i < intDigits; i++) {
            divisor *= 10;
        }
        *buffer++ = '0' + (intPart / divisor);
        intPart %= divisor;
        intDigits--;
    }

    // Convert fractional part, rounding to 2 decimal places
    *buffer++ = '.';
    float fractionalPart = num - (int)num;
    for (int i = 0; i < 2; i++) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        *buffer++ = '0' + digit;
        fractionalPart -= digit;
    }

    *buffer = '\0';
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
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
