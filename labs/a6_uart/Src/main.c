/*
 *
 * GPIO Pins
 * PA2: USART2_TX
 * PA3: USART2_RX
 *
 *
 */

/*
 * Character Reception Procedure on pp. 1350 of Reference Manual
 * Character Transmission Procedure on pp. 1347 of Reference Manual
 */
//40.7 has list of uart interrupts

#include "main.h"

void SystemClock_Config(void);
void UART_init();
void UART_print(char* data);
void USART_ESC_Code(char* code);
void USART2_IRQHandler(void);
void float_to_string(float num, char* buffer, uint8_t buff_len);



int main(void) {
	HAL_Init();
	SystemClock_Config();


	// turns on clock to GPIO banks A and C
	RCC->AHB2ENR  |= (1 << 0);		// enables clock to GPIOA
	RCC->APB1ENR1 |= (1 << 17);		// enables clock to USART2

	UART_init();
	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR (bit 38)
	__enable_irq();

	float test = 145.3;
	char tbuff[20];
	float_to_string(test, tbuff, 20);

	USART_ESC_Code("[3B");
	USART_ESC_Code("[5C");
	UART_print("All good students read the");
	USART_ESC_Code("[1B");
	USART_ESC_Code("[21D");
	USART_ESC_Code("[5m");
	UART_print("Reference Manual");
	USART_ESC_Code("[H");
	USART_ESC_Code("[0m");
	UART_print("Input: ");
	UART_print(tbuff);
	while (1) {
	}

}// end main

void USART_ESC_Code(char* code) {
	while(!(USART2->ISR & (1 << 7)));		// wait until transmit data register is empty
	USART2->TDR = 0x1B;
	UART_print(code);
}

void UART_print(char* data) {
	uint8_t i;
	for(i = 0; data[i] != 0; i++) {
		while(!(USART2->ISR & (1 << 7)));		// wait until transmit data register is empty
		USART2->TDR = data[i];
	}
}





void USART2_IRQHandler(void) {
	// if the interrupt was from a reception of a byte
	if(USART2->ISR & (1 << 5)) {
		USART2->TDR = USART2->RDR;		// puts what was read by the RDR into the TDR
		USART2->RQR |= (1 << 3);		// clears RXNE bit using the RQR, RXFRQ bit (do I need this?)
	}
}

void UART_init() {

	// UART GPIO INIT
	GPIOA->MODER &= ~(0x3 << 4 | 0x3 << 6);			// clears GPIOA pins 2-3
	GPIOA->MODER |=  (0x2 << 4 | 0x2 << 6);			// sets GPIOA pins 2-3 as alternate function mode (10)

	GPIOA->AFR[0] &= ~(0xF << 8 | 0xF << 12); 		// clears AFR for GPIOA pins 2-3
	GPIOA->AFR[0] |=  (0x7 << 8 | 0x7 << 12); 		// sets AFR GPIOA pins 2-3 as AF7

	/*
	 *	Bit 11 is useless because bit 13 is disabled
	 * 	Bit 9 is useless because bit 10 is disabled
	 *
	 */

	USART2->CR1 &= ~((1 << 28)		// clear M1 for 1 start bit and 8-bit data 		(28)
					|(0x3 << 26)	// clear EOBIE and RTOIE to prevent interrupts	(26, 27)
					|(1 << 15)		// clear OVER8 bit to oversample by 16			(15)
					|(1 << 14)		// clear CMIE to prevent interrupt				(14)
					|(1 << 13)		// clear MME to disable Mute Mode				(13)
					|(1 << 12) 		// clear M0 for 1 start bit and 8-bit data		(12)
					|(1 << 10)		// clear PCE to disable parity					(10)
					|(1 << 8)		// clear PEIE to disable parity error interrupt	(8)
					|(1 << 7) 		// clear TXEIE to disable TXE interrupt			(7)
					|(1 << 6)		// clear TCIE to prevent interrupt 				(6)
					|(1 << 4)		// clear IDLEIE to disable idle interrupts		(4)
					|(1 << 3)		// clear TE: don't enable transmit yet			(3)
					|(1 << 2)		// clear RE: don't enable receive yet			(2)
					|(1 << 0)		// clear UE: don't enable USART yet				(0)
					);
	USART2->CR1 |=   (1 << 5);		// set RXNEIE to enable RXN interrupt		(5)

	/*
	 * Bit 18 is used by a different mode (smartcard)
	 *
	 */
	USART2->CR2 &= ~((1 << 23)		// clear RTOEN to disable reciever timeout		(23)
					|(1 << 20)		// clear ABREN to disable auto baud rate		(20)
					|(1 << 19)		// clear MSBFIRST to start LSB					(19)
					|(1 << 17)		// clear TXINV for TX to idle high				(17)
					|(1 << 16)		// clear RXINV for RX to idle high				(16)
					|(1 << 15)		// clear SWAP to use default pinout				(15)
					|(1 << 14)		// clear LINEN to disable LIN mode				(14)
					|(0x3 << 12)	// set STOP to have 1 stop bit					(12, 13)
					|(1 << 11)		// clear CLKEN for asynchronous mode			(11)
					|(1 << 6)		// clear LBDIE to disable LIN interrupt			(6)
					);

	USART2->CR3 &= ~((1 << 24)		// clear TCBGTIE to disable guard time inter.	(24)
					|(1 << 14)		// clear DEM to  disable driver enable 			(14)
					|(1 << 9) 		// disable CTS									(9)
					|(1 << 8)		// disable RTS									(8)
					|(1 << 7)		// disable DMA transmitter						(7)
					|(1 << 6)		// disable DMA reciever							(6)
					|(1 << 5)		// disable smartcard							(5)
					|(1 << 4)		// disable NACK									(4)
					|(1 << 3)		// disable half-duplex							(3)
					|(1 << 1)		// disable IrDA mode							(1)
					|(1 << 0)		// disable error interrupts						(0)
					);

	/*USART2->CR3 |= 	((1 << 12)		// disable OVERRUN								(12)
					|(1 << 11)		// ONEBIT to sample with one bit				(11)
					);*/

	USART2->BRR = 35;				// value for 115.2 kbps for a clock at 4 MHz


	USART2->CR1 |= ((1 << 3)		// enables transmitter	(3)
				   |(1 << 2)		// enables receiver		(2)
				   |(1 << 0)		// enables USART		(0)
				   );
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
