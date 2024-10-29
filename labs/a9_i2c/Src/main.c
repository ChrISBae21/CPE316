
/*
 * PB6: I2C1_SCL
 * PB7: I2C1_SDA
 *
 * PA0: EEPROM Address bit selects
 * PA5: LED Output
 * Alternate Function 4
 */



#include "main.h"
uint8_t* GLOBAL_TXDR;
uint8_t* GLOBAL_RXDR;

void SystemClock_Config(void);
void I2C_init();
void I2C_read(uint8_t addr, uint8_t num_bytes, uint8_t* data);
void I2C_write(uint8_t _addr, uint8_t num_bytes, uint8_t* data);
void I2C_slave_addr(uint8_t addr);
void I2C_interrupt_init(void);
void I2C1_EV_IRQHandler(void);

void EEPROM_read(uint8_t addr, uint8_t num_bytes_out, uint8_t* datao, uint8_t num_bytes_in, uint8_t* datai);


int main(void) {
	HAL_Init();
	SystemClock_Config();

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~( (0x3 << 0) | (0x3 << 10) );	// Clear PA0 and PA5
	GPIOA->MODER |=  ( (0x1 << 0) | (0x1 << 10) );	// Set PA0 and PA5 to Output
	GPIOA->ODR |= ( (0x1 << 0));	// Output High
	GPIOA->ODR &= ~(0x1 << 5);		// Output Low to the LED


	uint8_t addr = 0x57;			// EEPROM's I2C Address
	uint8_t datao[32];				// Data to be output to the EEPROM
	uint8_t datai[32];				// Data to be received by the EEPROM

	datao[0] = 0x0B;				// Upper Memory Address of EPROM
	datao[1] = 0xB4;				// Lower Memory Address of EPROM
	datao[2] = 255;					// Data to write to EPROM

	GLOBAL_TXDR = datao;
	GLOBAL_RXDR = datai;


	I2C_init();
	//I2C_interrupt_init();

	while (1) {

		I2C_write(addr, 3, datao);			// write 3 bytes of data	

		HAL_Delay(3000);

		EEPROM_read(addr, 2, datao, 1, datai);	// EEPROM write address, then read data
		while(1) {
			if(datai[0] == datao[2])		// Check if the read data is the same as the written data
				GPIOA->ODR |= (0x1 << 5);	// Output High
			else
				GPIOA->ODR &= ~(0x1 << 5);	// Output Low
		}
	}
}



/*
 * Initialized with AUTOEND off
 */
void I2C_init() {

	/* I2C Clock Init */
	RCC->AHB2ENR &= ~(0x1 << 1);
	RCC->AHB2ENR |=  (0x1 << 1);		// Enable GPIOB Clock
	RCC->APB1ENR1 |= (0x1 << 21);		// Enable I2C1 Clock on APB1 (PCLK)
	RCC->CCIPR &= ~(0x3 << 12);			// Set the I2C1 Clock to PCLK from APB1

	/* I2C GPIO Init */
	GPIOB->MODER &= ~( (0x3 << 12) | (0x3 << 14) );		// clear PB6 and PB7 Pins
	GPIOB->MODER |=  ( (0x2 << 12) | (0x2 << 14) );		// set PB6 and PB7 as Alternate Function

	GPIOB->AFR[0] &= ~( (0xF << 24) | (0xF << 28) );	// clear AFR for PB6 and PB7
	GPIOB->AFR[0] |=  ( (0x4 << 24) | (0x4 << 28) );	// set AFR4 for PB6 and PB7
	GPIOB->OTYPER |= (1<<6) | (1<<7);					// set PB6 and PB7 to open-drain
	GPIOB->OSPEEDR |= (3<<12) | (3<<14);				// set the speed
	GPIOB->PUPDR |= (1<<12) | (1<<14);					// PB6 and PB7 pull-up

	/* I2C Register Init */

	I2C1->CR1 &= ~(0x1 << 0);		// disable the I2C1 before setting registers

	// Before enabling I2C1, Filters must be set
	I2C1->CR1 &= ~(0x1 << 12);		// disable Analog Filter
	I2C1->CR1 &= ~(0xF << 8);		// disable Digital Noise Filter

	I2C1->CR2 &= ~(0x1 << 25); 		// AUTOEND OFF

	I2C1->TIMINGR &= ~(0xFFFFFFFF);	// clear timing register
	I2C1->TIMINGR  |=  0x00703F55;	// timing for 100kHz I2C clock from 16 MHz system with 200ns rise time and 200ns fall time

	I2C1->CR1 &= ~(0x1 << 17);		// NOSTRETCH must be cleared in Master Mode
	I2C1->CR1 |=  (0x1 << 0);		// enable the I2C1

}

void I2C_interrupt_init(void) {
	I2C1->CR1 |= ((0x1 << 6) 		// TC Interrupt Enable
				 |(0x1 << 1)		// TXIE Interrupt Enable
				 |(0x1 << 2)		// RXIE Interrupt Enable
	);

	NVIC->ISER[0] |= (0x1 << 31);	// I2C1_EV Interrupt Enable

	__enable_irq();
}

void I2C_read(uint8_t addr, uint8_t num_bytes, uint8_t* data) {
	I2C1->CR2 |= (0x1 << 10);		// Master Read Request

	I2C1->CR2 &= ~(0xFF << 16);			// Clear the number of bytes
	I2C1->CR2 |=  (num_bytes << 16);	// set the number of bytes to be transfered
	I2C_slave_addr(addr);			// set the peripheral address
	I2C1->CR2 |= (0x1 << 13);			// START the data transfer


	for(int i = 0; i < num_bytes; i++) {
		while (!(I2C1->ISR & (1 << 2)));
		data[i] = I2C1->RXDR;
	}

	while(!(I2C1->ISR & (1 << 6)));		// TC bit

	I2C1->CR2 |= (0x1 << 14);	// Generate STOP


}


void I2C_write(uint8_t addr, uint8_t num_bytes, uint8_t* data) {
	//I2C_slave_addr_init(0x57);
	I2C1->CR2 &= ~(0x1 << 10);			// Master Write Request

	I2C1->CR2 &= ~(0xFF << 16);			// Clear the number of bytes
	I2C1->CR2 |=  (num_bytes << 16);	// set the number of bytes to be transfered
	I2C_slave_addr(addr);				// set the peripheral address

	I2C1->CR2 |= (0x1 << 13);			// START the data transfer



	for(int i = 0; i < num_bytes; i++) {
		while(!(I2C1->ISR & (1 << 1)));
		I2C1->TXDR = data[i];
	}

	while(!(I2C1->ISR & (1 << 6)));		// TC bit
	//GPIOA->ODR |= (0x1 << 5);
	I2C1->CR2 |= (0x1 << 14);	// Generate STOP

}

void I2C_slave_addr(uint8_t addr) {
	I2C1->CR2 &= ~(0x1 << 11);			// 7 bit data
	I2C1->CR2 &= ~(0x3FF << 0);			// clear SADD (slave addr)
	I2C1->CR2 |= (addr << 1);			// shift slave addr by 1 for 7-bit addr
}


void I2C1_EV_IRQHandler(void) {

	/*
	 * TXIS Bit (Successful Byte Transfer)
	 */
	if(I2C1->ISR & (1 << 1)) {
		I2C1->TXDR = *(++GLOBAL_TXDR);	// Grab next index and dereference the data
		// TXIS bit is cleared when next bit is written	so we don't have to do it
	}

	/*
	 * RXNE Bit (Successful Byte Transfer)
	 */
	else if(I2C1->ISR & (1 << 2)) {
		*(++GLOBAL_RXDR) = I2C1->RXDR;	// Grab next index and dereference to recieve data
		// RXNE bit is cleared when RXDR is read, don't have to clear

	}
	/*
	 * TC Bit (Successful NBYTE transfers)
	 */
	else if(I2C1->ISR & (1 << 6)) {

		GPIOA->ODR &= ~(0x1 << 5);
		I2C1->CR2 |= (0x1 << 14);	// Generate STOP
		// TC bit is cleared when START of STOP is set
	}
}

void EEPROM_read(uint8_t addr, uint8_t num_bytes_out, uint8_t* datao, uint8_t num_bytes_in, uint8_t* datai) {
	I2C_write(addr, num_bytes_out, datao);

	I2C_read(addr, num_bytes_in, datai);

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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
/*static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

} */

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
