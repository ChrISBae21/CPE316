/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * Documents Used:
  * STM32 Datasheet, Table 17 for SPI pins
  *
  * STM32 Reference Manual for GPIO and Alternate Function configuration
  * 	PG 1476 for SPI registers
  *
  * MCP4921 Datasheet for DAC pinouts and Voltage Equation
  * 	DAC input = (Vout * 4096) / Vref
  *
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void DAC_init();
void GPIO_init();
void SPI_init();
void DAC_write(uint16_t data);
uint16_t DAC_volt_conv(uint16_t vout);

#define VREF_MV 3300 // Reference Voltage in mV


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init();
	SystemClock_Config();

	// turns on clock to GPIO A and SPI1
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	RCC->APB2ENR |= (1 << 12); //RCC_APB2ENR_SPI1EN

	DAC_init();

	while (1) {
		DAC_write(1);
		HAL_Delay(3000);
		DAC_write(2);
		HAL_Delay(1000);
	}
}

void DAC_init() {
	GPIO_init();
	SPI_init();
}

void GPIO_init() {
	GPIOA->MODER &= ~(0x3 << 14 | 0x3 << 10 | 0x3 << 8);	// clears pins A 4, 5, 7
	GPIOA->MODER |= (0x2 << 14 | 0x2 << 10 | 0x2 << 8);   // sets pins A 4, 5, 7 as Alternate Function (10)
	GPIOA->OSPEEDR &= ~(3<<10)|(3<<14)|(3<<8);

	GPIOA->AFR[0] &= ~(0xF << 28 | 0xF << 20 | 0xF << 16);	// clear alternate function registers 4, 5, 7
	GPIOA->AFR[0] |= (0x5 << 28 | 0x5 << 20 | 0x5 << 16);		// sets 4, 5, 7 to AF5 for SPI
}


void SPI_init() {
	//SPI1->CR1 &= ~(0xFFFF);
	SPI1->CR1 &= ~(1 << 6);

	SPI1->CR1 &= ~(1 << 0);		// Clock Phase to be on the first edge
	SPI1->CR1 &= ~(1 << 1);		// Clock Polarity to be idle low
	SPI1->CR1 |= (1 << 2);		// Master Configuration
	SPI1->CR1 |= (0x7 << 3);	// fclk / 2
	SPI1->CR1 &= ~(1 << 7);		// MSB First

	SPI1->CR1 &= ~(1 << 9);		// SSM: NCS managed by hardware
	SPI1->CR1 &= ~(1 << 10);	// Duplex Mode, also for transmit only
	SPI1->CR1 |= (1 << 11);		// CRC length 16-bits
	SPI1->CR1 &= ~(1 << 13);	// CRC Calculation disabled
	SPI1->CR1 |= (1 << 14);		// Transmit Mode
	SPI1->CR1 &= ~(1 << 15);	// BIDIMODE low: 2 line undirectional data

	//this logic is for the NSS
	SPI1->CR2 |= (1 << 2);		// Enable SSOE	utput in master mode
	SPI1->CR2 |= (1 << 3);		// generate NCS pulse between data transfers to peripheral
	SPI1->CR2 &= ~(1 << 4);		// set to Motorola mode
	SPI1->CR2 |= (0xF << 8);	// 16-bit data transfers

	SPI1->CR1 |= (1 << 6);		// Enable SPI
}


void DAC_write(uint16_t data) {
	while(!((SPI1->SR) & (1<<1))) {};		// wait for TXE bit (buffer becomes empty)

	uint16_t dac_data = (uint16_t) DAC_volt_conv(data);

	dac_data |= (1 << 12);					// Active Mode Operation
	dac_data |= (1 << 13);					// Gain of 0
	dac_data &= ~(1 << 14);					// unbuffered (0 << 14)
	dac_data &= ~(1 << 15);					// Enable writes to DAC Reg (0 << 15)
	SPI1->DR = dac_data;

	while(~(SPI1->SR) & (1 << 1)) {};		// wait for TXE bit (buffer becomes empty)
	while(SPI1->SR & (1 << 7)) {};			// wait for BSY bit to reset (communication not busy)

}

uint16_t DAC_volt_conv(uint16_t vout) {
	return ((vout*4096*1000)/VREF_MV);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

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
