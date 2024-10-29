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
#include "dac.h"

/* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/



/**
  * @brief  The application entry point.
  * @retval int
  */

void DAC_init() {
	// PA 4, 5, 7
	RCC->APB2ENR |= (1 << 12); //RCC_APB2ENR_SPI1EN
	DAC_GPIO_init();
	DAC_SPI_init();
}

void DAC_GPIO_init() {
	GPIOA->MODER &= ~(0x3 << 14 | 0x3 << 10 | 0x3 << 8);	// clears pins A 4, 5, 7
	GPIOA->MODER |= (0x2 << 14 | 0x2 << 10 | 0x2 << 8);   // sets pins A 4, 5, 7 as Alternate Function (10)
	GPIOA->OSPEEDR &= ~(3<<10)|(3<<14)|(3<<8);

	GPIOA->AFR[0] &= ~(0xF << 28 | 0xF << 20 | 0xF << 16);	// clear alternate function registers 4, 5, 7
	GPIOA->AFR[0] |= (0x5 << 28 | 0x5 << 20 | 0x5 << 16);		// sets 4, 5, 7 to AF5 for SPI
}


void DAC_SPI_init() {
	//SPI1->CR1 &= ~(0xFFFF);
	SPI1->CR1 &= ~(1 << 6);

	SPI1->CR1 &= ~(1 << 0);		// Clock Phase to be on the first edge
	SPI1->CR1 &= ~(1 << 1);		// Clock Polarity to be idle low
	SPI1->CR1 |= (1 << 2);		// Master Configuration
	SPI1->CR1 &= ~(0x7 << 3);	// fclk / 2
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


	data |= (1 << 12);					// Active Mode Operation
	data |= (1 << 13);					// Gain of 0
	data &= ~(1 << 14);					// unbuffered (0 << 14)
	data &= ~(1 << 15);					// Enable writes to DAC Reg (0 << 15)
	SPI1->DR = data;

	while(~(SPI1->SR) & (1 << 1)) {};		// wait for TXE bit (buffer becomes empty)
	while(SPI1->SR & (1 << 7)) {};			// wait for BSY bit to reset (communication not busy)

}

uint16_t DAC_volt_conv(uint16_t vout) {
	return ((vout*4096)/VREF_MV);
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
