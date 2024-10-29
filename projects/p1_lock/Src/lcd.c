#include "main.h"
#include "lcd.h"


void LCD_init() {

	GPIOB->MODER &= ~(0x3FFFFF);	// clear PB 0-10 MODER pins
	GPIOB->MODER |=  (0x155555);	// set PB 0-10 MODER as write

	LCD_write(0x30, 0);		// wake up
	HAL_Delay(100);
	LCD_write(0x30, 0);		// wake up
	HAL_Delay(10);
	LCD_write(0x30, 0);		// wake up
	HAL_Delay(10);

	LCD_write(0x38, 0);		// 8-bit, Dual line
	LCD_write(0x10, 0);		// move display, shift left
	LCD_write(0x0C, 0);		// display on, cursor off
	LCD_write(0x01, 0);		// clears the screen
	HAL_Delay(1);			// delay
}

/* Writes value to the LCD as a instruction (mode = 0) or data (mode = 1)*/
void LCD_write(uint8_t value, uint16_t mode) {
	GPIOB->ODR &= ~(RS | RW);		// clear the RS and RW bits

	GPIOB->ODR |= (mode << 10);		// set the mode (0 instruction, 1 data)

	GPIOB->ODR |= E_BIT;			// set the E bit

	GPIOB->ODR &= ~(0xFF);			// clear the B port
	GPIOB->ODR |= (0xFF & value);	// set the data


	GPIOB->ODR &= ~(E_BIT);			// set E bit low
	HAL_Delay(1);
}

// line = 0 (line 1), line = 1 (line 2)
void LCD_write_string(const char* string, uint8_t line) {
	LCD_write((line << 6) | (1 << 7), 0);


	for(uint8_t i = 0; string[i] != '\0'; i++) {
		LCD_write(string[i], 1);
	}
}

