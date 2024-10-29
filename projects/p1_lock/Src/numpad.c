#include "main.h"
#include "numpad.h"
// ROW x COL numpad dimensions

// mapping for numpad
uint8_t mapping[4][4] = {
	    {1,2,3,10},
	    {4,5,6,11},
	    {7,8,9,12},
	    {14,0,15,13}
	};


// initializes the keypad inputs and outputs.
void init_keypad() {

	// clears and sets rows as output (MODER 0b01)
	ROW_PORT->MODER &= ~(ROW_PIN0 | ROW_PIN1 | ROW_PIN2 | ROW_PIN3);
	ROW_PORT->MODER |= ( (ROW_PIN0 | ROW_PIN1 | ROW_PIN2 | ROW_PIN3) & (0x55) );

	// clears and sets cols as input (MODER 0b00)
	COL_PORT->MODER &= ~(COL_PIN0 | COL_PIN1 | COL_PIN2 | COL_PIN3);
	COL_PORT->MODER |= ((COL_PIN0 | COL_PIN1 | COL_PIN2 | COL_PIN3) & (0x00 << COL_MODER_OFFSET) );

	// defaults to pull-down (PUPDR 0b10)
	COL_PORT->PUPDR &= ~(COL_PIN0 | COL_PIN1 | COL_PIN2 | COL_PIN3);
	COL_PORT->PUPDR |= ((COL_PIN0 | COL_PIN1 | COL_PIN2 | COL_PIN3) & (0xAA << COL_MODER_OFFSET));


}

uint8_t run_keypad() {

	for(int r = 0; r < ROWS; r++) {

		// set the current row high
		ROW_PORT->ODR &= ~(0xF);
		ROW_PORT->ODR |= 1 << r;

		for(int c = 0; c < COLS; c++) {
			// read the current state of the col
			if(COL_PORT->IDR & (1 << (c + COL_IDR_OFFSET))) {
				HAL_Delay(10);
				// check again to ensure button was pressed
				if(COL_PORT->IDR & (1 << (c + COL_IDR_OFFSET))) {
					return mapping[r][c];
				}
			}
		}
		// set the current row low
		ROW_PORT->ODR &= ~(0xF);
	}
	// no key was pressed, return error
	return 0xFF;

}



