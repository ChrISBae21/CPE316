#define ROWS 4
#define COLS 4

// define row and column pins

#define ROW_PORT GPIOC
#define COL_PORT GPIOC

// outputs
#define ROW_PIN0 0x3	//C0
#define ROW_PIN1 0xC	//C1
#define ROW_PIN2 0x30	//C2
#define ROW_PIN3 0xC0	//C3

// inputs
#define COL_PIN0 0x300	//C4
#define COL_PIN1 0xC00	//C5
#define COL_PIN2 0x3000	//C6
#define COL_PIN3 0xC000	//C7

// ROWS * 2 assuming the same pin-port for rows and cols (ex, port C), otherwise 0
#define COL_MODER_OFFSET 8

// assuming same pin-port for rows and cols, offset (PC4 - PC7)
#define COL_IDR_OFFSET 4


void init_keypad();

uint32_t run_keypad();



#ifndef SRC_NUMPAD_H_
#define SRC_NUMPAD_H_



#endif /* SRC_NUMPAD_H_ */
