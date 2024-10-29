/*
 * uart.h
 *
 *  Created on: Mar 1, 2024
 *      Author: cbgno
 */



void UART_init();
void UART_print(char* data);
void USART_ESC_Code(char* code);
void USART2_IRQHandler(void);
