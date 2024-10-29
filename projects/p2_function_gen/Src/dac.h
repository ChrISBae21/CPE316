/*
 * dac.h
 *
 *  Created on: Feb 3, 2024
 *      Author: cbgno
 */



void DAC_init();
void DAC_GPIO_init();
void DAC_SPI_init();
void DAC_write(uint16_t data);
uint16_t DAC_volt_conv(uint16_t vout);

#define VREF_MV 3300 // Reference Voltage in mV

#ifndef SRC_DAC_H_
#define SRC_DAC_H_



#endif /* SRC_DAC_H_ */
