#define E_BIT	0x100
#define RW		0x200
#define RS		0x400

void LCD_init();
void LCD_write(uint8_t value, uint16_t mode);
void LCD_write_string(const char* string, uint8_t line);

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#endif /* SRC_LCD_H_ */
