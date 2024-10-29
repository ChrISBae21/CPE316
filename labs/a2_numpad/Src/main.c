#include "main.h"
#include "uart.h"

// ROW x COL numpad dimensions
#define ROWS 4
#define COLS 4

//Hard Keypad is Col then Row
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

void SystemClock_Config(void);
void mydelay(int count);
void init_keypad();

char run_keypad();

// mapping for numpad
char mapping[4][4] = {
	    {1,2,3,10},
	    {4,5,6,11},
	    {7,8,9,12},
	    {14,0,15,13}
	};

int main(void){
	HAL_Init();
	SystemClock_Config();

	char input, temp_input;
	// turns on clock to GPIO banks A and C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);

	// LED output registers
	GPIOC->MODER &= ~(0xFF0000);	// clears PC 8-11
	GPIOC->MODER |=  (0x550000);	// sets PC 8-11 as output

	// create mapping for numpad
	init_keypad();
	UART_init();
	__enable_irq();
	UART_print("test");
	while (1){
		/*output = run_keypad();

		if(output != 0xFF) {
			GPIOC->ODR &= ~(0xF<<8);	// clear the LED state
			GPIOC->ODR |= output<<8;	// output to the LED
		}*/
//		input = temp_input = 0xFF;

//		input = temp_input = run_keypad();
		while((input = temp_input = run_keypad()) == 0xFF);
		while(temp_input != 0xFF) temp_input = run_keypad();
		USART2->TDR = input + '0';

	}

}// end main


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

char run_keypad() {

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
