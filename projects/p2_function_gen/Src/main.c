/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  *
  * EX from Lab Man: In 100Hz, we have 20 total points for one period (10ms), 0-3V
  * 				 this makes 2 Points Per Millisecond
  * 				 Number of Points in one 100Hz Period: 2 * 10ms = 20 points
  *
  * 				 Using the same number of points, for 200Hz (5ms)
  * 				 Number of Points in one 200Hz Period: 2 *  5ms = 10 points
  *
  *
  * 				 So, if we have a array of 240 points:
  * 				 24 Points Per Millisecond
  *
  * 				 100Hz: 24 * 10ms = 240 Points
  * 				 200Hz: 24 * 5ms = 120 Points
  * 				 300Hz: 24 * 3.3ms = 80 Points
  * 				 400Hz: 24 * 2.5ms = 60 Points
  * 				 500Hz: 25 * 2ms = 48 Points     --> Means we should take every 240/48 = 5 points
  *
  *
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "main.h"
#include "dac.h"
#include "numpad.h"

#define AMPL_MV 1500
#define VBIAS_MV 1500
#define SAMPLES 240
#define PI 3.1415
#define CLOCK 24000000


static uint16_t sine_array[240];
static uint16_t ramp_array[240];
static uint16_t triangle_array[240];
static uint16_t square_array[2];

uint32_t wave_type;
uint32_t freq;
uint32_t duty_cycle;
uint32_t wave_index;


void SystemClock_Config(void);
void INTERRUPT_init();
void TIM2_IRQHandler(void);
void setCCR1(uint32_t data);
void setARR(uint32_t data);
void LUT_init();
void setWave();
void outputWave();



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	HAL_Init();
	SystemClock_Config();



	uint32_t input;
	uint32_t temp_input;
	// default values for startup
	wave_type = 9;		// SQUARE WAVE
	freq = 1;			// 100 Hz
	duty_cycle = 5;		// 50% DUTY CYCLE
	wave_index = 0;		// INDEX 0

	// turns on clock to GPIO A
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	init_keypad();		// initialize the keypad
	DAC_init(); 		// initialize SPI communication with the DAC
	LUT_init();			// initialize values for LUTs
	INTERRUPT_init();	// initialize Timer and Interrupts

	while (1) {
			input = 0xFF;
			while(input == 0xFF)
				input = temp_input = run_keypad();
			while(temp_input != 0xFF) temp_input = run_keypad();

			switch(input) {
				case 1:		// 100 Hz
				case 2:		// 200 Hz
				case 3:		// 300 Hz
				case 4:		// 400 Hz
				case 5:		// 500 Hz
					freq = input;			// change frequency
					wave_type = wave_type;
					setWave();
					break;
				case 6:		// SINE
				case 7:		// TRIANGLE
				case 8:		// RAMP
				case 9:		// SQUARE
					wave_type = input;		// change wave type
					setWave();
					break;

				case 0:		// 50% DUTY CYCLE
					freq = freq;
					wave_type = wave_type;
					duty_cycle = 5;
					setWave();
					break;

				case 14:	// DECREASE DUTY CYCLE 10%
					freq = freq;
					wave_type = wave_type;

					// decrease duty cycle if possible
					if(duty_cycle - 1 != 0)
						duty_cycle -= 1;
					else
						duty_cycle = duty_cycle;
					setWave();

					break;

				case 15:	// INCREASE DUTY CYCLE 10%

					freq = freq;
					wave_type = wave_type;
					// increase duty cycle if possible
					if(duty_cycle + 1 != 10)
						duty_cycle += 1;
					else
						duty_cycle = duty_cycle;
					setWave();
					break;

				default:
					break;
		}
	}
}


/*
 * Interrupt Service Routine for ARR and CCR1
 */
void TIM2_IRQHandler(void) {

	// ARR
	if(TIM2->SR & 1) {
		outputWave();
		TIM2->SR &= ~(0x1);			 // clear status register of U.I. because we are servicing it
	}

	// CCR1
	else if(TIM2->SR & 0x2 && wave_type == 9) {
		DAC_write(square_array[0]);
		TIM2->SR &= ~(TIM_SR_CC1IF); // clear status register of CCR1 because we are servicing it
	}


}


/*
 * Outputs the specified waveform
 */
void outputWave() {
	wave_index = (wave_index + freq) % SAMPLES;	// increment or reset index

	switch(wave_type) {

	// SINE
	case 6:
		DAC_write(sine_array[wave_index]);
		break;

	// TRIANGLE
	case 7:
		DAC_write(triangle_array[wave_index]);
		break;

	// SAWTOOTH
	case 8:
		DAC_write(ramp_array[wave_index]);
		break;

	// SQUARE
	case 9:
		DAC_write(square_array[1]);
		break;

	// DEFAULT: SQUARE
	default:
		DAC_write(square_array[1]);
		break;

	}


}


/*
 * Sets the Waveform variables
 */
void setWave() {

	wave_index = 0;			// reset wave index
	TIM2->CR1 &= ~(1 << 0);	// disable counter
	TIM2->EGR |= (0x1);		// reset the counter

	switch(wave_type) {

	case 6:		// SINE

	case 7:		// TRIANGLE

	case 8:		// SAWTOOTH

		setARR( (CLOCK / (100*SAMPLES)) - 1 );		// set the ARR value

		TIM2->DIER |= (0x1);		// enable the ARR interrupt
		TIM2->DIER &= ~(0x2);		// disable CCR1 interrupt
		TIM2->SR &= ~(0x1);			// clear ARR interrupt flag
		TIM2->SR &= ~(0x2);			// clear CCR1 interrupt flag
		break;

	// SQUARE
	case 9:
		setARR (  (CLOCK / (freq * 100)) - 1 );						// set the ARR value
		setCCR1( ((CLOCK / (freq * 100)) * duty_cycle/10) - 1 );	// set CCR1 value

		TIM2->DIER |= (0x1);		// enable the ARR interrupt
		TIM2->DIER |= (0x2);		// enable the CCR1 interrupt


		TIM2->SR &= ~(0x1);			// clear interrupt flag
		TIM2->SR &= ~(0x2);			// clear interrupt flag

		break;

	// DEFAULT: SQUARE
	default:
		setARR (  (CLOCK / (freq * 100)) - 1 );						// set the ARR value
		setCCR1( ((CLOCK / (freq * 100)) * duty_cycle/10) - 1 );	// set CCR1 value

		TIM2->DIER |= (0x1);		// enable the ARR interrupt
		TIM2->DIER |= (0x2);		// enable the CCR1 interrupt

		TIM2->SR &= ~(0x1);			// clear interrupt flag
		TIM2->SR &= ~(0x2);			// clear interrupt flag

		break;
	}

	TIM2->CR1 |= (1 << 0);	// enable counter


}

/*
 * Initializes LUT for SINE, TRIANGLE, and RAMP (SAWTOOTH)
 */
void LUT_init() {

	// initialize SINE and RAMP LUT
	for(uint8_t i = 0; i < SAMPLES; i++) {
		sine_array[i] = DAC_volt_conv(AMPL_MV * sin(((2*PI)/SAMPLES) * i) + VBIAS_MV);
		ramp_array[i] = DAC_volt_conv((AMPL_MV + VBIAS_MV) * i/SAMPLES);
	}

	// initialize TRIANGLE LUT
	triangle_array[0] = triangle_array[SAMPLES-1] = ramp_array[0];
	for(uint8_t i = 1; i < SAMPLES/2; i++) {
		triangle_array[i] = ramp_array[i*2];
		triangle_array[SAMPLES-i-1] = ramp_array[i*2];
	}

	// initialize SQUARE LUT
	square_array[0] = DAC_volt_conv(0);
	square_array[1] = DAC_volt_conv(AMPL_MV + VBIAS_MV);

}


/*
 * Sets the CCR1 Value
 */
void setCCR1(uint32_t data) {
	TIM2->CCR1 = data;
}


/*
 * Sets the ARR Value
 */
void setARR(uint32_t data) {
	TIM2->ARR = data;
}



/*
 * Initializes the Interrupts on STM32
 */
void INTERRUPT_init() {
	__enable_irq();
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));	// TIM2 bit 28 of ISER0

	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);		// turn on clock to TIM2
	setWave();									// set the default wave (square, 100Hz, 50% DC)

	TIM2->CR1 |= TIM_CR1_CEN;					// enable timer
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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	//RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1){}
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
