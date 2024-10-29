# Compilation of Projects and assignments for CPE316 Microcontrollers and Embedded Applications.<br /> #




  ### P1: Electronic Lock
  
  > This project is about building a virtual electronic lock using an LCD display, a keypad, and an STM32L476RG microcontroller on a NUCLEO development board. The onboard LED will act as a visual indicator for the lock’s state, lighting up when locked and turning off when unlocked. Users can interact with the lock via a 4-digit PIN entered on the keypad, while the LCD provides feedback, showing prompts like “LOCKED” or “ENTER KEY” to guide the user.
> The digital lock powers up in a locked state, requiring a PIN entry to unlock. The LCD will display the PIN digits as they're entered, and pressing * will clear the entered numbers to restart the process. Once unlocked, the user can relock without re-entering the PIN, and when unlocked, the PIN can also be reset.

  ### P2: Function Generator
  
  > This project focuses on designing a function generator using a microcontroller and an external Digital-to-Analog Converter (DAC) with an SPI interface. The primary goal is to produce various analog waveforms, including sine, triangle, sawtooth, and square waves. Users will interact with the system through a keypad to select the waveform type, adjust the frequency, and set the duty cycle for the square wave, while an LCD display will provide real-time feedback on the current settings.
The function generator will be capable of producing waveforms with a peak-to-peak voltage of 3.0V, DC-biased at 1.5V. Frequencies will be adjustable from 100 Hz to 500 Hz, with a maximum deviation of ±2.5 Hz. The square wave will allow duty cycle adjustments in 10% increments, ranging from 10% to 90%, with a reset option to 50%. To ensure accurate waveform generation, the microcontroller will utilize timers and interrupts for precise timing control, achieving a sample rate of at least 75% of the calculated maximum resolution.



