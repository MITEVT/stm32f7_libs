# Template Program STM32F746xx
Written with the NUCLEO-F746GZ in mind, it performs the following functions:
-Enable Over Drive Mode
-Max out PLL at 216Mhz
-Appropriately prescale system clocks (AHB, APB1, APB2, FLASH)
-Switch SYSCLK input to PLL output
-Enable both clock outputs for PLL output (PA8) and SYSCLK output (PC9)
-Toggle GPIO PB7

## Programming Instructions
Copy the .bin file generated in the bin directory to the enumerated USB Mass Storage Device. It should disconnect and then reenumerate.
