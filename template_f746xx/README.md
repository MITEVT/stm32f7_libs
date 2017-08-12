# Template Program STM32F746xx
Written with the NUCLEO-F746GZ in mind, it performs the following functions:
* Enable Over Drive Mode
* Max out PLL at 216Mhz
* Appropriately prescale system clocks (AHB, APB1, APB2, FLASH)
* Switch SYSCLK input to PLL output
* Enable both clock outputs for PLL output (PA8) and SYSCLK output (PC9)
* Toggle GPIO PB7

## Programming Instructions
There are three ways to program the NUCLEO-F746GZ Board
### Manual Copy
Copy the .bin file generated in the bin directory to the enumerated USB Mass Storage Device. It should disconnect and then reenumerate. This approach is simple but it cause the ST-Link itself to reset and reenumerate.
### Scripted Copy
Run `make copy` which does the above but through Make. Ensure that the volume name under the variable MOUNT in the Makefile is correct
### OpenOCD (Recommended)
Run `make program` which uses OpenOCD to interface with the ST-Link and program the device. You need to ensure that the OCD_DIR and OCD variables in the Makefile are correct.
