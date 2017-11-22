target remote localhost:3333
file bin/uart_hal.elf
mon reset halt
tbreak main
c

define reset
	mon reset halt
end