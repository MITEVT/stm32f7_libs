EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MITEVT_ANALOG
LIBS:MITEVT_CONTACTORS
LIBS:MITEVT_interface
LIBS:MITEVT_mcontrollers
LIBS:MITEVT_OPTO
LIBS:MITEVT_power
LIBS:MITEVT_REG
LIBS:MITEVT_relay
LIBS:stm32f765zg_dev_board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 9
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L AP2210 U?
U 1 1 59BD77DA
P 4250 2800
F 0 "U?" H 4300 2850 60  0000 C CNN
F 1 "AP2210" H 4500 2750 60  0000 C CNN
F 2 "" H 3950 3000 60  0001 C CNN
F 3 "" H 3950 3000 60  0001 C CNN
F 4 "AP2210K-5.0TRG1DICT-ND" H 4250 2800 60  0001 C CNN "Digikey "
	1    4250 2800
	1    0    0    -1  
$EndComp
$Comp
L AP2210 U?
U 1 1 59BD7F95
P 5350 2800
F 0 "U?" H 5400 2850 60  0000 C CNN
F 1 "AP2210" H 5600 2750 60  0000 C CNN
F 2 "" H 5050 3000 60  0001 C CNN
F 3 "" H 5050 3000 60  0001 C CNN
F 4 "ap2210n-3.3trg1" H 5350 2800 60  0001 C CNN "Digikey"
	1    5350 2800
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD81EB
P 5050 3200
F 0 "C?" H 5075 3300 50  0000 L CNN
F 1 "1uF" H 5075 3100 50  0000 L CNN
F 2 "" H 5088 3050 50  0000 C CNN
F 3 "" H 5050 3200 50  0000 C CNN
	1    5050 3200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD8241
P 6150 3150
F 0 "C?" H 6175 3250 50  0000 L CNN
F 1 "2.2uF" H 6175 3050 50  0000 L CNN
F 2 "" H 6188 3000 50  0000 C CNN
F 3 "" H 6150 3150 50  0000 C CNN
	1    6150 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 59BD82C4
P 3400 3600
F 0 "#PWR?" H 3400 3350 50  0001 C CNN
F 1 "GND" H 3400 3450 50  0000 C CNN
F 2 "" H 3400 3600 50  0000 C CNN
F 3 "" H 3400 3600 50  0000 C CNN
	1    3400 3600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD830C
P 3950 3200
F 0 "C?" H 3975 3300 50  0000 L CNN
F 1 "1uF" H 3975 3100 50  0000 L CNN
F 2 "" H 3988 3050 50  0000 C CNN
F 3 "" H 3950 3200 50  0000 C CNN
	1    3950 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2950 3950 3050
Wire Wire Line
	3150 2950 4050 2950
Wire Wire Line
	4500 3450 4500 3300
Wire Wire Line
	3400 3450 6150 3450
Wire Wire Line
	3950 3450 3950 3350
Connection ~ 4500 3450
Wire Wire Line
	4950 2950 5150 2950
Wire Wire Line
	5050 2950 5050 3050
Wire Wire Line
	5050 3350 5050 3450
Connection ~ 5050 3450
Wire Wire Line
	3400 3050 3400 3600
Connection ~ 3950 3450
Connection ~ 3950 2950
Connection ~ 5050 2950
Wire Wire Line
	6050 2950 6300 2950
Wire Wire Line
	6150 2950 6150 3000
Wire Wire Line
	6150 3450 6150 3300
Wire Wire Line
	5600 3300 5600 3450
Connection ~ 5600 3450
$Comp
L +12V #PWR?
U 1 1 59BD8A5D
P 3400 2800
F 0 "#PWR?" H 3400 2650 50  0001 C CNN
F 1 "+12V" H 3400 2940 50  0000 C CNN
F 2 "" H 3400 2800 50  0000 C CNN
F 3 "" H 3400 2800 50  0000 C CNN
	1    3400 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2800 3400 2950
$Comp
L CONN_01X02 P?
U 1 1 59BD8ACE
P 2950 3000
F 0 "P?" H 2950 3150 50  0000 C CNN
F 1 "CONN_01X02" V 3050 3000 50  0000 C CNN
F 2 "" H 2950 3000 50  0000 C CNN
F 3 "" H 2950 3000 50  0000 C CNN
	1    2950 3000
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 59BD8D1E
P 6300 2800
F 0 "#PWR?" H 6300 2650 50  0001 C CNN
F 1 "+3.3V" H 6300 2940 50  0000 C CNN
F 2 "" H 6300 2800 50  0000 C CNN
F 3 "" H 6300 2800 50  0000 C CNN
	1    6300 2800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 59BD8D44
P 5100 2700
F 0 "#PWR?" H 5100 2550 50  0001 C CNN
F 1 "+5V" H 5100 2840 50  0000 C CNN
F 2 "" H 5100 2700 50  0000 C CNN
F 3 "" H 5100 2700 50  0000 C CNN
	1    5100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 2700 5100 2950
Connection ~ 5100 2950
Connection ~ 3400 2950
Wire Wire Line
	3150 3050 3400 3050
Connection ~ 3400 3450
Wire Wire Line
	6300 2950 6300 2800
Connection ~ 6150 2950
$EndSCHEMATC
