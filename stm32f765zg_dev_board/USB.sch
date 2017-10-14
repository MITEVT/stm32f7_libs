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
Sheet 8 9
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
L USB_OTG P1
U 1 1 59C6D352
P 2050 3450
F 0 "P1" H 2375 3325 50  0000 C CNN
F 1 "1050170001" H 2050 3650 50  0000 C CNN
F 2 "" V 2000 3350 50  0000 C CNN
F 3 "" V 2000 3350 50  0000 C CNN
	1    2050 3450
	0    -1   1    0   
$EndComp
Wire Wire Line
	2350 3250 2500 3250
Wire Wire Line
	2500 3250 2500 3100
Wire Wire Line
	2350 3650 2350 4050
$Comp
L GND #PWR?
U 1 1 59E2383E
P 2350 4050
F 0 "#PWR?" H 2350 3800 50  0001 C CNN
F 1 "GND" H 2350 3900 50  0000 C CNN
F 2 "" H 2350 4050 50  0000 C CNN
F 3 "" H 2350 4050 50  0000 C CNN
	1    2350 4050
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59E2388E
P 2600 3700
F 0 "R1" V 2680 3700 50  0000 C CNN
F 1 "100K" V 2600 3700 50  0000 C CNN
F 2 "" V 2530 3700 50  0000 C CNN
F 3 "" H 2600 3700 50  0000 C CNN
	1    2600 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3550 2600 3550
Wire Wire Line
	2600 3850 1950 3850
$Comp
L USBLC6-2SC6 U1
U 1 1 59E23FCE
P 3450 3750
F 0 "U1" H 3450 3950 60  0000 C CNN
F 1 "USBLC6-2SC6" H 3450 3450 60  0000 C CNN
F 2 "" H 2900 4100 60  0001 C CNN
F 3 "" H 2900 4100 60  0001 C CNN
	1    3450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3350 4800 3350
Wire Wire Line
	2350 3450 4800 3450
Wire Wire Line
	3100 3650 3050 3650
Wire Wire Line
	3050 3650 3050 3350
Connection ~ 3050 3350
Wire Wire Line
	3100 3800 3050 3800
Wire Wire Line
	3050 3800 3050 4050
$Comp
L GND #PWR?
U 1 1 59E24128
P 3050 4050
F 0 "#PWR?" H 3050 3800 50  0001 C CNN
F 1 "GND" H 3050 3900 50  0000 C CNN
F 2 "" H 3050 4050 50  0000 C CNN
F 3 "" H 3050 4050 50  0000 C CNN
	1    3050 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3950 2950 3950
Wire Wire Line
	2950 3950 2950 3450
Connection ~ 2950 3450
Wire Wire Line
	3800 3650 3850 3650
Wire Wire Line
	3850 3650 3850 3350
Connection ~ 3850 3350
Wire Wire Line
	3800 3950 3950 3950
Wire Wire Line
	3950 3950 3950 3450
Connection ~ 3950 3450
Wire Wire Line
	3800 3800 4200 3800
Wire Wire Line
	4200 3800 4200 3950
$Comp
L C C1
U 1 1 59E2424B
P 4200 4100
F 0 "C1" H 4225 4200 50  0000 L CNN
F 1 "100nF" H 4225 4000 50  0000 L CNN
F 2 "" H 4238 3950 50  0000 C CNN
F 3 "" H 4200 4100 50  0000 C CNN
	1    4200 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4250 4200 4400
$Comp
L GND #PWR?
U 1 1 59E242AD
P 4200 4400
F 0 "#PWR?" H 4200 4150 50  0001 C CNN
F 1 "GND" H 4200 4250 50  0000 C CNN
F 2 "" H 4200 4400 50  0000 C CNN
F 3 "" H 4200 4400 50  0000 C CNN
	1    4200 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3450 4600 3800
Connection ~ 4600 3450
$Comp
L R R2
U 1 1 59E243A6
P 4600 3950
F 0 "R2" V 4680 3950 50  0000 C CNN
F 1 "1K5" V 4600 3950 50  0000 C CNN
F 2 "" V 4530 3950 50  0000 C CNN
F 3 "" H 4600 3950 50  0000 C CNN
	1    4600 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4100 4600 4300
Wire Wire Line
	4600 4300 5100 4300
Wire Wire Line
	5100 4300 5100 4100
$Comp
L Q_NPN_BCE Q1
U 1 1 59E2446D
P 5200 3900
F 0 "Q1" H 5400 3950 50  0000 L CNN
F 1 "T19013" H 5400 3850 50  0000 L CNN
F 2 "" H 5400 4000 50  0000 C CNN
F 3 "" H 5200 3900 50  0000 C CNN
	1    5200 3900
	-1   0    0    -1  
$EndComp
$Comp
L +U5V #PWR?
U 1 1 59E245FB
P 4200 3800
F 0 "#PWR?" H 4200 3650 50  0001 C CNN
F 1 "+U5V" H 4200 3940 50  0000 C CNN
F 2 "" H 4200 3800 50  0000 C CNN
F 3 "" H 4200 3800 50  0000 C CNN
	1    4200 3800
	1    0    0    -1  
$EndComp
$Comp
L +U5V #PWR?
U 1 1 59E24646
P 2500 3100
F 0 "#PWR?" H 2500 2950 50  0001 C CNN
F 1 "+U5V" H 2500 3240 50  0000 C CNN
F 2 "" H 2500 3100 50  0000 C CNN
F 3 "" H 2500 3100 50  0000 C CNN
	1    2500 3100
	1    0    0    -1  
$EndComp
$Comp
L +U5V #PWR?
U 1 1 59E24696
P 5900 3350
F 0 "#PWR?" H 5900 3200 50  0001 C CNN
F 1 "+U5V" H 5900 3490 50  0000 C CNN
F 2 "" H 5900 3350 50  0000 C CNN
F 3 "" H 5900 3350 50  0000 C CNN
	1    5900 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3700 5100 3650
$Comp
L +3V3 #PWR?
U 1 1 59E24763
P 5100 3650
F 0 "#PWR?" H 5100 3500 50  0001 C CNN
F 1 "+3V3" H 5100 3790 50  0000 C CNN
F 2 "" H 5100 3650 50  0000 C CNN
F 3 "" H 5100 3650 50  0000 C CNN
	1    5100 3650
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 59E247EA
P 5900 4150
F 0 "R4" V 5980 4150 50  0000 C CNN
F 1 "36K" V 5900 4150 50  0000 C CNN
F 2 "" V 5830 4150 50  0000 C CNN
F 3 "" H 5900 4150 50  0000 C CNN
	1    5900 4150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 59E2489A
P 5900 3650
F 0 "R3" V 5980 3650 50  0000 C CNN
F 1 "10K" V 5900 3650 50  0000 C CNN
F 2 "" V 5830 3650 50  0000 C CNN
F 3 "" H 5900 3650 50  0000 C CNN
	1    5900 3650
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 59E249AC
P 6550 3900
F 0 "R5" V 6630 3900 50  0000 C CNN
F 1 "100" V 6550 3900 50  0000 C CNN
F 2 "" V 6480 3900 50  0000 C CNN
F 3 "" H 6550 3900 50  0000 C CNN
	1    6550 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 3350 5900 3500
Wire Wire Line
	5900 3800 5900 4000
Wire Wire Line
	5900 4300 5900 4400
$Comp
L GND #PWR?
U 1 1 59E24A83
P 5900 4400
F 0 "#PWR?" H 5900 4150 50  0001 C CNN
F 1 "GND" H 5900 4250 50  0000 C CNN
F 2 "" H 5900 4400 50  0000 C CNN
F 3 "" H 5900 4400 50  0000 C CNN
	1    5900 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3900 6400 3900
Connection ~ 5900 3900
Wire Wire Line
	7000 3900 6700 3900
Text HLabel 4800 3350 2    60   BiDi ~ 0
STLINK_USB_DM
Text HLabel 4800 3450 2    60   BiDi ~ 0
STLINK_USB_DP
Text HLabel 7000 3900 2    60   Output ~ 0
USB_RENUMn
$EndSCHEMATC
