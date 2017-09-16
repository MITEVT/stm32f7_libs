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
Sheet 5 9
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
L ISO1050 U?
U 1 1 59BD73B2
P 5450 2700
F 0 "U?" H 5100 3050 60  0000 C CNN
F 1 "ISO1050" H 5750 3050 60  0000 C CNN
F 2 "" H 5750 3050 60  0000 C CNN
F 3 "" H 5750 3050 60  0000 C CNN
	1    5450 2700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD746F
P 6900 2650
F 0 "C?" H 6925 2750 50  0000 L CNN
F 1 "C" H 6925 2550 50  0000 L CNN
F 2 "" H 6938 2500 50  0001 C CNN
F 3 "" H 6900 2650 50  0001 C CNN
	1    6900 2650
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 59BD76FD
P 8650 2350
F 0 "#FLG?" H 8650 2425 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 2500 50  0000 C CNN
F 2 "" H 8650 2350 50  0001 C CNN
F 3 "" H 8650 2350 50  0001 C CNN
	1    8650 2350
	1    0    0    -1  
$EndComp
Text Label 7950 2450 0    60   ~ 0
CAN1_POW_EXT
$Comp
L PWR_FLAG #FLG?
U 1 1 59BD7804
P 8650 3050
F 0 "#FLG?" H 8650 3125 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 3200 50  0000 C CNN
F 2 "" H 8650 3050 50  0001 C CNN
F 3 "" H 8650 3050 50  0001 C CNN
	1    8650 3050
	-1   0    0    1   
$EndComp
Text Label 7950 2900 0    60   ~ 0
CAN2_POW_GND
$Comp
L ISO1050 U?
U 1 1 59BD7C5A
P 5450 4550
F 0 "U?" H 5100 4900 60  0000 C CNN
F 1 "ISO1050" H 5750 4900 60  0000 C CNN
F 2 "" H 5750 4900 60  0000 C CNN
F 3 "" H 5750 4900 60  0000 C CNN
	1    5450 4550
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD7C66
P 6900 4500
F 0 "C?" H 6925 4600 50  0000 L CNN
F 1 "C" H 6925 4400 50  0000 L CNN
F 2 "" H 6938 4350 50  0001 C CNN
F 3 "" H 6900 4500 50  0001 C CNN
	1    6900 4500
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 59BD7C73
P 8650 4200
F 0 "#FLG?" H 8650 4275 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 4350 50  0000 C CNN
F 2 "" H 8650 4200 50  0001 C CNN
F 3 "" H 8650 4200 50  0001 C CNN
	1    8650 4200
	1    0    0    -1  
$EndComp
Text Label 7950 4300 0    60   ~ 0
CAN0_POW_EXT
$Comp
L PWR_FLAG #FLG?
U 1 1 59BD7C7C
P 8650 4900
F 0 "#FLG?" H 8650 4975 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 5050 50  0000 C CNN
F 2 "" H 8650 4900 50  0001 C CNN
F 3 "" H 8650 4900 50  0001 C CNN
	1    8650 4900
	-1   0    0    1   
$EndComp
Text Label 7950 4750 0    60   ~ 0
CAN0_POW_GND
$Comp
L GND #PWR?
U 1 1 59BD7D14
P 5350 5200
F 0 "#PWR?" H 5350 4950 50  0001 C CNN
F 1 "GND" H 5350 5050 50  0000 C CNN
F 2 "" H 5350 5200 50  0001 C CNN
F 3 "" H 5350 5200 50  0001 C CNN
	1    5350 5200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 59BD7D34
P 5350 3850
F 0 "#PWR?" H 5350 3700 50  0001 C CNN
F 1 "+3.3V" H 5350 3990 50  0000 C CNN
F 2 "" H 5350 3850 50  0001 C CNN
F 3 "" H 5350 3850 50  0001 C CNN
	1    5350 3850
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 59BD7D9B
P 5350 2000
F 0 "#PWR?" H 5350 1850 50  0001 C CNN
F 1 "+3.3V" H 5350 2140 50  0000 C CNN
F 2 "" H 5350 2000 50  0001 C CNN
F 3 "" H 5350 2000 50  0001 C CNN
	1    5350 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 59BD7E25
P 5350 3400
F 0 "#PWR?" H 5350 3150 50  0001 C CNN
F 1 "GND" H 5350 3250 50  0000 C CNN
F 2 "" H 5350 3400 50  0001 C CNN
F 3 "" H 5350 3400 50  0001 C CNN
	1    5350 3400
	1    0    0    -1  
$EndComp
Text HLabel 4350 2600 0    60   Input ~ 0
CAN1_TXD
Text HLabel 4350 2500 0    60   Input ~ 0
CAN1_RXD
Text HLabel 4300 4350 0    60   Input ~ 0
CAN0_RXD
Text HLabel 4300 4450 0    60   Input ~ 0
CAN0_TXD
$Comp
L CONN_01X04 J?
U 1 1 59BD817B
P 9900 4450
F 0 "J?" H 9900 4700 50  0000 C CNN
F 1 "CONN_01X04" V 10000 4450 50  0000 C CNN
F 2 "" H 9900 4450 50  0001 C CNN
F 3 "" H 9900 4450 50  0001 C CNN
	1    9900 4450
	1    0    0    -1  
$EndComp
Text Label 6350 2200 0    60   ~ 0
CAN1_ISOL
Text Label 6200 3400 0    60   ~ 0
CAN1_ISOH
Wire Wire Line
	7050 2450 6900 2450
Wire Wire Line
	6900 1800 6900 2450
Wire Wire Line
	6900 2450 6900 2500
Wire Wire Line
	6900 2800 6900 2900
Wire Wire Line
	6900 2900 6900 3600
Wire Wire Line
	6900 2900 7500 2900
Wire Wire Line
	7500 2900 8650 2900
Wire Wire Line
	7500 2900 7500 2800
Connection ~ 7500 2900
Wire Wire Line
	7950 2450 8650 2450
Wire Wire Line
	8650 2450 9700 2450
Wire Wire Line
	8650 2450 8650 2350
Wire Wire Line
	8650 2550 8650 2900
Wire Wire Line
	8650 2900 8650 3050
Wire Wire Line
	6900 3600 5550 3600
Wire Wire Line
	5550 3600 5550 3200
Connection ~ 6900 2900
Wire Wire Line
	5550 2200 5550 1800
Wire Wire Line
	5550 1800 6900 1800
Connection ~ 6900 2450
Wire Wire Line
	7050 4300 6900 4300
Wire Wire Line
	6900 3900 6900 4300
Wire Wire Line
	6900 4300 6900 4350
Wire Wire Line
	6900 4650 6900 4750
Wire Wire Line
	6900 4750 6900 5400
Wire Wire Line
	6900 4750 7500 4750
Wire Wire Line
	7500 4750 8650 4750
Wire Wire Line
	7500 4750 7500 4650
Connection ~ 7500 4750
Wire Wire Line
	8650 4300 8650 4200
Wire Wire Line
	8650 4400 8650 4750
Wire Wire Line
	8650 4750 8650 4900
Wire Wire Line
	6900 5400 5550 5400
Wire Wire Line
	5550 5400 5550 5050
Connection ~ 6900 4750
Wire Wire Line
	5550 4050 5550 3900
Wire Wire Line
	5550 3900 6900 3900
Connection ~ 6900 4300
Wire Wire Line
	6050 4450 6100 4450
Wire Wire Line
	6100 4450 6150 4450
Wire Wire Line
	6050 4650 6100 4650
Wire Wire Line
	6100 4650 6150 4650
Wire Wire Line
	5350 5050 5350 5200
Wire Wire Line
	5350 3850 5350 4050
Wire Wire Line
	5350 2000 5350 2200
Wire Wire Line
	5350 3200 5350 3400
Wire Wire Line
	4850 2500 4350 2500
Wire Wire Line
	4850 2600 4350 2600
Wire Wire Line
	4300 4350 4850 4350
Wire Wire Line
	4300 4450 4850 4450
Connection ~ 8650 2450
Wire Wire Line
	9700 2550 8650 2550
Connection ~ 8650 2900
Wire Wire Line
	9700 2650 9450 2650
Wire Wire Line
	9450 2750 9700 2750
Text Label 9450 2750 2    60   ~ 0
CAN1_ISOH
Text Label 9450 2650 2    60   ~ 0
CAN1_ISOL
Wire Wire Line
	9700 4400 8650 4400
Wire Wire Line
	9700 4500 9450 4500
Wire Wire Line
	9450 4600 9700 4600
Text Label 9450 4600 2    60   ~ 0
CAN0_ISOH
Text Label 9450 4500 2    60   ~ 0
CAN0_ISOL
Connection ~ 8650 4750
Wire Wire Line
	7950 4300 8650 4300
Wire Wire Line
	8650 4300 9700 4300
Connection ~ 8650 4300
$Comp
L CONN_01X04 J?
U 1 1 59BD7F5D
P 9900 2600
F 0 "J?" H 9900 2850 50  0000 C CNN
F 1 "CONN_01X04" V 10000 2600 50  0000 C CNN
F 2 "" H 9900 2600 50  0001 C CNN
F 3 "" H 9900 2600 50  0001 C CNN
	1    9900 2600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 59BD811F
P 6300 2600
F 0 "R?" V 6380 2600 50  0000 C CNN
F 1 "R" V 6300 2600 50  0000 C CNN
F 2 "" V 6230 2600 50  0001 C CNN
F 3 "" H 6300 2600 50  0001 C CNN
	1    6300 2600
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 59BD82FA
P 6300 2800
F 0 "R?" V 6380 2800 50  0000 C CNN
F 1 "R" V 6300 2800 50  0000 C CNN
F 2 "" V 6230 2800 50  0001 C CNN
F 3 "" H 6300 2800 50  0001 C CNN
	1    6300 2800
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 59BD8348
P 6650 2850
F 0 "C?" H 6675 2950 50  0000 L CNN
F 1 "C" H 6675 2750 50  0000 L CNN
F 2 "" H 6688 2700 50  0001 C CNN
F 3 "" H 6650 2850 50  0001 C CNN
	1    6650 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2600 6450 2700
Wire Wire Line
	6450 2700 6450 2800
Wire Wire Line
	6450 2700 6650 2700
Connection ~ 6450 2700
Wire Wire Line
	6050 2600 6100 2600
Wire Wire Line
	6100 2600 6150 2600
Wire Wire Line
	6050 2800 6100 2800
Wire Wire Line
	6100 2800 6150 2800
Wire Wire Line
	6100 2600 6100 2200
Wire Wire Line
	6100 2200 6350 2200
Connection ~ 6100 2600
Wire Wire Line
	6100 2800 6100 3400
Wire Wire Line
	6100 3400 6200 3400
Connection ~ 6100 2800
Wire Wire Line
	6650 3000 6650 3050
$Comp
L GND #PWR?
U 1 1 59BD8744
P 6650 3050
F 0 "#PWR?" H 6650 2800 50  0001 C CNN
F 1 "GND" H 6650 2900 50  0000 C CNN
F 2 "" H 6650 3050 50  0001 C CNN
F 3 "" H 6650 3050 50  0001 C CNN
	1    6650 3050
	1    0    0    -1  
$EndComp
Text Label 6350 4050 0    60   ~ 0
CAN0_ISOL
Text Label 6200 5250 0    60   ~ 0
CAN0_ISOH
$Comp
L R R?
U 1 1 59BD8941
P 6300 4450
F 0 "R?" V 6380 4450 50  0000 C CNN
F 1 "R" V 6300 4450 50  0000 C CNN
F 2 "" V 6230 4450 50  0001 C CNN
F 3 "" H 6300 4450 50  0001 C CNN
	1    6300 4450
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 59BD8947
P 6300 4650
F 0 "R?" V 6380 4650 50  0000 C CNN
F 1 "R" V 6300 4650 50  0000 C CNN
F 2 "" V 6230 4650 50  0001 C CNN
F 3 "" H 6300 4650 50  0001 C CNN
	1    6300 4650
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 59BD894D
P 6650 4700
F 0 "C?" H 6675 4800 50  0000 L CNN
F 1 "C" H 6675 4600 50  0000 L CNN
F 2 "" H 6688 4550 50  0001 C CNN
F 3 "" H 6650 4700 50  0001 C CNN
	1    6650 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4450 6450 4550
Wire Wire Line
	6450 4550 6450 4650
Wire Wire Line
	6450 4550 6650 4550
Connection ~ 6450 4550
Wire Wire Line
	6100 4450 6100 4050
Wire Wire Line
	6100 4050 6350 4050
Connection ~ 6100 4450
Wire Wire Line
	6100 4650 6100 5250
Wire Wire Line
	6100 5250 6200 5250
Connection ~ 6100 4650
Wire Wire Line
	6650 4850 6650 4900
$Comp
L GND #PWR?
U 1 1 59BD8962
P 6650 4900
F 0 "#PWR?" H 6650 4650 50  0001 C CNN
F 1 "GND" H 6650 4750 50  0000 C CNN
F 2 "" H 6650 4900 50  0001 C CNN
F 3 "" H 6650 4900 50  0001 C CNN
	1    6650 4900
	1    0    0    -1  
$EndComp
$Comp
L AP2210 U?
U 1 1 59BD8EF5
P 7750 2300
F 0 "U?" H 7800 2350 60  0000 C CNN
F 1 "AP2210" H 8000 2250 60  0000 C CNN
F 2 "" H 7450 2500 60  0001 C CNN
F 3 "" H 7450 2500 60  0001 C CNN
	1    7750 2300
	-1   0    0    -1  
$EndComp
$Comp
L AP2210 U?
U 1 1 59BD924C
P 7750 4150
F 0 "U?" H 7800 4200 60  0000 C CNN
F 1 "AP2210" H 8000 4100 60  0000 C CNN
F 2 "" H 7450 4350 60  0001 C CNN
F 3 "" H 7450 4350 60  0001 C CNN
	1    7750 4150
	-1   0    0    -1  
$EndComp
$EndSCHEMATC