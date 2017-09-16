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
L 7805 U?
U 1 1 59BD73FC
P 6500 1700
F 0 "U?" H 6650 1504 50  0000 C CNN
F 1 "7805" H 6500 1900 50  0000 C CNN
F 2 "" H 6500 1700 50  0001 C CNN
F 3 "" H 6500 1700 50  0001 C CNN
	1    6500 1700
	-1   0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 59BD746F
P 5950 1850
F 0 "C?" H 5975 1950 50  0000 L CNN
F 1 "C" H 5975 1750 50  0000 L CNN
F 2 "" H 5988 1700 50  0001 C CNN
F 3 "" H 5950 1850 50  0001 C CNN
	1    5950 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 1650 5950 1650
Wire Wire Line
	5950 1650 5950 1700
Wire Wire Line
	5950 2000 5950 2100
Wire Wire Line
	5950 2100 6500 2100
Wire Wire Line
	6500 2100 6500 1950
$EndSCHEMATC
