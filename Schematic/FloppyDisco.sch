EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Floppy Disco"
Date "2022-01-06"
Rev "1.0"
Comp "Christoffer Karlsson"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:USB_A J1
U 1 1 61D710B5
P 2450 2500
F 0 "J1" H 2507 2967 50  0000 C CNN
F 1 "USB connector" H 2507 2876 50  0000 C CNN
F 2 "" H 2600 2450 50  0001 C CNN
F 3 " ~" H 2600 2450 50  0001 C CNN
	1    2450 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 61D71BD8
P 3600 2600
F 0 "C1" H 3718 2646 50  0000 L CNN
F 1 "100µ" H 3718 2555 50  0000 L CNN
F 2 "" H 3638 2450 50  0001 C CNN
F 3 "~" H 3600 2600 50  0001 C CNN
	1    3600 2600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 61D76106
P 4700 2400
F 0 "J3" H 4780 2392 50  0000 L CNN
F 1 "Floppy disk power connector" H 4780 2301 50  0000 L CNN
F 2 "" H 4700 2400 50  0001 C CNN
F 3 "~" H 4700 2400 50  0001 C CNN
	1    4700 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2300 3600 2300
Connection ~ 3600 2300
Wire Wire Line
	2450 2900 2350 2900
Connection ~ 2450 2900
NoConn ~ 2750 2500
NoConn ~ 2750 2600
Wire Wire Line
	4500 2500 4500 2400
Wire Wire Line
	4500 2400 4150 2400
Connection ~ 4500 2400
Wire Wire Line
	4150 2400 4150 2900
Wire Wire Line
	3600 2300 3600 2450
Wire Wire Line
	3600 2750 3600 2900
Connection ~ 3600 2900
Wire Wire Line
	3600 2900 4150 2900
$Comp
L Connector:DIN-5_180degree J2
U 1 1 61D79136
P 2750 4350
F 0 "J2" H 2750 4075 50  0000 C CNN
F 1 "MIDI connector" H 2750 3984 50  0000 C CNN
F 2 "" H 2750 4350 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 2750 4350 50  0001 C CNN
	1    2750 4350
	1    0    0    -1  
$EndComp
$Comp
L Isolator:EL817 U1
U 1 1 61D7A019
P 4300 3950
F 0 "U1" H 4300 4275 50  0000 C CNN
F 1 "EL817" H 4300 4184 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 4100 3750 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL817.pdf" H 4300 3950 50  0001 L CNN
	1    4300 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3850 2250 4250
Wire Wire Line
	2250 4250 2450 4250
$Comp
L Device:R_US R1
U 1 1 61D7AD39
P 3550 3850
F 0 "R1" V 3345 3850 50  0000 C CNN
F 1 "220ohm" V 3436 3850 50  0000 C CNN
F 2 "" V 3590 3840 50  0001 C CNN
F 3 "~" H 3550 3850 50  0001 C CNN
	1    3550 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 3850 3700 3850
Wire Wire Line
	3400 3850 2250 3850
Wire Wire Line
	3050 4250 3300 4250
Wire Wire Line
	3300 4250 3300 4050
Wire Wire Line
	3300 4050 4000 4050
NoConn ~ 3050 4350
NoConn ~ 2750 4050
NoConn ~ 2450 4350
$Comp
L power:Earth #PWR02
U 1 1 61D7CB9F
P 3600 2900
F 0 "#PWR02" H 3600 2650 50  0001 C CNN
F 1 "Earth" H 3600 2750 50  0001 C CNN
F 2 "" H 3600 2900 50  0001 C CNN
F 3 "~" H 3600 2900 50  0001 C CNN
	1    3600 2900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 61D7CD24
P 3600 2300
F 0 "#PWR01" H 3600 2150 50  0001 C CNN
F 1 "+5V" H 3615 2473 50  0000 C CNN
F 2 "" H 3600 2300 50  0001 C CNN
F 3 "" H 3600 2300 50  0001 C CNN
	1    3600 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 61D7CEEC
P 4800 3600
F 0 "#PWR03" H 4800 3450 50  0001 C CNN
F 1 "+5V" H 4815 3773 50  0000 C CNN
F 2 "" H 4800 3600 50  0001 C CNN
F 3 "" H 4800 3600 50  0001 C CNN
	1    4800 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 61D7D1CE
P 4800 4400
F 0 "R2" H 4868 4446 50  0000 L CNN
F 1 "180ohm" H 4868 4355 50  0000 L CNN
F 2 "" V 4840 4390 50  0001 C CNN
F 3 "~" H 4800 4400 50  0001 C CNN
	1    4800 4400
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR04
U 1 1 61D7D724
P 4800 4750
F 0 "#PWR04" H 4800 4500 50  0001 C CNN
F 1 "Earth" H 4800 4600 50  0001 C CNN
F 2 "" H 4800 4750 50  0001 C CNN
F 3 "~" H 4800 4750 50  0001 C CNN
	1    4800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3600 4800 3850
Wire Wire Line
	4800 3850 4600 3850
Wire Wire Line
	4600 4050 4800 4050
Wire Wire Line
	4800 4050 4800 4250
Wire Wire Line
	4800 4550 4800 4750
Wire Wire Line
	6700 4650 6050 4650
Wire Wire Line
	6050 4650 6050 4850
$Comp
L power:Earth #PWR05
U 1 1 61D82CAF
P 6050 4850
F 0 "#PWR05" H 6050 4600 50  0001 C CNN
F 1 "Earth" H 6050 4700 50  0001 C CNN
F 2 "" H 6050 4850 50  0001 C CNN
F 3 "~" H 6050 4850 50  0001 C CNN
	1    6050 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 61D8384A
P 7100 2450
F 0 "C2" H 7192 2496 50  0000 L CNN
F 1 "0.1µ" H 7192 2405 50  0000 L CNN
F 2 "" H 7100 2450 50  0001 C CNN
F 3 "~" H 7100 2450 50  0001 C CNN
	1    7100 2450
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR07
U 1 1 61D840D1
P 7100 2550
F 0 "#PWR07" H 7100 2300 50  0001 C CNN
F 1 "Earth" H 7100 2400 50  0001 C CNN
F 2 "" H 7100 2550 50  0001 C CNN
F 3 "~" H 7100 2550 50  0001 C CNN
	1    7100 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR06
U 1 1 61D843D9
P 6900 2200
F 0 "#PWR06" H 6900 2050 50  0001 C CNN
F 1 "+5V" H 6915 2373 50  0000 C CNN
F 2 "" H 6900 2200 50  0001 C CNN
F 3 "" H 6900 2200 50  0001 C CNN
	1    6900 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 2650 6900 2250
Wire Wire Line
	7100 2350 7100 2250
Wire Wire Line
	7100 2250 6900 2250
Connection ~ 6900 2250
Wire Wire Line
	6900 2250 6900 2200
$Comp
L Connector_Generic:Conn_01x34 J4
U 1 1 61D85223
P 8800 3700
F 0 "J4" H 8880 3692 50  0000 L CNN
F 1 "Floppy disk connector" H 8880 3601 50  0000 L CNN
F 2 "" H 8800 3700 50  0001 C CNN
F 3 "~" H 8800 3700 50  0001 C CNN
	1    8800 3700
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_Nano_Every A1
U 1 1 61D7E5D2
P 6700 3650
F 0 "A1" H 6700 2561 50  0000 C CNN
F 1 "Arduino pro mini" H 6700 2470 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6700 3650 50  0001 C CIN
F 3 "https://content.arduino.cc/assets/NANOEveryV3.0_sch.pdf" H 6700 3650 50  0001 C CNN
	1    6700 3650
	1    0    0    -1  
$EndComp
Connection ~ 4800 4050
Text GLabel 6000 3250 0    50   Output ~ 0
SELECT
Text GLabel 6000 3350 0    50   Output ~ 0
DIR
Text GLabel 6000 3450 0    50   Output ~ 0
STEP
Wire Wire Line
	6200 3250 6000 3250
Wire Wire Line
	6200 3350 6000 3350
Wire Wire Line
	6200 3450 6000 3450
Text GLabel 8350 3800 0    50   Input ~ 0
DIR
Text GLabel 8350 4000 0    50   Input ~ 0
STEP
Text GLabel 8350 3200 0    50   Input ~ 0
SELECT
Wire Wire Line
	8350 3200 8600 3200
Wire Wire Line
	8600 3800 8350 3800
Wire Wire Line
	8600 4000 8350 4000
Wire Wire Line
	8600 3100 7900 3100
Wire Wire Line
	8600 3900 7900 3900
Wire Wire Line
	7900 3900 7900 4000
$Comp
L power:Earth #PWR08
U 1 1 61D95916
P 7900 4000
F 0 "#PWR08" H 7900 3750 50  0001 C CNN
F 1 "Earth" H 7900 3850 50  0001 C CNN
F 2 "" H 7900 4000 50  0001 C CNN
F 3 "~" H 7900 4000 50  0001 C CNN
	1    7900 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3700 7900 3700
Wire Wire Line
	7900 3700 7900 3900
Connection ~ 7900 3900
Wire Wire Line
	7900 3100 7900 3700
Connection ~ 7900 3700
NoConn ~ 6200 2950
NoConn ~ 6200 3150
NoConn ~ 6200 3550
NoConn ~ 6200 3650
NoConn ~ 6200 3750
NoConn ~ 6200 3850
NoConn ~ 6200 3950
NoConn ~ 6200 4150
NoConn ~ 6200 4250
NoConn ~ 6200 4350
NoConn ~ 7200 4350
NoConn ~ 7200 4250
NoConn ~ 7200 4150
NoConn ~ 7200 4050
NoConn ~ 7200 3950
NoConn ~ 7200 3850
NoConn ~ 7200 3750
NoConn ~ 7200 3650
NoConn ~ 7200 3450
NoConn ~ 7200 3150
NoConn ~ 7200 3050
NoConn ~ 6800 2650
NoConn ~ 6600 2650
NoConn ~ 8600 5400
NoConn ~ 8600 5300
NoConn ~ 8600 5200
NoConn ~ 8600 5100
NoConn ~ 8600 5000
NoConn ~ 8600 4900
NoConn ~ 8600 4800
NoConn ~ 8600 4700
NoConn ~ 8600 4600
NoConn ~ 8600 4500
NoConn ~ 8600 4400
NoConn ~ 8600 4300
NoConn ~ 8600 4200
NoConn ~ 8600 4100
NoConn ~ 8600 3600
NoConn ~ 8600 3500
NoConn ~ 8600 3400
NoConn ~ 8600 3300
NoConn ~ 8600 3000
NoConn ~ 8600 2900
NoConn ~ 8600 2800
NoConn ~ 8600 2700
NoConn ~ 8600 2600
NoConn ~ 8600 2500
NoConn ~ 8600 2400
NoConn ~ 8600 2300
NoConn ~ 8600 2200
NoConn ~ 8600 2100
NoConn ~ 4500 2600
Wire Wire Line
	6200 2850 5450 2850
Wire Wire Line
	5450 2850 5450 4050
Wire Wire Line
	4800 4050 5450 4050
$Comp
L Switch:SW_SPST SW1
U 1 1 61DCEBA3
P 3150 2300
F 0 "SW1" H 3150 2535 50  0000 C CNN
F 1 "SW_SPST" H 3150 2444 50  0000 C CNN
F 2 "" H 3150 2300 50  0001 C CNN
F 3 "~" H 3150 2300 50  0001 C CNN
	1    3150 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 2300 2950 2300
Wire Wire Line
	3350 2300 3600 2300
Wire Wire Line
	2450 2900 3600 2900
$EndSCHEMATC