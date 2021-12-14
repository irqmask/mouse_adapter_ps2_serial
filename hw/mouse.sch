EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PS/2 to Serial Mouse Adapter"
Date "2021-05-20"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L mouse-rescue:Mini-DIN-6-connectors-mouse-rescue J7
U 1 1 60A5670F
P 8900 3350
F 0 "J7" H 8900 3717 50  0000 C CNN
F 1 "Mini-DIN-6" H 8900 3626 50  0000 C CNN
F 2 "Connector_Miscelleaneus:MiniDIN_6pole" H 8900 3350 50  0001 C CNN
F 3 "http://service.powerdynamics.com/ec/Catalog17/Section%2011.pdf" H 8900 3350 50  0001 C CNN
	1    8900 3350
	1    0    0    -1  
$EndComp
$Comp
L standard:CRYSTAL U1
U 1 1 60A5814C
P 1950 4750
F 0 "U1" H 2150 5000 50  0000 C CNN
F 1 "12MHz" H 2150 4900 50  0000 C CNN
F 2 "Crystal:CRY_HC49" H 1950 4750 60  0001 C CNN
F 3 "" H 1950 4750 60  0000 C CNN
	1    1950 4750
	1    0    0    -1  
$EndComp
$Comp
L connectors:CONN_5X1 J5
U 1 1 60A58680
P 8800 2150
F 0 "J5" H 8978 2176 50  0000 L CNN
F 1 "CONN_5X1" H 8978 2104 25  0000 L CNN
F 2 "Connector_Miscelleaneus:USB_A" H 8800 2150 60  0001 C CNN
F 3 "" H 8800 2150 60  0000 C CNN
	1    8800 2150
	1    0    0    -1  
$EndComp
$Comp
L connectors:CONN_2X1 J4
U 1 1 60A58FFF
P 8800 1350
F 0 "J4" H 8978 1376 50  0000 L CNN
F 1 "CONN_2X1" H 8978 1304 25  0000 L CNN
F 2 "Connector_Header:PCB_CON2" H 8800 1350 60  0001 C CNN
F 3 "" H 8800 1350 60  0000 C CNN
	1    8800 1350
	1    0    0    1   
$EndComp
$Comp
L connectors:CONN-ISP-6 J3
U 1 1 60A593BF
P 5000 5050
F 0 "J3" H 5000 5375 50  0000 C CNN
F 1 "CONN-ISP-6" H 5000 5303 25  0000 C CNN
F 2 "Connector_Header:HEADER_3x2" H 4850 4800 25  0001 L CNN
F 3 "" H 5000 5050 60  0000 C CNN
	1    5000 5050
	1    0    0    -1  
$EndComp
$Comp
L connectors:JUMPER2 J1
U 1 1 60A59B08
P 4000 4550
F 0 "J1" H 4350 4550 50  0000 C CNN
F 1 "JUMPER2" H 4600 4550 25  0000 C CNN
F 2 "Connector_Header:HEADER_2x1" H 4000 4550 60  0001 C CNN
F 3 "" H 4000 4550 60  0000 C CNN
	1    4000 4550
	1    0    0    -1  
$EndComp
$Comp
L connectors:JUMPER2 J2
U 1 1 60A5A61B
P 4000 4650
F 0 "J2" H 4350 4650 50  0000 C CNN
F 1 "JUMPER2" H 4600 4650 25  0000 C CNN
F 2 "Connector_Header:HEADER_2x1" H 4000 4650 60  0001 C CNN
F 3 "" H 4000 4650 60  0000 C CNN
	1    4000 4650
	1    0    0    -1  
$EndComp
$Comp
L connectors:CONN_5X2 J9
U 1 1 60A5AEB4
P 8900 5200
F 0 "J9" H 8900 5650 50  0000 C CNN
F 1 "CONN_5X2" H 8900 5550 25  0000 C CNN
F 2 "Connector_Header:HEADER_5x2" H 8900 5200 60  0001 C CNN
F 3 "" H 8900 5200 60  0000 C CNN
	1    8900 5200
	1    0    0    -1  
$EndComp
$Comp
L mouse-rescue:ATtiny2313-20PU-ics_controller_atmel-mouse-rescue U2
U 1 1 60A609EE
P 3050 5250
F 0 "U2" H 2600 6300 50  0000 C CNN
F 1 "ATtiny2313-20PU" H 3050 5250 50  0000 C CNN
F 2 "Housings_DIP:DIP20" H 3050 5250 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2543-AVR-ATtiny2313_Datasheet.pdf" H 3050 5250 50  0001 C CNN
	1    3050 5250
	1    0    0    -1  
$EndComp
$Comp
L standard:C C2
U 1 1 60A69D25
P 1700 4650
F 0 "C2" H 1700 4800 50  0000 C CNN
F 1 "18pF" H 1850 4700 50  0000 C CNN
F 2 "Capacitors_THT:C_CER_2.54mm" H 1700 4650 60  0001 C CNN
F 3 "" H 1700 4650 60  0000 C CNN
	1    1700 4650
	1    0    0    -1  
$EndComp
$Comp
L standard:C C3
U 1 1 60A6A570
P 1700 4850
F 0 "C3" H 1700 4700 50  0000 C CNN
F 1 "18pF" H 1850 4800 50  0000 C CNN
F 2 "Capacitors_THT:C_CER_2.54mm" H 1700 4850 60  0001 C CNN
F 3 "" H 1700 4850 60  0000 C CNN
	1    1700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4650 2300 4650
Wire Wire Line
	1800 4850 1950 4850
Wire Wire Line
	2300 4750 2300 4650
Connection ~ 2300 4650
Wire Wire Line
	2300 4650 2450 4650
Wire Wire Line
	1950 4750 1950 4850
Connection ~ 1950 4850
Wire Wire Line
	1950 4850 2450 4850
Wire Wire Line
	1600 4650 1550 4650
Wire Wire Line
	1550 4650 1550 4850
Wire Wire Line
	1550 4850 1600 4850
Wire Wire Line
	1550 4850 1550 4950
Connection ~ 1550 4850
$Comp
L power:GND #PWR03
U 1 1 60A6DBFD
P 1550 4950
F 0 "#PWR03" H 1550 4750 50  0001 C CNN
F 1 "GND" H 1550 4840 50  0001 C CNN
F 2 "" H 1550 4950 50  0001 C CNN
F 3 "" H 1550 4950 50  0001 C CNN
	1    1550 4950
	1    0    0    -1  
$EndComp
$Comp
L standard:R R1
U 1 1 60A72045
P 1550 3700
F 0 "R1" V 1350 3800 50  0000 L CNN
F 1 "15k" H 1550 3700 50  0000 C CNN
F 2 "Resistor:R_RM10.16mm" H 1550 3700 60  0001 C CNN
F 3 "" H 1550 3700 60  0000 C CNN
	1    1550 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	2450 4450 1550 4450
$Comp
L power:+5V #PWR02
U 1 1 60A756D3
P 1550 3450
F 0 "#PWR02" H 1550 3650 50  0001 C CNN
F 1 "+5V" H 1546 3593 50  0000 C CNN
F 2 "" H 1550 3450 50  0001 C CNN
F 3 "" H 1550 3450 50  0001 C CNN
	1    1550 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 60A75DD3
P 3050 3450
F 0 "#PWR04" H 3050 3650 50  0001 C CNN
F 1 "+5V" H 3046 3593 50  0000 C CNN
F 2 "" H 3050 3450 50  0001 C CNN
F 3 "" H 3050 3450 50  0001 C CNN
	1    3050 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6350 3050 6450
$Comp
L power:GND #PWR05
U 1 1 60A91D2E
P 3050 6450
F 0 "#PWR05" H 3050 6250 50  0001 C CNN
F 1 "GND" H 3050 6340 50  0001 C CNN
F 2 "" H 3050 6450 50  0001 C CNN
F 3 "" H 3050 6450 50  0001 C CNN
	1    3050 6450
	1    0    0    -1  
$EndComp
Text Label 3700 5350 0    50   ~ 0
RXD
Text Label 3700 5450 0    50   ~ 0
TXD
Wire Wire Line
	3650 5350 3850 5350
Wire Wire Line
	3650 5450 3850 5450
$Comp
L standard:LED D1
U 1 1 60A9939B
P 3850 4000
F 0 "D1" V 4004 4262 50  0000 L CNN
F 1 "LED 3mm green, 2mA, 1,9V" V 4095 4262 50  0000 L CNN
F 2 "LED:LED_RM2.54mm_D3mm" H 3850 4000 60  0001 C CNN
F 3 "" H 3850 4000 60  0000 C CNN
	1    3850 4000
	0    1    1    0   
$EndComp
$Comp
L standard:R R2
U 1 1 60A9B0C4
P 4000 3700
F 0 "R2" V 3954 3778 50  0000 L CNN
F 1 "1k5" H 4000 3700 50  0000 C CNN
F 2 "Resistor:R_RM10.16mm" H 4000 3700 60  0001 C CNN
F 3 "" H 4000 3700 60  0000 C CNN
	1    4000 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 4450 4000 4450
Wire Wire Line
	4000 4450 4000 4400
Wire Wire Line
	4000 3900 4000 4000
Wire Wire Line
	4000 3500 4000 3450
$Comp
L power:+5V #PWR07
U 1 1 60AA2D6A
P 4000 3450
F 0 "#PWR07" H 4000 3650 50  0001 C CNN
F 1 "+5V" H 3996 3593 50  0000 C CNN
F 2 "" H 4000 3450 50  0001 C CNN
F 3 "" H 4000 3450 50  0001 C CNN
	1    4000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 3450 3050 3500
Entry Wire Line
	3900 4950 4000 5050
Text Label 3700 4950 0    50   ~ 0
MOSI
Text Label 3700 5050 0    50   ~ 0
MISO
Text Label 3700 5150 0    50   ~ 0
SCL
Wire Wire Line
	3650 4950 3900 4950
Wire Wire Line
	3650 5050 3900 5050
Wire Wire Line
	3650 5150 3900 5150
Entry Wire Line
	3900 5050 4000 5150
Entry Wire Line
	3900 5150 4000 5250
Entry Wire Line
	3850 5350 3950 5450
Entry Wire Line
	3850 5450 3950 5550
Text Label 2200 4450 0    50   ~ 0
RESET
Text Label 5350 5050 0    50   ~ 0
MOSI
Text Label 4450 4950 0    50   ~ 0
MISO
Text Label 4450 5050 0    50   ~ 0
SCL
Text Label 4450 5150 0    50   ~ 0
RESET
Wire Wire Line
	4700 4950 4400 4950
Wire Wire Line
	4700 5050 4400 5050
Wire Wire Line
	4700 5150 4400 5150
Wire Wire Line
	5300 5050 5600 5050
Wire Wire Line
	5300 4950 5600 4950
Wire Wire Line
	5600 4950 5600 4850
Wire Wire Line
	5300 5150 5600 5150
Wire Wire Line
	5600 5150 5600 5250
$Comp
L power:+5V #PWR09
U 1 1 60AC67D8
P 5600 4850
F 0 "#PWR09" H 5600 5050 50  0001 C CNN
F 1 "+5V" H 5596 4993 50  0000 C CNN
F 2 "" H 5600 4850 50  0001 C CNN
F 3 "" H 5600 4850 50  0001 C CNN
	1    5600 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60AC71DC
P 5600 5250
F 0 "#PWR010" H 5600 5050 50  0001 C CNN
F 1 "GND" H 5600 5140 50  0001 C CNN
F 2 "" H 5600 5250 50  0001 C CNN
F 3 "" H 5600 5250 50  0001 C CNN
	1    5600 5250
	1    0    0    -1  
$EndComp
Entry Wire Line
	5600 5050 5700 5150
Entry Wire Line
	4300 5050 4400 4950
Entry Wire Line
	4300 5150 4400 5050
Entry Wire Line
	4300 5250 4400 5150
Wire Bus Line
	5700 5150 5700 5450
Wire Bus Line
	5700 5450 4300 5450
$Comp
L standard:C C4
U 1 1 60AD2914
P 3250 3700
F 0 "C4" V 3204 3803 50  0000 L CNN
F 1 "100nF" V 3295 3803 50  0000 L CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x2.5mm" H 3250 3700 60  0001 C CNN
F 3 "" H 3250 3700 60  0000 C CNN
	1    3250 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 3600 3250 3500
Wire Wire Line
	3250 3500 3050 3500
Connection ~ 3050 3500
Wire Wire Line
	3050 3500 3050 4150
Wire Wire Line
	3250 3800 3250 3900
$Comp
L power:GND #PWR06
U 1 1 60AD6A26
P 3250 3900
F 0 "#PWR06" H 3250 3700 50  0001 C CNN
F 1 "GND" H 3250 3790 50  0001 C CNN
F 2 "" H 3250 3900 50  0001 C CNN
F 3 "" H 3250 3900 50  0001 C CNN
	1    3250 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3450 1550 3500
Wire Wire Line
	1550 3900 1550 4450
$Comp
L standard:C C1
U 1 1 60AFD241
P 1400 4450
F 0 "C1" H 1400 4690 50  0000 C CNN
F 1 "4,7nF" H 1400 4599 50  0000 C CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x2.5mm" H 1400 4450 60  0001 C CNN
F 3 "" H 1400 4450 60  0000 C CNN
	1    1400 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 4450 1250 4450
Wire Wire Line
	1250 4450 1250 4950
Wire Wire Line
	1500 4450 1550 4450
Connection ~ 1550 4450
$Comp
L power:GND #PWR01
U 1 1 60B0117E
P 1250 4950
F 0 "#PWR01" H 1250 4750 50  0001 C CNN
F 1 "GND" H 1250 4840 50  0001 C CNN
F 2 "" H 1250 4950 50  0001 C CNN
F 3 "" H 1250 4950 50  0001 C CNN
	1    1250 4950
	1    0    0    -1  
$EndComp
$Comp
L ics_misc:MAX232N U3
U 1 1 60A6431C
P 7450 5050
F 0 "U3" H 7450 5750 50  0000 C CNN
F 1 "MAX232N" H 7450 5650 50  0000 C CNN
F 2 "Housings_DIP:DIP16" H 7450 4550 25  0001 C CNN
F 3 "" H 7450 5050 60  0000 C CNN
	1    7450 5050
	1    0    0    -1  
$EndComp
$Comp
L standard:C C7
U 1 1 60A67EDC
P 6650 4700
F 0 "C7" H 6450 4650 50  0000 L CNN
F 1 "1,0µF/50V (MKS2)" H 6150 4200 50  0000 L CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x5mm" H 6650 4700 60  0001 C CNN
F 3 "" H 6650 4700 60  0000 C CNN
	1    6650 4700
	1    0    0    1   
$EndComp
$Comp
L standard:C C8
U 1 1 60A6992C
P 6650 5000
F 0 "C8" H 6800 4950 50  0000 C CNN
F 1 "1,0µF/50V (MKS2)" H 6800 4400 50  0000 C CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x5mm" H 6650 5000 60  0001 C CNN
F 3 "" H 6650 5000 60  0000 C CNN
	1    6650 5000
	-1   0    0    1   
$EndComp
$Comp
L standard:C C5
U 1 1 60A6A2B9
P 6300 4800
F 0 "C5" H 6200 4900 50  0000 C CNN
F 1 "1,0µF/50V (MKS2)" H 6500 5300 50  0000 C CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x5mm" H 6300 4800 60  0001 C CNN
F 3 "" H 6300 4800 60  0000 C CNN
	1    6300 4800
	1    0    0    -1  
$EndComp
$Comp
L standard:C C6
U 1 1 60A6A9EF
P 6300 5200
F 0 "C6" H 6200 5300 50  0000 C CNN
F 1 "1,0µF/50V (MKS2)" H 6500 5900 50  0000 C CNN
F 2 "Capacitors_THT:C_RM5.08mm_5x5mm" H 6300 5200 60  0001 C CNN
F 3 "" H 6300 5200 60  0000 C CNN
	1    6300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4800 6900 4800
Wire Wire Line
	6900 4900 6500 4900
Wire Wire Line
	6500 4900 6500 4700
Wire Wire Line
	6500 4700 6550 4700
Wire Wire Line
	6900 5100 6500 5100
Wire Wire Line
	6500 5100 6500 5000
Wire Wire Line
	6500 5000 6550 5000
Wire Wire Line
	6400 5200 6900 5200
Wire Wire Line
	6750 5000 6900 5000
Wire Wire Line
	6750 4700 6900 4700
Wire Wire Line
	6200 4800 6150 4800
Wire Wire Line
	6150 4800 6150 5200
Wire Wire Line
	6150 5200 6200 5200
Wire Wire Line
	6150 5200 6150 5300
Connection ~ 6150 5200
$Comp
L power:GND #PWR011
U 1 1 60A92B4B
P 6150 5300
F 0 "#PWR011" H 6150 5100 50  0001 C CNN
F 1 "GND" H 6150 5190 50  0001 C CNN
F 2 "" H 6150 5300 50  0001 C CNN
F 3 "" H 6150 5300 50  0001 C CNN
	1    6150 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4700 8100 4700
Wire Wire Line
	8100 4700 8100 4200
$Comp
L power:GND #PWR013
U 1 1 60A9577E
P 8100 4800
F 0 "#PWR013" H 8100 4600 50  0001 C CNN
F 1 "GND" H 8100 4690 50  0001 C CNN
F 2 "" H 8100 4800 50  0001 C CNN
F 3 "" H 8100 4800 50  0001 C CNN
	1    8100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4800 8100 4800
Wire Wire Line
	9100 5000 9200 5000
Wire Wire Line
	9200 5000 9200 4900
Wire Wire Line
	8700 5100 8550 5100
Wire Wire Line
	8550 5100 8550 5000
Wire Wire Line
	8700 5200 8550 5200
Wire Wire Line
	8550 5200 8550 5650
$Comp
L power:GND #PWR020
U 1 1 60AB7687
P 8550 6250
F 0 "#PWR020" H 8550 6050 50  0001 C CNN
F 1 "GND" H 8550 6140 50  0001 C CNN
F 2 "" H 8550 6250 50  0001 C CNN
F 3 "" H 8550 6250 50  0001 C CNN
	1    8550 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 5300 8450 5300
Wire Wire Line
	8450 5300 8450 5650
Wire Wire Line
	6800 5650 6800 5400
Wire Wire Line
	6800 5400 6900 5400
Text Label 8050 5100 0    50   ~ 0
RXD
Text Label 8050 5200 0    50   ~ 0
TXD
Text Label 8050 5400 0    50   ~ 0
RTS
Wire Wire Line
	8000 5100 8200 5100
Wire Wire Line
	8000 5200 8200 5200
Wire Wire Line
	8000 5400 8200 5400
NoConn ~ 8000 5300
NoConn ~ 6900 5300
NoConn ~ 8700 5000
NoConn ~ 8700 5400
NoConn ~ 9100 5100
NoConn ~ 9100 5200
NoConn ~ 9100 5300
NoConn ~ 9100 5400
Entry Wire Line
	8200 5100 8300 5200
Entry Wire Line
	8200 5200 8300 5300
Entry Wire Line
	8200 5400 8300 5500
$Comp
L connectors:CONN_3X1 J6
U 1 1 60AFC066
P 8800 6050
F 0 "J6" H 8900 5700 50  0000 C CNN
F 1 "CONN_3X1" H 8900 5800 25  0000 C CNN
F 2 "Connector_Header:HEADER_3x1" H 8800 6050 60  0001 C CNN
F 3 "" H 8800 6050 60  0000 C CNN
	1    8800 6050
	1    0    0    1   
$EndComp
Text Label 8400 6050 0    50   ~ 0
RXD
Text Label 8400 5950 0    50   ~ 0
TXD
Wire Wire Line
	8600 6150 8550 6150
Wire Wire Line
	8550 6150 8550 6250
Wire Wire Line
	8600 5950 8400 5950
Wire Wire Line
	8600 6050 8400 6050
Entry Wire Line
	8300 6050 8400 5950
Entry Wire Line
	8300 6150 8400 6050
Wire Wire Line
	8000 4900 9200 4900
Wire Wire Line
	8550 5000 8000 5000
Wire Wire Line
	8450 5650 6800 5650
$Comp
L power:GND #PWR019
U 1 1 60B66E32
P 8550 5650
F 0 "#PWR019" H 8550 5450 50  0001 C CNN
F 1 "GND" H 8550 5540 50  0001 C CNN
F 2 "" H 8550 5650 50  0001 C CNN
F 3 "" H 8550 5650 50  0001 C CNN
	1    8550 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 60B7E233
P 8550 1500
F 0 "#PWR017" H 8550 1300 50  0001 C CNN
F 1 "GND" H 8550 1390 50  0001 C CNN
F 2 "" H 8550 1500 50  0001 C CNN
F 3 "" H 8550 1500 50  0001 C CNN
	1    8550 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1400 8550 1400
Wire Wire Line
	8550 1400 8550 1500
$Comp
L power:+5V #PWR016
U 1 1 60B92028
P 8550 900
F 0 "#PWR016" H 8550 1100 50  0001 C CNN
F 1 "+5V" H 8546 1043 50  0000 C CNN
F 2 "" H 8550 900 50  0001 C CNN
F 3 "" H 8550 900 50  0001 C CNN
	1    8550 900 
	1    0    0    -1  
$EndComp
$Comp
L standard:CP C9
U 1 1 60B94C3F
P 8400 1150
F 0 "C9" V 8300 1050 50  0000 R CNN
F 1 "100µF" V 8400 1050 50  0000 R CNN
F 2 "Capacitors_THT:CP_RM2.54mm_D6.5mm" H 8400 1150 60  0001 C CNN
F 3 "" H 8400 1150 60  0000 C CNN
F 4 "35V" V 8500 1050 50  0000 R CNN "Voltage"
F 5 "+-10%" V 8600 1050 50  0000 R CNN "Tolrance"
	1    8400 1150
	0    1    1    0   
$EndComp
Wire Wire Line
	8600 1300 8550 1300
Wire Wire Line
	8550 1300 8550 950 
Wire Wire Line
	8400 1000 8400 950 
Wire Wire Line
	8400 950  8550 950 
Connection ~ 8550 950 
Wire Wire Line
	8550 950  8550 900 
Wire Wire Line
	8400 1300 8400 1400
Wire Wire Line
	8400 1400 8550 1400
Connection ~ 8550 1400
Wire Bus Line
	4300 5450 4000 5450
Connection ~ 4300 5450
Wire Bus Line
	3950 6250 5800 6250
$Comp
L power:+5V #PWR015
U 1 1 60C0019A
P 8450 3250
F 0 "#PWR015" H 8450 3450 50  0001 C CNN
F 1 "+5V" H 8446 3393 50  0000 C CNN
F 2 "" H 8450 3250 50  0001 C CNN
F 3 "" H 8450 3250 50  0001 C CNN
	1    8450 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 60C008D2
P 8100 4200
F 0 "#PWR012" H 8100 4400 50  0001 C CNN
F 1 "+5V" H 8096 4343 50  0000 C CNN
F 2 "" H 8100 4200 50  0001 C CNN
F 3 "" H 8100 4200 50  0001 C CNN
	1    8100 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 60C0327F
P 9350 3300
F 0 "#PWR023" H 9350 3100 50  0001 C CNN
F 1 "GND" H 9350 3190 50  0001 C CNN
F 2 "" H 9350 3300 50  0001 C CNN
F 3 "" H 9350 3300 50  0001 C CNN
	1    9350 3300
	1    0    0    -1  
$EndComp
$Comp
L connectors:CONN_4X2 J8
U 1 1 60C10996
P 8900 4100
F 0 "J8" H 8900 4375 50  0000 C CNN
F 1 "CONN_4X2" H 8900 4303 25  0000 C CNN
F 2 "Connector_Header:HEADER_4x2" H 8900 4100 60  0001 C CNN
F 3 "" H 8900 4100 60  0000 C CNN
	1    8900 4100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4150 4550 4250 4550
Wire Wire Line
	4250 4550 4250 4650
Wire Wire Line
	3650 4550 3850 4550
Wire Wire Line
	3650 4650 3850 4650
Wire Wire Line
	4150 4650 4250 4650
Connection ~ 4250 4650
Wire Wire Line
	4250 4650 4250 4750
$Comp
L power:GND #PWR08
U 1 1 60C6BC45
P 4250 4750
F 0 "#PWR08" H 4250 4550 50  0001 C CNN
F 1 "GND" H 4250 4640 50  0001 C CNN
F 2 "" H 4250 4750 50  0001 C CNN
F 3 "" H 4250 4750 50  0001 C CNN
	1    4250 4750
	1    0    0    -1  
$EndComp
Text Label 3700 5550 0    50   ~ 0
MCLK
Text Label 3700 5650 0    50   ~ 0
MDTA
Entry Wire Line
	3850 5550 3950 5650
Entry Wire Line
	3850 5650 3950 5750
Wire Wire Line
	3650 5550 3850 5550
Wire Wire Line
	3650 5650 3850 5650
Entry Wire Line
	3850 5750 3950 5850
Wire Wire Line
	3650 5750 3850 5750
Text Label 3700 5750 0    50   ~ 0
RTS
Wire Bus Line
	5800 6250 5800 3650
Text Label 9250 3450 0    50   ~ 0
MDTA
Text Label 9250 3250 0    50   ~ 0
MCLK
Entry Wire Line
	9450 3250 9550 3350
Entry Wire Line
	9450 3450 9550 3550
Wire Wire Line
	9200 3250 9450 3250
Wire Wire Line
	9200 3450 9450 3450
Wire Wire Line
	9200 3350 9250 3350
Wire Wire Line
	9250 3350 9250 3300
Wire Wire Line
	9250 3300 9350 3300
Wire Wire Line
	8450 3250 8450 3350
Wire Wire Line
	8450 3350 8600 3350
Text Label 8450 4200 0    50   ~ 0
MDTA
Text Label 9250 4300 0    50   ~ 0
MCLK
Entry Wire Line
	9450 4300 9550 4200
Entry Wire Line
	8350 4100 8450 4200
$Comp
L power:+5V #PWR021
U 1 1 60AA9634
P 8600 3900
F 0 "#PWR021" H 8600 4100 50  0001 C CNN
F 1 "+5V" H 8596 4043 50  0000 C CNN
F 2 "" H 8600 3900 50  0001 C CNN
F 3 "" H 8600 3900 50  0001 C CNN
	1    8600 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 60AA9F35
P 8600 4400
F 0 "#PWR022" H 8600 4200 50  0001 C CNN
F 1 "GND" H 8600 4290 50  0001 C CNN
F 2 "" H 8600 4400 50  0001 C CNN
F 3 "" H 8600 4400 50  0001 C CNN
	1    8600 4400
	1    0    0    -1  
$EndComp
NoConn ~ 9100 4100
NoConn ~ 8700 4100
NoConn ~ 8600 3450
NoConn ~ 8600 3250
Connection ~ 9550 3650
Text Notes 9450 6050 0    50   ~ 0
Serial Debug Terminal\nTTL level only!
Text Notes 9450 5100 0    50   ~ 0
Mouse output\nSerial, V24
Text Notes 9750 3300 0    50   ~ 0
PS/2 Mouse input
Text Notes 9750 3950 0    50   ~ 0
PS/2 Mouse Input\nto connect to slot brackets
Text Notes 9750 1350 0    50   ~ 0
5V DC Input
Wire Wire Line
	9450 4300 9100 4300
Wire Wire Line
	8700 4200 8450 4200
Wire Wire Line
	8700 4000 8600 4000
Wire Wire Line
	8600 4000 8600 3900
Wire Wire Line
	8700 4300 8600 4300
Wire Wire Line
	8600 4300 8600 4400
Wire Bus Line
	8350 4100 8350 3650
Connection ~ 8350 3650
Wire Bus Line
	8350 3650 9550 3650
Wire Bus Line
	9550 3650 9550 4200
NoConn ~ 9100 4200
NoConn ~ 9100 4000
Wire Wire Line
	8600 1950 8550 1950
Wire Wire Line
	8550 1850 8550 1950
Wire Wire Line
	8600 2250 8550 2250
Wire Wire Line
	8550 2250 8550 2350
Wire Wire Line
	8550 2350 8600 2350
$Comp
L power:+5V #PWR018
U 1 1 60BD329C
P 8550 1850
F 0 "#PWR018" H 8550 2050 50  0001 C CNN
F 1 "+5V" H 8546 1993 50  0000 C CNN
F 2 "" H 8550 1850 50  0001 C CNN
F 3 "" H 8550 1850 50  0001 C CNN
	1    8550 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 60BD4B56
P 8450 2750
F 0 "#PWR014" H 8450 2550 50  0001 C CNN
F 1 "GND" H 8450 2640 50  0001 C CNN
F 2 "" H 8450 2750 50  0001 C CNN
F 3 "" H 8450 2750 50  0001 C CNN
	1    8450 2750
	1    0    0    -1  
$EndComp
Connection ~ 8550 2350
$Comp
L standard:R R4
U 1 1 60BDEFAD
P 8350 2450
F 0 "R4" V 8350 2500 50  0000 L CNN
F 1 "15k" H 8350 2450 50  0000 C CNN
F 2 "Resistor:R_RM10.16mm" H 8350 2450 60  0001 C CNN
F 3 "" H 8350 2450 60  0000 C CNN
	1    8350 2450
	0    1    1    0   
$EndComp
$Comp
L standard:R R3
U 1 1 60BDF9DC
P 8200 2450
F 0 "R3" V 8200 2300 50  0000 L CNN
F 1 "15k" H 8200 2450 50  0000 C CNN
F 2 "Resistor:R_RM10.16mm" H 8200 2450 60  0001 C CNN
F 3 "" H 8200 2450 60  0000 C CNN
	1    8200 2450
	0    1    1    0   
$EndComp
Entry Wire Line
	7800 2150 7900 2050
Wire Wire Line
	8600 2150 8350 2150
Wire Wire Line
	8350 2150 8350 2250
Wire Wire Line
	8600 2050 8200 2050
Wire Wire Line
	8200 2050 8200 2250
Wire Wire Line
	8200 2650 8200 2700
Wire Wire Line
	8200 2700 8350 2700
Wire Wire Line
	8550 2350 8550 2700
Wire Wire Line
	8350 2650 8350 2700
Connection ~ 8350 2700
Wire Wire Line
	8350 2700 8450 2700
Wire Wire Line
	8450 2750 8450 2700
Connection ~ 8450 2700
Wire Wire Line
	8450 2700 8550 2700
Entry Wire Line
	7800 2250 7900 2150
Entry Wire Line
	7800 2250 7900 2150
Connection ~ 8200 2050
Connection ~ 8350 2150
Text Label 7950 2050 0    50   ~ 0
MCLK
Text Label 7950 2150 0    50   ~ 0
MDTA
Wire Bus Line
	7800 3650 8350 3650
Wire Wire Line
	7900 2050 8200 2050
Wire Wire Line
	7900 2150 8350 2150
Text Notes 9200 2350 0    50   ~ 0
+5V\nD+\nD-\nGND\nShield
Text Notes 9750 2300 0    50   ~ 0
USB A connector\n(depending on firmware\n as USB host or PS/2 host)
$Comp
L logo:OPEN_HARDWARE_1 LOGO1
U 1 1 60CE72DB
P 1450 7450
F 0 "LOGO1" H 1697 7528 60  0000 L CNN
F 1 "OPEN_HARDWARE_1" H 1697 7422 60  0000 L CNN
F 2 "Symbol:Symbol_OSHW-Logo_Copper" H 1450 7450 50  0001 C CNN
F 3 "" H 1450 7450 50  0001 C CNN
	1    1450 7450
	1    0    0    -1  
$EndComp
NoConn ~ 3650 4750
NoConn ~ 3650 4850
NoConn ~ 3650 5850
NoConn ~ 3650 5950
Wire Bus Line
	8300 6250 5800 6250
Connection ~ 5800 6250
Wire Bus Line
	7800 3650 5800 3650
Connection ~ 7800 3650
Wire Bus Line
	9550 3350 9550 3650
Wire Bus Line
	7800 2150 7800 3650
Wire Bus Line
	4000 5050 4000 5450
Wire Bus Line
	4300 5050 4300 5450
Wire Bus Line
	3950 5450 3950 6250
Wire Bus Line
	8300 5200 8300 6250
$EndSCHEMATC
