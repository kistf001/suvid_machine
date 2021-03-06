EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Relay_SolidState:MOC3061M U1
U 1 1 5E9C2196
P 5850 2100
F 0 "U1" H 5850 2425 50  0000 C CNN
F 1 "MOC3061M" H 5850 2334 50  0000 C CNN
F 2 "Package_DIP:DIP-6_W7.62mm" H 5650 1900 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/MO/MOC3061M.pdf" H 5850 2100 50  0001 L CNN
	1    5850 2100
	1    0    0    -1  
$EndComp
$Comp
L Relay_SolidState:MOC3061M U2
U 1 1 5E9C341A
P 5850 3100
F 0 "U2" H 5850 3425 50  0000 C CNN
F 1 "MOC3061M" H 5850 3334 50  0000 C CNN
F 2 "Package_DIP:DIP-6_W7.62mm" H 5650 2900 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/MO/MOC3061M.pdf" H 5850 3100 50  0001 L CNN
	1    5850 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5E9CF9F5
P 4850 2000
F 0 "R6" V 4643 2000 50  0000 C CNN
F 1 "470R" V 4734 2000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 4780 2000 50  0001 C CNN
F 3 "~" H 4850 2000 50  0001 C CNN
	1    4850 2000
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5E9D08EB
P 4850 3000
F 0 "R7" V 4643 3000 50  0000 C CNN
F 1 "470R" V 4734 3000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 4780 3000 50  0001 C CNN
F 3 "~" H 4850 3000 50  0001 C CNN
	1    4850 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 2000 5000 2000
Wire Wire Line
	5550 3000 5000 3000
$Comp
L Device:R R9
U 1 1 5E9D1CF8
P 7000 2350
F 0 "R9" V 6793 2350 50  0000 C CNN
F 1 "330R" V 6884 2350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 6930 2350 50  0001 C CNN
F 3 "~" H 7000 2350 50  0001 C CNN
	1    7000 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5E9D3251
P 7000 3350
F 0 "R11" V 6793 3350 50  0000 C CNN
F 1 "330R" V 6884 3350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 6930 3350 50  0001 C CNN
F 3 "~" H 7000 3350 50  0001 C CNN
	1    7000 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 2000 6350 2000
Wire Wire Line
	6350 2000 6350 1850
Wire Wire Line
	6350 2350 6350 2200
Wire Wire Line
	6350 2200 6150 2200
Wire Wire Line
	6150 3000 6350 3000
Wire Wire Line
	6350 3000 6350 2850
Wire Wire Line
	6350 3350 6350 3200
Wire Wire Line
	6350 3200 6150 3200
$Comp
L power:GND #PWR013
U 1 1 5E9D8271
P 5350 2350
F 0 "#PWR013" H 5350 2100 50  0001 C CNN
F 1 "GND" H 5355 2177 50  0000 C CNN
F 2 "" H 5350 2350 50  0001 C CNN
F 3 "" H 5350 2350 50  0001 C CNN
	1    5350 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5E9D8C35
P 5350 3350
F 0 "#PWR014" H 5350 3100 50  0001 C CNN
F 1 "GND" H 5355 3177 50  0000 C CNN
F 2 "" H 5350 3350 50  0001 C CNN
F 3 "" H 5350 3350 50  0001 C CNN
	1    5350 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2200 5350 2200
Wire Wire Line
	5350 2200 5350 2350
Wire Wire Line
	5550 3200 5350 3200
Wire Wire Line
	5350 3200 5350 3350
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5EA057B4
P 2600 2550
F 0 "A1" H 2600 1461 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 2600 1370 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 2600 2550 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 2600 2550 50  0001 C CNN
	1    2600 2550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4600 2350 4600 2000
Wire Wire Line
	4600 3000 4600 2450
$Comp
L power:GND #PWR021
U 1 1 5EA656F0
P 8350 5500
F 0 "#PWR021" H 8350 5250 50  0001 C CNN
F 1 "GND" H 8355 5327 50  0000 C CNN
F 2 "" H 8350 5500 50  0001 C CNN
F 3 "" H 8350 5500 50  0001 C CNN
	1    8350 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR020
U 1 1 5EA68404
P 8350 4700
F 0 "#PWR020" H 8350 4550 50  0001 C CNN
F 1 "+5VA" H 8365 4873 50  0000 C CNN
F 2 "" H 8350 4700 50  0001 C CNN
F 3 "" H 8350 4700 50  0001 C CNN
	1    8350 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 4700 8350 4950
$Comp
L Device:R R8
U 1 1 5EB0A62A
P 7000 1850
F 0 "R8" V 6793 1850 50  0000 C CNN
F 1 "330R" V 6884 1850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 6930 1850 50  0001 C CNN
F 3 "~" H 7000 1850 50  0001 C CNN
	1    7000 1850
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 5EB29EA4
P 7000 2850
F 0 "R10" V 6793 2850 50  0000 C CNN
F 1 "330R" V 6884 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 6930 2850 50  0001 C CNN
F 3 "~" H 7000 2850 50  0001 C CNN
	1    7000 2850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5EB5BCF8
P 5350 5750
F 0 "#PWR017" H 5350 5500 50  0001 C CNN
F 1 "GND" H 5355 5577 50  0000 C CNN
F 2 "" H 5350 5750 50  0001 C CNN
F 3 "" H 5350 5750 50  0001 C CNN
	1    5350 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5EB5F896
P 5350 5000
F 0 "#PWR016" H 5350 4750 50  0001 C CNN
F 1 "GND" H 5355 4827 50  0000 C CNN
F 2 "" H 5350 5000 50  0001 C CNN
F 3 "" H 5350 5000 50  0001 C CNN
	1    5350 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5EB5F89C
P 5350 4250
F 0 "#PWR015" H 5350 4000 50  0001 C CNN
F 1 "GND" H 5355 4077 50  0000 C CNN
F 2 "" H 5350 4250 50  0001 C CNN
F 3 "" H 5350 4250 50  0001 C CNN
	1    5350 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4100 5350 4250
Wire Wire Line
	5350 4850 5350 5000
Wire Wire Line
	5350 5600 5350 5750
Wire Wire Line
	5000 4050 4150 4050
Wire Wire Line
	4350 2950 4350 3850
Wire Wire Line
	4250 3050 4250 3950
$Comp
L power:+5VA #PWR08
U 1 1 5ED47C24
P 2400 1300
F 0 "#PWR08" H 2400 1150 50  0001 C CNN
F 1 "+5VA" H 2415 1473 50  0000 C CNN
F 2 "" H 2400 1300 50  0001 C CNN
F 3 "" H 2400 1300 50  0001 C CNN
	1    2400 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1300 2400 1350
$Comp
L Mechanical:MountingHole_Pad H25
U 1 1 5EDA71AE
P 8800 1350
F 0 "H25" V 8754 1500 50  0000 L CNN
F 1 "ac+" V 8845 1500 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 1350 50  0001 C CNN
F 3 "~" H 8800 1350 50  0001 C CNN
	1    8800 1350
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H26
U 1 1 5EDA80BB
P 8800 1600
F 0 "H26" V 8754 1750 50  0000 L CNN
F 1 "ac-" V 8845 1750 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 1600 50  0001 C CNN
F 3 "~" H 8800 1600 50  0001 C CNN
	1    8800 1600
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H27
U 1 1 5EDF5165
P 8800 2050
F 0 "H27" V 8754 2200 50  0000 L CNN
F 1 "heater+" V 8845 2200 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 2050 50  0001 C CNN
F 3 "~" H 8800 2050 50  0001 C CNN
	1    8800 2050
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H28
U 1 1 5EDF516B
P 8800 2350
F 0 "H28" V 8754 2500 50  0000 L CNN
F 1 "heater-" V 8845 2500 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 2350 50  0001 C CNN
F 3 "~" H 8800 2350 50  0001 C CNN
	1    8800 2350
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H29
U 1 1 5EE13A03
P 8800 3050
F 0 "H29" V 8754 3200 50  0000 L CNN
F 1 "motor+" V 8845 3200 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 3050 50  0001 C CNN
F 3 "~" H 8800 3050 50  0001 C CNN
	1    8800 3050
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H30
U 1 1 5EE13A09
P 8800 3350
F 0 "H30" V 8754 3500 50  0000 L CNN
F 1 "motor-" V 8845 3500 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO7380_Pad" H 8800 3350 50  0001 C CNN
F 3 "~" H 8800 3350 50  0001 C CNN
	1    8800 3350
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H31
U 1 1 5EE19454
P 8800 3800
F 0 "H31" V 8754 3950 50  0000 L CNN
F 1 "ac2dc+" V 8845 3950 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 8800 3800 50  0001 C CNN
F 3 "~" H 8800 3800 50  0001 C CNN
	1    8800 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	8300 4100 8700 4100
$Comp
L Mechanical:MountingHole_Pad H32
U 1 1 5EE1945A
P 8800 4100
F 0 "H32" V 8754 4250 50  0000 L CNN
F 1 "ac2dc-" V 8845 4250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 8800 4100 50  0001 C CNN
F 3 "~" H 8800 4100 50  0001 C CNN
	1    8800 4100
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H33
U 1 1 5EE3BF41
P 8800 4950
F 0 "H33" V 8754 5100 50  0000 L CNN
F 1 "dc+" V 8845 5100 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 8800 4950 50  0001 C CNN
F 3 "~" H 8800 4950 50  0001 C CNN
	1    8800 4950
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H34
U 1 1 5EE3BF47
P 8800 5250
F 0 "H34" V 8754 5400 50  0000 L CNN
F 1 "dc-" V 8845 5400 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 8800 5250 50  0001 C CNN
F 3 "~" H 8800 5250 50  0001 C CNN
	1    8800 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	8700 4950 8350 4950
Wire Wire Line
	8700 5250 8550 5250
Wire Wire Line
	8350 5250 8350 5500
$Comp
L Mechanical:MountingHole_Pad H11
U 1 1 5EE58206
P 6300 3850
F 0 "H11" V 6254 4000 50  0000 L CNN
F 1 "button1" V 6345 4000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 3850 50  0001 C CNN
F 3 "~" H 6300 3850 50  0001 C CNN
	1    6300 3850
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H12
U 1 1 5EE5820C
P 6300 4100
F 0 "H12" V 6254 4250 50  0000 L CNN
F 1 "b_gnd" V 6345 4250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 4100 50  0001 C CNN
F 3 "~" H 6300 4100 50  0001 C CNN
	1    6300 4100
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H13
U 1 1 5EE6C66E
P 6300 4600
F 0 "H13" V 6254 4750 50  0000 L CNN
F 1 "button2" V 6345 4750 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 4600 50  0001 C CNN
F 3 "~" H 6300 4600 50  0001 C CNN
	1    6300 4600
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H14
U 1 1 5EE6C674
P 6300 4850
F 0 "H14" V 6254 5000 50  0000 L CNN
F 1 "b_gnd" V 6345 5000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 4850 50  0001 C CNN
F 3 "~" H 6300 4850 50  0001 C CNN
	1    6300 4850
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H15
U 1 1 5EE7143E
P 6300 5350
F 0 "H15" V 6254 5500 50  0000 L CNN
F 1 "button3" V 6345 5500 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 5350 50  0001 C CNN
F 3 "~" H 6300 5350 50  0001 C CNN
	1    6300 5350
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H16
U 1 1 5EE71444
P 6300 5600
F 0 "H16" V 6254 5750 50  0000 L CNN
F 1 "b_gnd" V 6345 5750 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 5600 50  0001 C CNN
F 3 "~" H 6300 5600 50  0001 C CNN
	1    6300 5600
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H17
U 1 1 5EE7AE2C
P 6300 6100
F 0 "H17" V 6254 6250 50  0000 L CNN
F 1 "button4" V 6345 6250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 6100 50  0001 C CNN
F 3 "~" H 6300 6100 50  0001 C CNN
	1    6300 6100
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H18
U 1 1 5EE7AE32
P 6300 6350
F 0 "H18" V 6254 6500 50  0000 L CNN
F 1 "b_gnd" V 6345 6500 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6300 6350 50  0001 C CNN
F 3 "~" H 6300 6350 50  0001 C CNN
	1    6300 6350
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 5600 6200 5600
Wire Wire Line
	5350 6350 5350 6500
$Comp
L power:GND #PWR018
U 1 1 5EB58BB9
P 5350 6500
F 0 "#PWR018" H 5350 6250 50  0001 C CNN
F 1 "GND" H 5355 6327 50  0000 C CNN
F 2 "" H 5350 6500 50  0001 C CNN
F 3 "" H 5350 6500 50  0001 C CNN
	1    5350 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 6350 6200 6350
Wire Wire Line
	5100 3950 5100 4600
Wire Wire Line
	4900 6100 6200 6100
Wire Wire Line
	5000 5350 6200 5350
Wire Wire Line
	5000 4050 5000 5350
Wire Wire Line
	4900 4150 4900 6100
Wire Wire Line
	5100 4600 6200 4600
Wire Wire Line
	5350 4850 6200 4850
Wire Wire Line
	5350 4100 6200 4100
Wire Wire Line
	4150 3150 4150 4050
Wire Wire Line
	4050 3250 4050 4150
Wire Wire Line
	4600 2000 4700 2000
Wire Wire Line
	4600 3000 4700 3000
Wire Wire Line
	3100 2350 4600 2350
Wire Wire Line
	3100 2450 4600 2450
Wire Wire Line
	4050 4150 4900 4150
Wire Wire Line
	4250 3950 5100 3950
Wire Wire Line
	4350 3850 6200 3850
Wire Wire Line
	3100 3250 4050 3250
Wire Wire Line
	3100 3150 4150 3150
Wire Wire Line
	3100 3050 4250 3050
Wire Wire Line
	3100 2950 4350 2950
Wire Wire Line
	3850 4000 3850 5750
Wire Wire Line
	1600 4000 3850 4000
Wire Wire Line
	4150 5750 4150 5900
Connection ~ 4150 5750
Wire Wire Line
	1600 4000 1600 2850
Wire Wire Line
	4150 5750 3850 5750
Wire Wire Line
	3250 5750 3250 5900
Connection ~ 3250 5750
Wire Wire Line
	2950 5750 3250 5750
Wire Wire Line
	2950 4150 2950 5750
Wire Wire Line
	1450 4150 2950 4150
Wire Wire Line
	1450 2750 1450 4150
Wire Wire Line
	2350 5750 2350 5900
Connection ~ 2350 5750
Wire Wire Line
	1300 4300 1300 2650
Wire Wire Line
	2000 4300 1300 4300
Wire Wire Line
	2000 5750 2000 4300
Wire Wire Line
	2350 5750 2000 5750
Wire Wire Line
	1450 5750 1450 5900
Connection ~ 1450 5750
Wire Wire Line
	1150 5750 1450 5750
Wire Wire Line
	1150 2550 1150 5750
Wire Wire Line
	4150 4850 4150 5050
Wire Wire Line
	3250 4850 3250 5050
Wire Wire Line
	2350 4850 2350 5050
Wire Wire Line
	1450 4850 1450 5050
$Comp
L Mechanical:MountingHole_Pad H7
U 1 1 5EF8B1A4
P 3250 5150
F 0 "H7" H 3150 5107 50  0000 R CNN
F 1 "AnalogIN3" H 3150 5198 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3250 5150 50  0001 C CNN
F 3 "~" H 3250 5150 50  0001 C CNN
	1    3250 5150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 5600 3250 5750
Wire Wire Line
	3250 6200 3250 6500
$Comp
L Mechanical:MountingHole_Pad H8
U 1 1 5EF8B19E
P 3250 5500
F 0 "H8" H 3350 5549 50  0000 L CNN
F 1 "AnalogOUT3" H 3350 5458 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3250 5500 50  0001 C CNN
F 3 "~" H 3250 5500 50  0001 C CNN
	1    3250 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR09
U 1 1 5EF8B191
P 3250 4850
F 0 "#PWR09" H 3250 4700 50  0001 C CNN
F 1 "+5VA" H 3265 5023 50  0000 C CNN
F 2 "" H 3250 4850 50  0001 C CNN
F 3 "" H 3250 4850 50  0001 C CNN
	1    3250 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5EF8B18B
P 3250 6050
F 0 "R4" V 3043 6050 50  0000 C CNN
F 1 "10k" V 3134 6050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 3180 6050 50  0001 C CNN
F 3 "~" H 3250 6050 50  0001 C CNN
	1    3250 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 5600 2350 5750
Wire Wire Line
	2350 6200 2350 6500
$Comp
L Mechanical:MountingHole_Pad H5
U 1 1 5EF8B183
P 2350 5150
F 0 "H5" H 2250 5107 50  0000 R CNN
F 1 "AnalogIN2" H 2250 5198 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 2350 5150 50  0001 C CNN
F 3 "~" H 2350 5150 50  0001 C CNN
	1    2350 5150
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H6
U 1 1 5EF8B17D
P 2350 5500
F 0 "H6" H 2450 5549 50  0000 L CNN
F 1 "AnalogOUT2" H 2450 5458 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 2350 5500 50  0001 C CNN
F 3 "~" H 2350 5500 50  0001 C CNN
	1    2350 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR06
U 1 1 5EF8B170
P 2350 4850
F 0 "#PWR06" H 2350 4700 50  0001 C CNN
F 1 "+5VA" H 2365 5023 50  0000 C CNN
F 2 "" H 2350 4850 50  0001 C CNN
F 3 "" H 2350 4850 50  0001 C CNN
	1    2350 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5EF8B16A
P 2350 6050
F 0 "R3" V 2143 6050 50  0000 C CNN
F 1 "10k" V 2234 6050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 2280 6050 50  0001 C CNN
F 3 "~" H 2350 6050 50  0001 C CNN
	1    2350 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	1450 5600 1450 5750
Wire Wire Line
	1450 6200 1450 6500
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5EF854B3
P 1450 5150
F 0 "H3" H 1350 5107 50  0000 R CNN
F 1 "AnalogIN1" H 1350 5198 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1450 5150 50  0001 C CNN
F 3 "~" H 1450 5150 50  0001 C CNN
	1    1450 5150
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5EF854AD
P 1450 5500
F 0 "H4" H 1550 5549 50  0000 L CNN
F 1 "AnalogOUT1" H 1550 5458 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1450 5500 50  0001 C CNN
F 3 "~" H 1450 5500 50  0001 C CNN
	1    1450 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR03
U 1 1 5EF854A0
P 1450 4850
F 0 "#PWR03" H 1450 4700 50  0001 C CNN
F 1 "+5VA" H 1465 5023 50  0000 C CNN
F 2 "" H 1450 4850 50  0001 C CNN
F 3 "" H 1450 4850 50  0001 C CNN
	1    1450 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5EF8549A
P 1450 6050
F 0 "R2" V 1243 6050 50  0000 C CNN
F 1 "10k" V 1334 6050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 1380 6050 50  0001 C CNN
F 3 "~" H 1450 6050 50  0001 C CNN
	1    1450 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 5600 4150 5750
Wire Wire Line
	4150 6200 4150 6500
$Comp
L Mechanical:MountingHole_Pad H9
U 1 1 5EF6F5FD
P 4150 5150
F 0 "H9" H 4050 5107 50  0000 R CNN
F 1 "AnalogIN4" H 4050 5198 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4150 5150 50  0001 C CNN
F 3 "~" H 4150 5150 50  0001 C CNN
	1    4150 5150
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H10
U 1 1 5EF65DF8
P 4150 5500
F 0 "H10" H 4250 5549 50  0000 L CNN
F 1 "AnalogOUT4" H 4250 5458 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4150 5500 50  0001 C CNN
F 3 "~" H 4150 5500 50  0001 C CNN
	1    4150 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR011
U 1 1 5E9FBF4A
P 4150 4850
F 0 "#PWR011" H 4150 4700 50  0001 C CNN
F 1 "+5VA" H 4165 5023 50  0000 C CNN
F 2 "" H 4150 4850 50  0001 C CNN
F 3 "" H 4150 4850 50  0001 C CNN
	1    4150 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E9D7D8B
P 4150 6050
F 0 "R5" V 3943 6050 50  0000 C CNN
F 1 "10k" V 4034 6050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P2.54mm_Vertical" V 4080 6050 50  0001 C CNN
F 3 "~" H 4150 6050 50  0001 C CNN
	1    4150 6050
	-1   0    0    1   
$EndComp
Wire Wire Line
	1150 2550 2100 2550
Wire Wire Line
	1300 2650 2100 2650
Wire Wire Line
	1450 2750 2100 2750
Wire Wire Line
	1600 2850 2100 2850
$Comp
L power:+48V #PWR022
U 1 1 5F126DFA
P 8500 1300
F 0 "#PWR022" H 8500 1150 50  0001 C CNN
F 1 "+48V" H 8515 1473 50  0000 C CNN
F 2 "" H 8500 1300 50  0001 C CNN
F 3 "" H 8500 1300 50  0001 C CNN
	1    8500 1300
	1    0    0    -1  
$EndComp
$Comp
L power:-48V #PWR019
U 1 1 5F126E00
P 8300 1300
F 0 "#PWR019" H 8300 1400 50  0001 C CNN
F 1 "-48V" H 8315 1473 50  0000 C CNN
F 2 "" H 8300 1300 50  0001 C CNN
F 3 "" H 8300 1300 50  0001 C CNN
	1    8300 1300
	1    0    0    -1  
$EndComp
Connection ~ 8500 2050
Wire Wire Line
	8500 2050 8700 2050
Wire Wire Line
	8300 1300 8300 1600
Wire Wire Line
	8500 1300 8500 1350
Wire Wire Line
	8300 1600 8700 1600
Wire Wire Line
	8500 1350 8700 1350
Connection ~ 8300 1600
Connection ~ 8500 1350
Wire Wire Line
	8500 1350 8500 2050
Wire Wire Line
	8500 2050 8500 3050
Wire Wire Line
	8500 3800 8700 3800
Wire Wire Line
	8700 3050 8500 3050
Connection ~ 8500 3050
Wire Wire Line
	8500 3050 8500 3800
$Comp
L Mechanical:MountingHole_Pad H19
U 1 1 5F1D5F12
P 6500 2000
F 0 "H19" V 6454 2150 50  0000 L CNN
F 1 "moc" V 6545 2150 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6500 2000 50  0001 C CNN
F 3 "~" H 6500 2000 50  0001 C CNN
	1    6500 2000
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H20
U 1 1 5F1DAB4A
P 6500 2200
F 0 "H20" V 6454 2350 50  0000 L CNN
F 1 "moc" V 6545 2350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6500 2200 50  0001 C CNN
F 3 "~" H 6500 2200 50  0001 C CNN
	1    6500 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	6400 2200 6350 2200
Connection ~ 6350 2200
Wire Wire Line
	6400 2000 6350 2000
Connection ~ 6350 2000
Wire Wire Line
	8300 1600 8300 1850
$Comp
L Triac_Thyristor:BT138-600 Q2
U 1 1 5E9C0FC6
P 7750 2100
F 0 "Q2" H 7878 2146 50  0000 L CNN
F 1 "BT138-600" H 7878 2055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 7950 2025 50  0001 L CIN
F 3 "https://assets.nexperia.com/documents/data-sheet/BT138_SER_D_E.pdf" H 7750 2100 50  0001 L CNN
	1    7750 2100
	1    0    0    -1  
$EndComp
$Comp
L Triac_Thyristor:BT138-600 Q3
U 1 1 5E9C1636
P 7750 3100
F 0 "Q3" H 7878 3146 50  0000 L CNN
F 1 "BT138-600" H 7878 3055 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 7950 3025 50  0001 L CIN
F 3 "https://assets.nexperia.com/documents/data-sheet/BT138_SER_D_E.pdf" H 7750 3100 50  0001 L CNN
	1    7750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 2350 7750 2250
Wire Wire Line
	7750 1850 7750 1950
Wire Wire Line
	7750 2850 7750 2950
Wire Wire Line
	7750 3250 7750 3350
$Comp
L Mechanical:MountingHole_Pad H23
U 1 1 5F1DF0AB
P 7450 2200
F 0 "H23" V 7687 2203 50  0000 C CNN
F 1 "triac" V 7596 2203 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 7450 2200 50  0001 C CNN
F 3 "~" H 7450 2200 50  0001 C CNN
	1    7450 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7550 2200 7600 2200
Wire Wire Line
	8300 1850 7750 1850
Connection ~ 8300 1850
Wire Wire Line
	8300 1850 8300 2850
Wire Wire Line
	8700 2350 7750 2350
Wire Wire Line
	8300 2850 7750 2850
Connection ~ 8300 2850
Wire Wire Line
	8300 2850 8300 4100
Wire Wire Line
	8700 3350 7750 3350
Wire Wire Line
	7150 1850 7750 1850
Connection ~ 7750 1850
Wire Wire Line
	7750 2350 7150 2350
Connection ~ 7750 2350
Wire Wire Line
	6350 2350 6850 2350
Wire Wire Line
	6350 1850 6850 1850
Wire Wire Line
	6350 2850 6850 2850
Wire Wire Line
	6350 3350 6850 3350
Wire Wire Line
	7150 3350 7750 3350
Connection ~ 7750 3350
Wire Wire Line
	7750 2850 7150 2850
Connection ~ 7750 2850
$Comp
L Mechanical:MountingHole_Pad H24
U 1 1 5F25A6FF
P 7450 3200
F 0 "H24" V 7687 3203 50  0000 C CNN
F 1 "triac" V 7596 3203 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 7450 3200 50  0001 C CNN
F 3 "~" H 7450 3200 50  0001 C CNN
	1    7450 3200
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H21
U 1 1 5F25E858
P 6500 3000
F 0 "H21" V 6454 3150 50  0000 L CNN
F 1 "moc" V 6545 3150 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6500 3000 50  0001 C CNN
F 3 "~" H 6500 3000 50  0001 C CNN
	1    6500 3000
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H22
U 1 1 5F25E85E
P 6500 3200
F 0 "H22" V 6454 3350 50  0000 L CNN
F 1 "moc" V 6545 3350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6500 3200 50  0001 C CNN
F 3 "~" H 6500 3200 50  0001 C CNN
	1    6500 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	6350 3000 6400 3000
Connection ~ 6350 3000
Wire Wire Line
	6400 3200 6350 3200
Connection ~ 6350 3200
Wire Wire Line
	7550 3200 7600 3200
$Comp
L power:+5VA #PWR01
U 1 1 5EA83041
P 800 700
F 0 "#PWR01" H 800 550 50  0001 C CNN
F 1 "+5VA" H 815 873 50  0000 C CNN
F 2 "" H 800 700 50  0001 C CNN
F 3 "" H 800 700 50  0001 C CNN
	1    800  700 
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5EA87F95
P 1350 1400
F 0 "R1" V 1143 1400 50  0000 C CNN
F 1 "470R" V 1234 1400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P5.08mm_Vertical" V 1280 1400 50  0001 C CNN
F 3 "~" H 1350 1400 50  0001 C CNN
	1    1350 1400
	0    -1   1    0   
$EndComp
Wire Wire Line
	1100 1400 1200 1400
Wire Wire Line
	2500 3550 2400 3550
Wire Wire Line
	2350 3550 2350 3600
Wire Wire Line
	2600 3550 2600 3600
Wire Wire Line
	2600 3600 2400 3600
Wire Wire Line
	2400 3600 2400 3550
Connection ~ 2400 3550
Wire Wire Line
	2400 3550 2350 3550
Wire Wire Line
	800  1600 800  1700
Wire Wire Line
	1500 1400 1750 1400
Wire Wire Line
	1750 1400 1750 700 
Wire Wire Line
	1750 700  3500 700 
Wire Wire Line
	3500 700  3500 2550
Wire Wire Line
	3500 2550 3100 2550
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5EAEF0FF
P 250 850
F 0 "H1" V 204 1000 50  0000 L CNN
F 1 "buzz+" V 295 1000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 250 850 50  0001 C CNN
F 3 "~" H 250 850 50  0001 C CNN
	1    250  850 
	0    -1   1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5EAF4769
P 250 1100
F 0 "H2" V 204 1250 50  0000 L CNN
F 1 "buzz-" V 295 1250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 250 1100 50  0001 C CNN
F 3 "~" H 250 1100 50  0001 C CNN
	1    250  1100
	0    -1   1    0   
$EndComp
Wire Wire Line
	350  1100 800  1100
Wire Wire Line
	800  1100 800  1200
Wire Wire Line
	350  850  800  850 
Wire Wire Line
	800  850  800  700 
$Comp
L Transistor_BJT:BC337 Q1
U 1 1 5EB3C0FD
P 900 1400
F 0 "Q1" H 1091 1446 50  0000 L CNN
F 1 "BC337" H 1091 1355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 1091 1309 50  0001 L CIN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bc337.pdf" H 900 1400 50  0001 L CNN
	1    900  1400
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5EBAA366
P 1450 6500
F 0 "#PWR0101" H 1450 6250 50  0001 C CNN
F 1 "GND" H 1455 6327 50  0000 C CNN
F 2 "" H 1450 6500 50  0001 C CNN
F 3 "" H 1450 6500 50  0001 C CNN
	1    1450 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5EBAFC42
P 2350 6500
F 0 "#PWR0102" H 2350 6250 50  0001 C CNN
F 1 "GND" H 2355 6327 50  0000 C CNN
F 2 "" H 2350 6500 50  0001 C CNN
F 3 "" H 2350 6500 50  0001 C CNN
	1    2350 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5EBB5629
P 3250 6500
F 0 "#PWR0103" H 3250 6250 50  0001 C CNN
F 1 "GND" H 3255 6327 50  0000 C CNN
F 2 "" H 3250 6500 50  0001 C CNN
F 3 "" H 3250 6500 50  0001 C CNN
	1    3250 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5EBBAF21
P 4150 6500
F 0 "#PWR0104" H 4150 6250 50  0001 C CNN
F 1 "GND" H 4155 6327 50  0000 C CNN
F 2 "" H 4150 6500 50  0001 C CNN
F 3 "" H 4150 6500 50  0001 C CNN
	1    4150 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5EBC7400
P 2350 3600
F 0 "#PWR0105" H 2350 3350 50  0001 C CNN
F 1 "GND" H 2355 3427 50  0000 C CNN
F 2 "" H 2350 3600 50  0001 C CNN
F 3 "" H 2350 3600 50  0001 C CNN
	1    2350 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5EBD3064
P 800 1750
F 0 "#PWR0106" H 800 1500 50  0001 C CNN
F 1 "GND" H 805 1577 50  0000 C CNN
F 2 "" H 800 1750 50  0001 C CNN
F 3 "" H 800 1750 50  0001 C CNN
	1    800  1750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5EC3F75E
P 4300 1300
F 0 "J1" H 4380 1292 50  0000 L CNN
F 1 "Conn_01x04" H 4380 1201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4300 1300 50  0001 C CNN
F 3 "~" H 4300 1300 50  0001 C CNN
	1    4300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1500 3750 1500
Wire Wire Line
	3750 1500 3750 2850
Wire Wire Line
	3750 2850 3100 2850
Wire Wire Line
	4100 1400 3600 1400
Wire Wire Line
	3600 1400 3600 2750
Wire Wire Line
	3600 2750 3100 2750
Wire Wire Line
	4100 1200 2800 1200
Wire Wire Line
	2800 1200 2800 1350
Wire Wire Line
	2800 1350 2400 1350
Connection ~ 2400 1350
Wire Wire Line
	2400 1350 2400 1550
Wire Wire Line
	4100 1300 2950 1300
Wire Wire Line
	2950 1300 2950 1450
Wire Wire Line
	2950 1450 1550 1450
Wire Wire Line
	1550 1450 1550 1700
Wire Wire Line
	1550 1700 800  1700
Connection ~ 800  1700
Wire Wire Line
	800  1700 800  1750
$Comp
L Mechanical:MountingHole_Pad H35
U 1 1 5EC9ECBE
P 9750 5050
F 0 "H35" V 9704 5200 50  0000 L CNN
F 1 "dc-" V 9795 5200 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad" H 9750 5050 50  0001 C CNN
F 3 "~" H 9750 5050 50  0001 C CNN
	1    9750 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	9650 5050 9650 5500
Wire Wire Line
	9650 5500 8550 5500
Wire Wire Line
	8550 5500 8550 5250
Connection ~ 8550 5250
Wire Wire Line
	8550 5250 8350 5250
$EndSCHEMATC
