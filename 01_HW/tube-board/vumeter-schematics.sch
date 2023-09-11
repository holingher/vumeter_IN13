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
L Amplifier_Operational:LM358 U1
U 2 1 5DF71836
P 3800 2200
F 0 "U1" H 3800 2567 50  0000 C CNN
F 1 "LM358" H 3800 2476 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3800 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 3800 2200 50  0001 C CNN
	2    3800 2200
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM358 U1
U 3 1 5DF94615
P 1350 4850
F 0 "U1" H 1308 4896 50  0000 L CNN
F 1 "LM358" H 1308 4805 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 1350 4850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 1350 4850 50  0001 C CNN
	3    1350 4850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MJE13003 Q2
U 1 1 5DF9ECB9
P 4700 2200
F 0 "Q2" H 4892 2246 50  0000 L CNN
F 1 "MJE340" H 4892 2155 50  0000 L CNN
F 2 "vumeter:TO-126-3_Vertical-HV" H 4900 2125 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/MJE13003-D.PDF" H 4700 2200 50  0001 L CNN
	1    4700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2200 4500 2200
Wire Wire Line
	4800 2400 4800 2550
Wire Wire Line
	4800 2550 3500 2550
Wire Wire Line
	3500 2550 3500 2300
$Comp
L Device:R_US R_Sense2
U 1 1 5DFA7FD5
P 4800 2850
F 0 "R_Sense2" H 4868 2896 50  0000 L CNN
F 1 "330" H 4868 2805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4840 2840 50  0001 C CNN
F 3 "RMCF1206JT330R" H 4800 2850 50  0001 C CNN
	1    4800 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2550 4800 2700
Connection ~ 4800 2550
$Comp
L nixies-us:IN-13 N2
U 1 1 5DFB2365
P 4900 1550
F 0 "N2" V 4900 1778 45  0000 L CNN
F 1 "IN-13" H 4900 1550 45  0001 L BNN
F 2 "vumeter:fixed-IN-13-footprint-wide-sockets" H 4930 1700 20  0001 C CNN
F 3 "" H 4900 1550 50  0001 C CNN
	1    4900 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 1850 4800 2000
$Comp
L Device:R_POT R_adj2
U 1 1 5DFB7E84
P 4800 3300
F 0 "R_adj2" H 4730 3346 50  0000 R CNN
F 1 "3266X >1k" H 4730 3255 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266X_Horizontal" H 4800 3300 50  0001 C CNN
F 3 "~" H 4800 3300 50  0001 C CNN
	1    4800 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3300 4950 3450
Wire Wire Line
	4950 3450 4800 3450
Wire Wire Line
	4800 3000 4800 3150
Wire Wire Line
	4800 3700 4800 3450
Connection ~ 4800 3450
$Comp
L Device:R_US R_Cathode2
U 1 1 5DFBA043
P 5350 2550
F 0 "R_Cathode2" H 5418 2596 50  0000 L CNN
F 1 "550k" H 5418 2505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5390 2540 50  0001 C CNN
F 3 "~" H 5350 2550 50  0001 C CNN
	1    5350 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1850 5350 1850
Wire Wire Line
	5350 1850 5350 2400
Wire Wire Line
	5350 2700 5350 3700
Wire Wire Line
	5350 3700 4800 3700
Text GLabel 4900 900  0    50   Input ~ 0
HV1
Wire Wire Line
	4900 900  4900 1250
$Comp
L Amplifier_Operational:LM358 U1
U 1 1 5DFE5343
P 2650 2200
F 0 "U1" H 2650 2567 50  0000 C CNN
F 1 "LM358" H 2650 2476 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2650 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 2650 2200 50  0001 C CNN
	1    2650 2200
	-1   0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MJE13003 Q1
U 1 1 5DFE5349
P 1750 2200
F 0 "Q1" H 1942 2246 50  0000 L CNN
F 1 "MJE340" H 1942 2155 50  0000 L CNN
F 2 "vumeter:TO-126-3_Vertical-HV" H 1950 2125 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/MJE13003-D.PDF" H 1750 2200 50  0001 L CNN
	1    1750 2200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2350 2200 1950 2200
Wire Wire Line
	1650 2400 1650 2550
Wire Wire Line
	1650 2550 2950 2550
Wire Wire Line
	2950 2550 2950 2300
$Comp
L Device:R_US R_Sense1
U 1 1 5DFE5353
P 1650 2850
F 0 "R_Sense1" H 1582 2896 50  0000 R CNN
F 1 "330" H 1582 2805 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1690 2840 50  0001 C CNN
F 3 "RMCF1206JT330R" H 1650 2850 50  0001 C CNN
	1    1650 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1650 2550 1650 2700
Connection ~ 1650 2550
$Comp
L nixies-us:IN-13 N1
U 1 1 5DFE5361
P 1550 1550
F 0 "N1" V 1550 1778 45  0000 L CNN
F 1 "IN-13" H 1550 1550 45  0001 L BNN
F 2 "vumeter:fixed-IN-13-footprint-wide-sockets" H 1580 1700 20  0001 C CNN
F 3 "" H 1550 1550 50  0001 C CNN
	1    1550 1550
	0    -1   1    0   
$EndComp
Wire Wire Line
	1650 1850 1650 2000
$Comp
L Device:R_POT R_adj1
U 1 1 5DFE5368
P 1650 3300
F 0 "R_adj1" H 1580 3346 50  0000 R CNN
F 1 "3266X >1k" H 1580 3255 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266X_Horizontal" H 1650 3300 50  0001 C CNN
F 3 "3266X-1-102LF‎" H 1650 3300 50  0001 C CNN
	1    1650 3300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1500 3300 1500 3450
Wire Wire Line
	1500 3450 1650 3450
Wire Wire Line
	1650 3000 1650 3150
Wire Wire Line
	1650 3700 1650 3450
Connection ~ 1650 3450
$Comp
L Device:R_US R_Cathode1
U 1 1 5DFE5373
P 1100 2550
F 0 "R_Cathode1" H 1032 2596 50  0000 R CNN
F 1 "550k" H 1032 2505 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1140 2540 50  0001 C CNN
F 3 "RMCF1206FT560K" H 1100 2550 50  0001 C CNN
	1    1100 2550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1450 1850 1100 1850
Wire Wire Line
	1100 1850 1100 2400
Wire Wire Line
	1100 2700 1100 3700
Wire Wire Line
	1100 3700 1650 3700
Text GLabel 1550 900  2    50   Input ~ 0
HV1
Wire Wire Line
	1550 900  1550 1250
Text GLabel 1600 5750 2    50   Input ~ 0
HV1
Text GLabel 1100 5750 0    50   Input ~ 0
HV2
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5DFFAD70
P 1350 5750
F 0 "JP1" H 1350 5955 50  0000 C CNN
F 1 "HV Bridge" H 1350 5864 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1350 5750 50  0001 C CNN
F 3 "~" H 1350 5750 50  0001 C CNN
	1    1350 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5750 1100 5750
Wire Wire Line
	1500 5750 1600 5750
$Comp
L power:GND #PWR01
U 1 1 5DFFD2AC
P 1250 5150
F 0 "#PWR01" H 1250 4900 50  0001 C CNN
F 1 "GND" H 1255 4977 50  0000 C CNN
F 2 "" H 1250 5150 50  0001 C CNN
F 3 "" H 1250 5150 50  0001 C CNN
	1    1250 5150
	1    0    0    -1  
$EndComp
Text GLabel 1250 4550 0    50   Input ~ 0
V_mid
$Comp
L Device:C C_pwm1
U 1 1 5E0002CC
P 2800 1600
F 0 "C_pwm1" V 2548 1600 50  0000 C CNN
F 1 "0.1u" V 2639 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2838 1450 50  0001 C CNN
F 3 "C1206C104K5RAC7800" H 2800 1600 50  0001 C CNN
	1    2800 1600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E003F26
P 2650 1600
F 0 "#PWR04" H 2650 1350 50  0001 C CNN
F 1 "GND" V 2655 1472 50  0000 R CNN
F 2 "" H 2650 1600 50  0001 C CNN
F 3 "" H 2650 1600 50  0001 C CNN
	1    2650 1600
	0    1    -1   0   
$EndComp
Wire Wire Line
	2950 1350 2950 1600
Wire Wire Line
	2950 1600 2950 2100
Connection ~ 2950 1600
Text GLabel 2950 1050 0    50   Input ~ 0
PWM1
$Comp
L Device:C C_pwm2
U 1 1 5E01461F
P 3650 1600
F 0 "C_pwm2" V 3398 1600 50  0000 C CNN
F 1 "0.1u" V 3489 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3688 1450 50  0001 C CNN
F 3 "~" H 3650 1600 50  0001 C CNN
	1    3650 1600
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5E01462B
P 3800 1600
F 0 "#PWR06" H 3800 1350 50  0001 C CNN
F 1 "GND" V 3805 1472 50  0000 R CNN
F 2 "" H 3800 1600 50  0001 C CNN
F 3 "" H 3800 1600 50  0001 C CNN
	1    3800 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 1350 3500 1600
Wire Wire Line
	3500 1600 3500 2100
Connection ~ 3500 1600
Text GLabel 3500 1050 2    50   Input ~ 0
PWM2
$Comp
L MCU_Microchip_SAMD:ATSAMD11C14A-SS U2
U 1 1 5E015C8F
P 5200 6000
F 0 "U2" H 5200 6881 50  0000 C CNN
F 1 "ATSAMD11C14" H 5200 6790 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5200 4950 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42363-SAM-D11_Datasheet.pdf" H 5200 5300 50  0001 C CNN
	1    5200 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5E01A58E
P 6050 5300
F 0 "#PWR07" H 6050 5050 50  0001 C CNN
F 1 "GND" V 6055 5172 50  0000 R CNN
F 2 "" H 6050 5300 50  0001 C CNN
F 3 "" H 6050 5300 50  0001 C CNN
	1    6050 5300
	0    -1   1    0   
$EndComp
Wire Wire Line
	5750 5300 5200 5300
$Comp
L Connector:Conn_ARM_JTAG_SWD_10 J1
U 1 1 5E01CCB0
P 2950 6000
F 0 "J1" H 2507 6046 50  0000 R CNN
F 1 "SWD" H 2507 5955 50  0000 R CNN
F 2 "vumeter:PinHeader_2x05_P1.27mm_Vertical-smallholes" H 2950 6000 50  0001 C CNN
F 3 "20021111-00010T4LF‎" V 2600 4750 50  0001 C CNN
	1    2950 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5400 2950 5300
Wire Wire Line
	2950 5300 5200 5300
Connection ~ 5200 5300
Text GLabel 4200 5300 1    50   Input ~ 0
3V3
Wire Wire Line
	2850 6600 2950 6600
$Comp
L power:GND #PWR05
U 1 1 5E036241
P 5200 6700
F 0 "#PWR05" H 5200 6450 50  0001 C CNN
F 1 "GND" H 5205 6527 50  0000 C CNN
F 2 "" H 5200 6700 50  0001 C CNN
F 3 "" H 5200 6700 50  0001 C CNN
	1    5200 6700
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5E03680F
P 2950 6600
F 0 "#PWR02" H 2950 6350 50  0001 C CNN
F 1 "GND" H 2955 6427 50  0000 C CNN
F 2 "" H 2950 6600 50  0001 C CNN
F 3 "" H 2950 6600 50  0001 C CNN
	1    2950 6600
	-1   0    0    -1  
$EndComp
Connection ~ 2950 6600
$Comp
L Regulator_Linear:LM1117-3.3 U3
U 1 1 5E0AEB06
P 2650 4550
F 0 "U3" H 2650 4792 50  0000 C CNN
F 1 "LM1117-3.3" H 2650 4701 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2650 4550 50  0001 C CNN
F 3 "LM1117MPX-3.3/NOPB" H 2650 4550 50  0001 C CNN
	1    2650 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5E0B5114
P 2650 5100
F 0 "#PWR09" H 2650 4850 50  0001 C CNN
F 1 "GND" H 2655 4927 50  0000 C CNN
F 2 "" H 2650 5100 50  0001 C CNN
F 3 "" H 2650 5100 50  0001 C CNN
	1    2650 5100
	-1   0    0    -1  
$EndComp
Text GLabel 2950 4550 2    50   Input ~ 0
3V3
$Comp
L Amplifier_Operational:LM358 U4
U 2 1 5E0DEE69
P 8850 2200
F 0 "U4" H 8850 2567 50  0000 C CNN
F 1 "LM358" H 8850 2476 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 8850 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 8850 2200 50  0001 C CNN
	2    8850 2200
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MJE13003 Q4
U 1 1 5E0DEE6F
P 9750 2200
F 0 "Q4" H 9942 2246 50  0000 L CNN
F 1 "MJE340" H 9942 2155 50  0000 L CNN
F 2 "vumeter:TO-126-3_Vertical-HV" H 9950 2125 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/MJE13003-D.PDF" H 9750 2200 50  0001 L CNN
	1    9750 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 2200 9550 2200
Wire Wire Line
	9850 2400 9850 2550
Wire Wire Line
	9850 2550 8550 2550
Wire Wire Line
	8550 2550 8550 2300
$Comp
L Device:R_US R_Sense4
U 1 1 5E0DEE79
P 9850 2850
F 0 "R_Sense4" H 9918 2896 50  0000 L CNN
F 1 "330" H 9918 2805 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 9890 2840 50  0001 C CNN
F 3 "~" H 9850 2850 50  0001 C CNN
	1    9850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2550 9850 2700
Connection ~ 9850 2550
$Comp
L nixies-us:IN-13 N4
U 1 1 5E0DEE87
P 9950 1550
F 0 "N4" V 9950 1778 45  0000 L CNN
F 1 "IN-13" H 9950 1550 45  0001 L BNN
F 2 "vumeter:fixed-IN-13-footprint-wide-sockets" H 9980 1700 20  0001 C CNN
F 3 "" H 9950 1550 50  0001 C CNN
	1    9950 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 1850 9850 2000
$Comp
L Device:R_POT R_adj4
U 1 1 5E0DEE8E
P 9850 3300
F 0 "R_adj4" H 9780 3346 50  0000 R CNN
F 1 "3266X >1k" H 9780 3255 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266X_Horizontal" H 9850 3300 50  0001 C CNN
F 3 "~" H 9850 3300 50  0001 C CNN
	1    9850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 3300 10000 3450
Wire Wire Line
	10000 3450 9850 3450
Wire Wire Line
	9850 3000 9850 3150
Wire Wire Line
	9850 3700 9850 3450
Connection ~ 9850 3450
$Comp
L Device:R_US R_Cathode4
U 1 1 5E0DEE99
P 10400 2550
F 0 "R_Cathode4" H 10468 2596 50  0000 L CNN
F 1 "550k" H 10468 2505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 10440 2540 50  0001 C CNN
F 3 "~" H 10400 2550 50  0001 C CNN
	1    10400 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 1850 10400 1850
Wire Wire Line
	10400 1850 10400 2400
Wire Wire Line
	10400 2700 10400 3700
Wire Wire Line
	10400 3700 9850 3700
Text GLabel 9950 900  0    50   Input ~ 0
HV2
Wire Wire Line
	9950 900  9950 1250
$Comp
L Amplifier_Operational:LM358 U4
U 1 1 5E0DEEAC
P 7700 2200
F 0 "U4" H 7700 2567 50  0000 C CNN
F 1 "LM358" H 7700 2476 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7700 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 7700 2200 50  0001 C CNN
	1    7700 2200
	-1   0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MJE13003 Q3
U 1 1 5E0DEEB2
P 6800 2200
F 0 "Q3" H 6992 2246 50  0000 L CNN
F 1 "MJE340" H 6992 2155 50  0000 L CNN
F 2 "vumeter:TO-126-3_Vertical-HV" H 7000 2125 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/MJE13003-D.PDF" H 6800 2200 50  0001 L CNN
	1    6800 2200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7400 2200 7000 2200
Wire Wire Line
	6700 2400 6700 2550
Wire Wire Line
	6700 2550 8000 2550
Wire Wire Line
	8000 2550 8000 2300
$Comp
L Device:R_US R_Sense3
U 1 1 5E0DEEBC
P 6700 2850
F 0 "R_Sense3" H 6632 2896 50  0000 R CNN
F 1 "330" H 6632 2805 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6740 2840 50  0001 C CNN
F 3 "RMCF1206JT330R" H 6700 2850 50  0001 C CNN
	1    6700 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6700 2550 6700 2700
Connection ~ 6700 2550
$Comp
L nixies-us:IN-13 N3
U 1 1 5E0DEECA
P 6600 1550
F 0 "N3" V 6600 1778 45  0000 L CNN
F 1 "IN-13" H 6600 1550 45  0001 L BNN
F 2 "vumeter:fixed-IN-13-footprint-wide-sockets" H 6630 1700 20  0001 C CNN
F 3 "" H 6600 1550 50  0001 C CNN
	1    6600 1550
	0    -1   1    0   
$EndComp
Wire Wire Line
	6700 1850 6700 2000
$Comp
L Device:R_POT R_adj3
U 1 1 5E0DEED1
P 6700 3300
F 0 "R_adj3" H 6630 3346 50  0000 R CNN
F 1 "3266X >1k" H 6630 3255 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266X_Horizontal" H 6700 3300 50  0001 C CNN
F 3 "~" H 6700 3300 50  0001 C CNN
	1    6700 3300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6550 3300 6550 3450
Wire Wire Line
	6550 3450 6700 3450
Wire Wire Line
	6700 3000 6700 3150
Wire Wire Line
	6700 3700 6700 3450
Connection ~ 6700 3450
$Comp
L Device:R_US R_Cathode3
U 1 1 5E0DEEDC
P 6150 2550
F 0 "R_Cathode3" H 6082 2596 50  0000 R CNN
F 1 "550k" H 6082 2505 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6190 2540 50  0001 C CNN
F 3 "~" H 6150 2550 50  0001 C CNN
	1    6150 2550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6500 1850 6150 1850
Wire Wire Line
	6150 1850 6150 2400
Wire Wire Line
	6150 2700 6150 3700
Wire Wire Line
	6150 3700 6700 3700
Text GLabel 6600 900  2    50   Input ~ 0
HV2
Wire Wire Line
	6600 900  6600 1250
$Comp
L Device:C C_pwm3
U 1 1 5E0DEEEF
P 7850 1600
F 0 "C_pwm3" V 7598 1600 50  0000 C CNN
F 1 "0.1u" V 7689 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7888 1450 50  0001 C CNN
F 3 "~" H 7850 1600 50  0001 C CNN
	1    7850 1600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5E0DEEFB
P 7700 1600
F 0 "#PWR011" H 7700 1350 50  0001 C CNN
F 1 "GND" V 7705 1472 50  0000 R CNN
F 2 "" H 7700 1600 50  0001 C CNN
F 3 "" H 7700 1600 50  0001 C CNN
	1    7700 1600
	0    1    -1   0   
$EndComp
Wire Wire Line
	8000 1350 8000 1600
Wire Wire Line
	8000 1600 8000 2100
Connection ~ 8000 1600
Text GLabel 8000 1050 0    50   Input ~ 0
PWM3
$Comp
L Device:C C_pwm4
U 1 1 5E0DEF05
P 8700 1600
F 0 "C_pwm4" V 8448 1600 50  0000 C CNN
F 1 "0.1u" V 8539 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8738 1450 50  0001 C CNN
F 3 "~" H 8700 1600 50  0001 C CNN
	1    8700 1600
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5E0DEF11
P 8850 1600
F 0 "#PWR012" H 8850 1350 50  0001 C CNN
F 1 "GND" V 8855 1472 50  0000 R CNN
F 2 "" H 8850 1600 50  0001 C CNN
F 3 "" H 8850 1600 50  0001 C CNN
	1    8850 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8550 1350 8550 1600
Wire Wire Line
	8550 1600 8550 2100
Connection ~ 8550 1600
Text GLabel 8550 1050 2    50   Input ~ 0
PWM4
Text GLabel 5700 5800 2    50   Input ~ 0
PWM2
Text GLabel 5700 5900 2    50   Input ~ 0
PWM1
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 5E107773
P 6350 7150
F 0 "J3" V 6222 7330 50  0000 L CNN
F 1 "Conn_01x04" V 6313 7330 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6350 7150 50  0001 C CNN
F 3 "M20-9990445" H 6350 7150 50  0001 C CNN
	1    6350 7150
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5E10A2A7
P 4200 7200
F 0 "J2" V 4072 7380 50  0000 L CNN
F 1 "Conn_01x04" V 4163 7380 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4200 7200 50  0001 C CNN
F 3 "M20-9990445" H 4200 7200 50  0001 C CNN
	1    4200 7200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5E114F73
P 4000 7000
F 0 "#PWR015" H 4000 6750 50  0001 C CNN
F 1 "GND" H 4005 6827 50  0000 C CNN
F 2 "" H 4000 7000 50  0001 C CNN
F 3 "" H 4000 7000 50  0001 C CNN
	1    4000 7000
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5E115979
P 6150 6950
F 0 "#PWR016" H 6150 6700 50  0001 C CNN
F 1 "GND" H 6155 6777 50  0000 C CNN
F 2 "" H 6150 6950 50  0001 C CNN
F 3 "" H 6150 6950 50  0001 C CNN
	1    6150 6950
	1    0    0    1   
$EndComp
Text GLabel 4300 7000 1    50   Input ~ 0
V_mid
Text GLabel 6450 6950 1    50   Input ~ 0
V_mid
$Comp
L Amplifier_Operational:LM358 U4
U 3 1 5DF76687
P 1800 4850
F 0 "U4" H 1758 4896 50  0000 L CNN
F 1 "LM358" H 1758 4805 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 1800 4850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2904-n.pdf" H 1800 4850 50  0001 C CNN
	3    1800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4550 1700 4550
$Comp
L power:GND #PWR014
U 1 1 5DF86B93
P 1700 5150
F 0 "#PWR014" H 1700 4900 50  0001 C CNN
F 1 "GND" H 1705 4977 50  0000 C CNN
F 2 "" H 1700 5150 50  0001 C CNN
F 3 "" H 1700 5150 50  0001 C CNN
	1    1700 5150
	1    0    0    -1  
$EndComp
Text GLabel 5700 6300 2    50   Input ~ 0
rx1
Text GLabel 5700 6200 2    50   Input ~ 0
tx1
Text GLabel 4100 7000 1    50   Input ~ 0
tx1
Text GLabel 4200 7000 1    50   Input ~ 0
rx1
Text GLabel 6350 6950 1    50   Input ~ 0
tx2
Text GLabel 6250 6950 1    50   Input ~ 0
rx2
$Comp
L vumeter:NCH8200HV U6
U 1 1 5DF77E40
P 10500 4900
F 0 "U6" H 10500 4865 50  0000 C CNN
F 1 "NCH8200HV" H 10500 4774 50  0000 C CNN
F 2 "vumeter:NCH8200HV" H 10500 4900 50  0001 C CNN
F 3 "" H 10500 4900 50  0001 C CNN
	1    10500 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5DF7C416
P 8800 5500
F 0 "#PWR018" H 8800 5250 50  0001 C CNN
F 1 "GND" V 8805 5372 50  0000 R CNN
F 2 "" H 8800 5500 50  0001 C CNN
F 3 "" H 8800 5500 50  0001 C CNN
	1    8800 5500
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5DF7C76D
P 10900 5500
F 0 "#PWR020" H 10900 5250 50  0001 C CNN
F 1 "GND" V 10905 5372 50  0000 R CNN
F 2 "" H 10900 5500 50  0001 C CNN
F 3 "" H 10900 5500 50  0001 C CNN
	1    10900 5500
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5DF7CB2A
P 10100 5500
F 0 "#PWR019" H 10100 5250 50  0001 C CNN
F 1 "GND" V 10105 5372 50  0000 R CNN
F 2 "" H 10100 5500 50  0001 C CNN
F 3 "" H 10100 5500 50  0001 C CNN
	1    10100 5500
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5DF7D0D9
P 8000 5500
F 0 "#PWR017" H 8000 5250 50  0001 C CNN
F 1 "GND" V 8005 5372 50  0000 R CNN
F 2 "" H 8000 5500 50  0001 C CNN
F 3 "" H 8000 5500 50  0001 C CNN
	1    8000 5500
	0    1    -1   0   
$EndComp
Text GLabel 6500 5100 0    50   Input ~ 0
V_mid
Text GLabel 8800 5200 2    50   Input ~ 0
HV1
Text GLabel 10900 5200 2    50   Input ~ 0
HV2
$Comp
L Device:C C_decouple1
U 1 1 5E0182A3
P 5900 5300
F 0 "C_decouple1" V 5648 5300 50  0000 C CNN
F 1 "0.1u" V 5739 5300 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5938 5150 50  0001 C CNN
F 3 "C1206C104K5RAC7800" H 5900 5300 50  0001 C CNN
	1    5900 5300
	0    -1   1    0   
$EndComp
$Comp
L vumeter:NCH8200HV U5
U 1 1 5DF76F83
P 8400 4900
F 0 "U5" H 8400 4865 50  0000 C CNN
F 1 "NCH8200HV" H 8400 4774 50  0000 C CNN
F 2 "vumeter:NCH8200HV" H 8400 4900 50  0001 C CNN
F 3 "" H 8400 4900 50  0001 C CNN
	1    8400 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C_pwr2
U 1 1 5E156E90
P 2950 4850
F 0 "C_pwr2" V 2698 4850 50  0000 C CNN
F 1 "10u" V 2789 4850 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2988 4700 50  0001 C CNN
F 3 "GRT31CR61H106ME01L" H 2950 4850 50  0001 C CNN
	1    2950 4850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2650 4850 2650 5000
Wire Wire Line
	2950 4700 2950 4550
Wire Wire Line
	2950 5000 2650 5000
Connection ~ 2650 5000
Wire Wire Line
	2650 5000 2650 5100
$Comp
L Device:C C_pwr1
U 1 1 5E16A6E7
P 2350 4850
F 0 "C_pwr1" V 2098 4850 50  0000 C CNN
F 1 "10u" V 2189 4850 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2388 4700 50  0001 C CNN
F 3 "GRT31CR61H106ME01L" H 2350 4850 50  0001 C CNN
	1    2350 4850
	1    0    0    1   
$EndComp
Wire Wire Line
	2350 4700 2350 4550
Connection ~ 2350 4550
Wire Wire Line
	2350 5000 2650 5000
Wire Wire Line
	1700 4550 2350 4550
Connection ~ 1700 4550
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5E18D6B5
P 9150 6150
F 0 "H1" H 9250 6199 50  0000 L CNN
F 1 "MountingHole_Pad" H 9250 6108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 9150 6150 50  0001 C CNN
F 3 "~" H 9150 6150 50  0001 C CNN
	1    9150 6150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5E19C90D
P 9450 6150
F 0 "H2" H 9550 6199 50  0000 L CNN
F 1 "MountingHole_Pad" H 9550 6108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 9450 6150 50  0001 C CNN
F 3 "~" H 9450 6150 50  0001 C CNN
	1    9450 6150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5E19CEDD
P 9750 6150
F 0 "H3" H 9850 6199 50  0000 L CNN
F 1 "MountingHole_Pad" H 9850 6108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 9750 6150 50  0001 C CNN
F 3 "~" H 9750 6150 50  0001 C CNN
	1    9750 6150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5E19D344
P 10050 6150
F 0 "H4" H 10150 6199 50  0000 L CNN
F 1 "MountingHole_Pad" H 10150 6108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 10050 6150 50  0001 C CNN
F 3 "~" H 10050 6150 50  0001 C CNN
	1    10050 6150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J4
U 1 1 5E1BE142
P 4200 4500
F 0 "J4" H 4228 4476 50  0000 L CNN
F 1 "Standoff_1" H 4228 4385 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 4200 4500 50  0001 C CNN
F 3 "76341-304LF" H 4200 4500 50  0001 C CNN
	1    4200 4500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5E1BFFE6
P 5300 4550
F 0 "J5" H 5192 4125 50  0000 C CNN
F 1 "Standoff_2" H 5192 4216 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5300 4550 50  0001 C CNN
F 3 "76341-304LF" H 5300 4550 50  0001 C CNN
	1    5300 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	5500 4350 5500 4450
Wire Wire Line
	5500 4450 5500 4550
Connection ~ 5500 4450
Wire Wire Line
	5500 4550 5500 4650
Connection ~ 5500 4550
Wire Wire Line
	4000 4400 4000 4500
Wire Wire Line
	4000 4500 4000 4600
Connection ~ 4000 4500
Wire Wire Line
	4000 4600 4000 4700
Connection ~ 4000 4600
Wire Wire Line
	7950 5200 7950 4850
Wire Wire Line
	7950 4850 10100 4850
Wire Wire Line
	10100 4850 10100 5200
Wire Wire Line
	7950 5200 8000 5200
Wire Wire Line
	3450 5700 4700 5700
Wire Wire Line
	3450 5900 4700 5900
Wire Wire Line
	3450 6000 4700 6000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DFA0D02
P 1250 4550
F 0 "#FLG0101" H 1250 4625 50  0001 C CNN
F 1 "PWR_FLAG" H 1250 4723 50  0000 C CNN
F 2 "" H 1250 4550 50  0001 C CNN
F 3 "~" H 1250 4550 50  0001 C CNN
	1    1250 4550
	1    0    0    -1  
$EndComp
Connection ~ 1250 4550
NoConn ~ 3450 6100
NoConn ~ 3450 6200
NoConn ~ 9150 6250
NoConn ~ 9450 6250
NoConn ~ 9750 6250
NoConn ~ 10050 6250
$Comp
L power:GND #PWR0101
U 1 1 5DFCFB19
P 6700 3800
F 0 "#PWR0101" H 6700 3550 50  0001 C CNN
F 1 "GND" H 6705 3627 50  0000 C CNN
F 2 "" H 6700 3800 50  0001 C CNN
F 3 "" H 6700 3800 50  0001 C CNN
	1    6700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3700 6700 3800
Connection ~ 6700 3700
Wire Wire Line
	1650 3700 4800 3700
Connection ~ 1650 3700
Connection ~ 4800 3700
Wire Wire Line
	5350 3700 6150 3700
Connection ~ 5350 3700
Connection ~ 6150 3700
Wire Wire Line
	6700 3700 9850 3700
Connection ~ 9850 3700
$Comp
L Device:Jumper JP2
U 1 1 5DFA1642
P 6800 5100
F 0 "JP2" H 6800 5364 50  0000 C CNN
F 1 "HV_pwr" H 6800 5273 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6800 5100 50  0001 C CNN
F 3 "68000-102HLF, STC02SYAN" H 6800 5100 50  0001 C CNN
	1    6800 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5300 8000 5200
Connection ~ 8000 5200
Text GLabel 1650 1950 0    50   Input ~ 0
M1
Text GLabel 4800 1950 0    50   Input ~ 0
M2
Text GLabel 6700 1950 0    50   Input ~ 0
M3
Text GLabel 9850 1950 0    50   Input ~ 0
M4
$Comp
L Connector:Conn_01x01_Male J6
U 1 1 5DFF7CF9
P 8900 6450
F 0 "J6" H 9008 6631 50  0000 C CNN
F 1 "Conn_01x01_Male" H 9008 6540 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 8900 6450 50  0001 C CNN
F 3 "~" H 8900 6450 50  0001 C CNN
	1    8900 6450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J7
U 1 1 5DFF8E34
P 10250 6450
F 0 "J7" H 10222 6382 50  0000 R CNN
F 1 "Conn_01x01_Male" H 10222 6473 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 10250 6450 50  0001 C CNN
F 3 "~" H 10250 6450 50  0001 C CNN
	1    10250 6450
	-1   0    0    1   
$EndComp
NoConn ~ 9100 6450
NoConn ~ 10050 6450
Text GLabel 5700 6000 2    50   Input ~ 0
tx2
Text GLabel 5700 6100 2    50   Input ~ 0
rx2
Text GLabel 4700 6200 0    50   Input ~ 0
PWM3
Text GLabel 4700 6300 0    50   Input ~ 0
PWM4
Text Notes 6000 5800 0    50   ~ 0
v0 - compat
Text Notes 6000 5900 0    50   ~ 0
v0 - compat
$Comp
L Power_Management:TPS22917DBV U7
U 1 1 5E00C472
P 7500 5400
F 0 "U7" H 7500 5767 50  0000 C CNN
F 1 "TPS2281DBVR" H 7500 5676 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 7500 5900 50  0001 C CNN
F 3 "TPS2281DBVR" H 7550 4700 50  0001 C CNN
	1    7500 5400
	1    0    0    -1  
$EndComp
NoConn ~ 7100 5400
Wire Wire Line
	8000 5300 7900 5300
$Comp
L power:GND #PWR0102
U 1 1 5E022973
P 7500 5700
F 0 "#PWR0102" H 7500 5450 50  0001 C CNN
F 1 "GND" V 7505 5572 50  0000 R CNN
F 2 "" H 7500 5700 50  0001 C CNN
F 3 "" H 7500 5700 50  0001 C CNN
	1    7500 5700
	-1   0    0    -1  
$EndComp
NoConn ~ 7900 5400
Wire Wire Line
	5700 5700 7100 5700
Wire Wire Line
	7100 5700 7100 5500
Wire Wire Line
	7100 5100 7100 5300
Text GLabel 6950 5700 3    50   Input ~ 0
hv_en
$Comp
L Device:R_US R_pwm1
U 1 1 5E002090
P 2950 1200
F 0 "R_pwm1" H 3018 1246 50  0000 L CNN
F 1 "5k" H 3018 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2990 1190 50  0001 C CNN
F 3 "RNCP1206FTD2K00" H 2950 1200 50  0001 C CNN
	1    2950 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R_pwm2
U 1 1 5E014625
P 3500 1200
F 0 "R_pwm2" H 3432 1246 50  0000 R CNN
F 1 "5k" H 3432 1155 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3540 1190 50  0001 C CNN
F 3 "~" H 3500 1200 50  0001 C CNN
	1    3500 1200
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R_pwm3
U 1 1 5E0DEEF5
P 8000 1200
F 0 "R_pwm3" H 8068 1246 50  0000 L CNN
F 1 "5k" H 8068 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8040 1190 50  0001 C CNN
F 3 "~" H 8000 1200 50  0001 C CNN
	1    8000 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R_pwm4
U 1 1 5E0DEF0B
P 8550 1200
F 0 "R_pwm4" H 8482 1246 50  0000 R CNN
F 1 "5k" H 8482 1155 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8590 1190 50  0001 C CNN
F 3 "~" H 8550 1200 50  0001 C CNN
	1    8550 1200
	-1   0    0    -1  
$EndComp
Text GLabel 1100 2050 0    50   Input ~ 0
M5
Text GLabel 5350 2100 0    50   Input ~ 0
M6
Text GLabel 6150 2100 0    50   Input ~ 0
M7
Text GLabel 10400 2100 0    50   Input ~ 0
M8
Text GLabel 4050 5700 1    50   Input ~ 0
rst
Text GLabel 4200 5900 1    50   Input ~ 0
swclk
Text GLabel 4050 6000 3    50   Input ~ 0
swdio
$EndSCHEMATC
