# SoilSense
Measurement of capacitance of soil probe to determine moisture content of soil
Uses an LC oscillator at f from 50-100MHz
Frequency of oscillation is proportional to the inverse root of capacitance

At these frequencies and temperatures 0 < T < 30 degC the dielectric function of water is ~80; small changes in soil moisture content create a large change in capacitance

There are two physical prototypes, mark I & II:

Mark I
Uses a Clapp oscillator and emitter follower with BC547B transistors.  In the tank circuit is a toroidal inductor of 13-16 turns and 3x 22pF capacitors.
Base input and stray capacitance estimated as 7 and 3 respectively.  

Mark II
Uses a Motorola VCO MC1648 a toroidal inductor of 9 turns and 10pF cpacitor in the tank circuit (adds to input capacitance of 6pF)

Marks I & II use the same prescaler and counter circuits to convert the high frequency oscillation to a low frequency square wave, reducing the frequency by 1/2^17 

The low frequency is measured using a 16MHz 5V Promini and the result sent to a gateway using a cheap 433MHz RF transmitter 


Repository contains:

Clapp circuit 181226.log	:	LTSpice simulation of Clapp oscillator and transmission line
ReadNodeDataNew.R	:	R analysis of data from soil probe
Schematic_Soil-Sense-Mark-1_buffered-Clapp-circuit_20181225231505.pdf	:	Schematic of circuit for SoilSenseMark1 from EasyEDA
SketchVxx.cpp	:	Old - hidden
SoilSenseMark1	:	C++ code for ProMini measurement and reporting sequence
SoilSenseMark2	:	C++ code for ProMini measurement and reporting sequence
SoilSenseMark2_board.pdf	:	Vero board design for SoilSenseMark2 from Eagle
SoilSenseMark2_schematic.pdf	:	Schematic of circuit for SoilSenseMark2 from Eagle
freqCounterV4.brd	:	Vero board design for SoilSenseMark2 from Eagle
freqCounterV4.sch	:	Schematic of circuit for SoilSenseMark2 from Eagle


