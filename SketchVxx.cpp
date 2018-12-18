/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */



/*


v3 working with Uno and Promini.  Mirror file on AtmelStudio
v4 with inclusion of OneWire to read DS18b20 digital thermometer
project soilSensev6 has a stable method to measure frequency
the first result is obtained by combining two measurements at different timer2 clock frequencies
the second result is obtained only from the measurement at the lowest frequency
the MC is in powersave during the measurement
measurement is started and terminated on the rising edge of the signal to INT1 (pin3)
there is a problem with the variation calculation for large values of nCount
v7 change timing of measurement loop to reduce polarisation
v8 distribute soil probe measurements uniformly;
improve the estimate of measurement time
v11 revert to usew of main loop rather than interrupts
*/

#include <OneWire.h>
// use RadioHead to transmit messages
// with a simple ASK transmitter
// Implements a simplex (one-way) transmitter with an TX-C1 module

#include <RH_ASK.h>
#include <LowPower.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#define GET_COUNT() count = TCNT2
#define USE_SER 0






//Note: an object of this class was already pre-instantiated in the .cpp file of this library, so you can simply access its methods (functions)
//      directly now through the object name "timer2"
//eRCaGuy_Timer2_Counter timer2;  //this is what the pre-instantiation line from the .cpp file looks like

volatile uint8_t count;

const int interruptPin = 3;
const int pinPowerCounter = 5;
const int pinPower1 = 6;
const int pinPower2 = 7;
const int pinLed = 13;
const byte receive_pin = -1;
const byte transmit_pin = 12;
const byte digT_pin = 8;



uint8_t preScaler[8] = {0, 0, 3, 5, 6, 7, 8, 10}; // powers of 2
const uint8_t clockFrequency = 4; // i.e. 2^4 in MHz
uint8_t preScalerSelect[2] = {6, 1}; // i.e. first pass divide by 256 : second by 1
const uint8_t nCount = 8; // number to average is 2^nCount

int pinPower = pinPower1;
bool measureOsc1 = true; // first measurement is for osc1
bool coarse = true; // first measurement is coarse; next fine
bool initialized = true; // when false can use for debug/learn
// store results here
uint32_t meanResults[2];
uint32_t variationResults[2];



const int baud = 500;
const byte node = 12;

OneWire  ds(digT_pin);  // a 4.7K pull-up resistor is necessary
// digital temperature sensor

RH_ASK driver(baud, receive_pin, transmit_pin);//speed, Rx pin, Tx pin
/// Constructor.
/// At present only one instance of RH_ASK per sketch is supported.
/// \param[in] speed The desired bit rate in bits per second
/// \param[in] rxPin The pin that is used to get data from the receiver
/// \param[in] txPin The pin that is used to send data to the transmitter
/// \param[in] pttPin The pin that is connected to the transmitter controller. It will be set HIGH to enable the transmitter (unless pttInverted is true).
/// \param[in] pttInverted true if you desire the pttin to be inverted so that LOW will enable the transmitter.


const byte nVcc = 6; // ADC reads to determine Vcc; power of 2

extern volatile unsigned long timer0_millis;
const uint32_t periodStatusReport = 600000L;
uint32_t periodOscReport = 30000L; // changed to check effect of polarisation
uint32_t nextStatusReport = 0L;
uint32_t nextOscReport = 0L;
uint32_t now = 0L; // beginning of time


uint32_t roundDiv(uint32_t a, uint32_t b);
uint32_t bitDiv(uint32_t a, byte b);

////////////////////////////////////struct PayloadItem/////////////////////////////////////
typedef struct {
	byte nodeId; //store this nodeId
	byte count;
	byte bin2usCoarse;
	byte bin2usFine;
	byte varCoarse;
	byte varFine;
	unsigned long coarseTime;
	unsigned long fineTime;
} PayloadItem;
// count keeps track of the # packets sent out

PayloadItem payloadTime;


////////////////////////////////////struct PayloadItem2/////////////////////////////////////
typedef struct {
	byte nodeId; //store this nodeId
	byte count;
	int temp;
	int Vcc;
	unsigned long millisec;
	unsigned long empty;
} PayloadItem2;
// time is approximate since  using only watchdog timer

PayloadItem2 payloadStatus;


//////////////////////////////////Initialise the ISR///////////////////////////////////////

ISR(INT1_vect) {
	GET_COUNT();
}

//////////////////////////////////flashLED///////////////////////////////////////
void flashLED(byte nTimes)
{
	for (uint8_t i = 0; i < nTimes; i++)
	{
		LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
		digitalWrite(pinLed, HIGH);
		LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
		digitalWrite(pinLed, LOW);
	}

}

//////////////////////////////////setup///////////////////////////////////////

void setup()
{


	pinMode(interruptPin, INPUT);
	pinMode(pinPower1, OUTPUT);
	pinMode(pinPower2, OUTPUT);
	pinMode(pinPowerCounter, OUTPUT);
	pinMode(pinLed, OUTPUT);



	// turn-off the ADC
	ADCSRA &= ~(1 << ADEN);

	//set-up timer2 to "normal" operation mode.  See datasheet pg. 147, 155, & 157-158 (incl. Table 18-8).
	//-This is important so that the timer2 counter, TCNT2, counts only UP and not also down.
	//Note:  this messes up PWM outputs on pins 3 & 11, as well as
	//interferes with the tone() library (http://arduino.cc/en/reference/tone)
	//-To do this we must make WGM22, WGM21, & WGM20, within TCCR2A & TCCR2B, all have values of 0.
	TCCR2A &= 0b11111100; //set WGM21 & WGM20 to 0 (see datasheet pg. 155).
	TCCR2B &= 0b11110111; //set WGM22 to 0 (see pg. 158).
	// no overflow to enable longer period in powersave
	TIMSK2 &= ~(_BV(TOIE2));

	// Initialise the IO
	if (!driver.init())
	{

		flashLED(10);
		while (true) {
			;; // do nothing forever
		}
	}


	payloadTime.count = 0;
	payloadTime.nodeId = node;
	payloadStatus.count = 0;
	payloadStatus.nodeId = node | 0x20; // bit 5 set to indicate status packet
	payloadStatus.empty = 0L;

	flashLED(3);

	#if USE_SER

	//print result
	Serial.begin(115200);
	delay(1000);
	if (!initialized)
	{
		int a = 20;
		int b = 7;
		Serial.print("20/7 = ");
		Serial.println(roundDiv(a,b));
		b = -7;
		Serial.print("20/-7 = ");
		Serial.println(roundDiv(a,b));
		a = -20;
		Serial.print("-20/-7 = ");
		Serial.println(roundDiv(a,b));
		uint32_t c = 385;
		Serial.print("385/256 with bitDiv = ");
		Serial.println(bitDiv(c,8));
		int32_t d = 0x7fffffff;
		Serial.print("0x7fffffff/256 with bitDiv = ");
		Serial.println(bitDiv(d,8));
		Serial.flush();
		Serial.end();
		initialized = true;
	}
	#endif

}
//////////////////////////////////sendTimePacket///////////////////////////////////////

void sendTimePacket()
{
	// send packet
	driver.send((uint8_t*)(&payloadTime), sizeof(payloadTime));
	driver.waitPacketSent();
	++payloadTime.count;
}
//////////////////////////////////sendStatusPacket///////////////////////////////////////

void sendStatusPacket()
{
	// send packet
	driver.send((uint8_t*)(&payloadStatus), sizeof(payloadStatus));
	driver.waitPacketSent();
	++payloadStatus.count;
}
//////////////////////////////////setPrescaler///////////////////////////////////////
void setPrescaler(byte value)
{
	// set prescaler for timer2
	// Bit 2:0 – CS22:0: Clock Select.  The three Clock Select bits select the clock source to be used by the Timer/Counter
	// off 0x0
	// clkT2S/1 0x01
	// clkT2S/8 0x02
	// clkT2S/32 0x03
	// clkT2S/64 0x04
	// clkT2S/128 0x05
	// clkT2S/256 0x06
	// clkT2S/1024 0x07
	// could develop an auto-ranging function
	// see pg 156 Atmel-8271I-AVR- ATmega-Datasheet_10/2014

	TCCR2B = TCCR2B & 0b11111000 | value;
}
/////////////////////////////////roundDiv//////////////////////////////////////////
uint32_t roundDiv(uint32_t a, uint32_t b)
// returns a/b rounded to the nearest integer
{
	uint32_t result =  ((a + (b >> 1)) / b);
	return result;
}

/////////////////////////////////////bitDiv//////////////////////////////////////
uint32_t bitDiv(uint32_t a, byte b)
// returns a / 2^b rounded to the nearest integer
{
	uint32_t result = (a + (bit(b - 1))) >> b;
	return result;
}

//////////////////////////////////readADC///////////////////////////////////////
uint32_t readADC( byte channel, byte noSamples )
{

	// http://forum.arduino.cc/index.php?topic=38119.0
	// function uses 2 to the power of noSamples

	TIMSK0 = 0; // turn off timer0 for lower jitter.  Note disables millis()

	uint32_t sumX;
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	/*
	A normal conversion takes 13 ADC clock cycles. The first conversion after the ADC is switched on (ADEN in
	ADCSRA is set) takes 25 ADC clock cycles in order to initialize the analog circuitry.
	When the bandgap reference voltage is used as input to the ADC, it will take a certain time for the voltage to
	stabilize. If not stabilized, the first value read after the first conversion may be wrong.
	*/
	ADCSRA = (1 << ADEN) | 0x07;
	// Bits 2:0 – ADPS[2:0]: ADC Prescaler Select Bits
	// These bits determine the division factor between the system clock frequency and the input clock to the ADC.
	ADMUX = channel; // the last 3 bits of MUX define the analogue pin
	// first 4 bits determine reference voltage
	delayMicroseconds(20000); // Wait for voltage to settle
	//dummy read
	ADCSRA |= _BV(ADSC); // start the conversion
	while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes
	sumX = 0;
	//  Generate an interrupt when the conversion is finished


	for (uint16_t i = 0; i < bit(noSamples) ; i++)
	{
		ADCSRA |= _BV(ADSC); // Start conversion
		while (bit_is_set(ADCSRA, ADSC)); // measuring
		uint32_t adcVal = ADCL | (ADCH << 8);
		sumX += adcVal;
	}

	timer0_millis += 27; // (8 x number of measurements x 13 + 12) / 1000 + time for settling
	TIMSK0 = 1; // turn timer0 back on
	// turn-off the ADC
	ADCSRA &= ~(1 << ADEN);

	// Return the conversion result
	return sumX;
}








//////////////////////////////////measureVcc///////////////////////////////////////
// From : http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
uint32_t measureVcc()
{
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference

	// determine the ADC channel for the internal reference voltage
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	byte muxVal = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	byte muxVal = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	byte muxVal = _BV(MUX3) | _BV(MUX2);
	#else
	byte muxVal = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif


	uint32_t result = (1 << nVcc) * 1173751L / readADC( muxVal, nVcc); // Calculate Vcc (in mV); estimated as intRef_Volts*1023*1000 = 1125300L
	return result; // Vcc in millivolts

}


//////////////////////////////////measureTemp///////////////////////////////////////
int16_t measureTemp()
{
	byte addr[8];
	int16_t tmp;

	if (ds.reset()) // check device is present
	{
		ds.skip();
		ds.write(0x44, 1);        // start conversion, with parasite power on at the end
		LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);     // maybe 750ms is enough, maybe not
		timer0_millis += 1000;
		// we might do a ds.depower() here, but the reset will take care of it.
		ds.reset();
		ds.skip();
		ds.write(0xBE);         // Read Scratchpad
		tmp = ds.read();
		tmp += (ds.read()<<8);


		//          for (uint8_t i = 0; i < 9; i++) // we need 9 bytes
		//          {
		//            data[i] = ds.read();
		//          }
		//          if (OneWire::crc8(data, 8) == data[8])
		//          {
		//            // Convert the data to actual temperature
		//            // because the result is a 16 bit signed integer, it should
		//            // be stored to an "int16_t" type, which is always 16 bits
		//            // even when compiled on a 32 bit processor.
		//            tmp = (data[1] << 8) | data[0];
		//            byte cfg = (data[4] & 0x60);
		//            // at lower res, the low bits are undefined, so let's zero them
		//            if (cfg == 0x00) tmp = tmp & ~7;  // 9 bit resolution, 93.75 ms
		//            else if (cfg == 0x20) tmp = tmp & ~3; // 10 bit res, 187.5 ms
		//            else if (cfg == 0x40) tmp = tmp & ~1; // 11 bit res, 375 ms
		//            // default is 12 bit resolution, 750 ms conversion time
		//            //  celsius = (float)tmp / 16.0;
		//            //  fahrenheit = celsius * 1.8 + 32.0;
		//          }
	} //if device present
	return(tmp);
}

//////////////////////////////////sendStatus///////////////////////////////////////
void sendStatus()
{
	//assemble packet
	payloadStatus.temp = measureTemp();
	payloadStatus.Vcc = measureVcc();
	payloadStatus.millisec = now;
	sendStatusPacket();
	#if USE_SER
	Serial.begin(115200);
	delay(1000);
	Serial.print(F("temp= "));
	Serial.print(payloadStatus.temp);
	Serial.print(F(" Vcc= "));
	Serial.print(payloadStatus.Vcc);
	Serial.print(F(" millissec= "));
	Serial.println(payloadStatus.millisec);
	Serial.flush();
	Serial.end();
	#endif

}



//////////////////////////////////loop///////////////////////////////////////
void loop()
{
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // wait to measure next port
	timer0_millis += 8000L;
	now = timer0_millis;
	if (now > nextStatusReport)
	{
		nextStatusReport = now + periodStatusReport;
		sendStatus();
	}
	if (now > nextOscReport)
	{
		// measure osc
		nextOscReport = now + (periodOscReport >> 1); // divide by 2 since report on alternate cycles so as to reduce polarisation


		// start measuring using power save

		// loop through coarse then fine measurement
		uint8_t j = (byte)coarse; // a global state variable seemed more elegant than a global index variable;
		digitalWrite(pinPower, HIGH); // power-up oscillator
		digitalWrite(pinPowerCounter, HIGH); // power-up counter board

		LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); // allow oscillator to stabilise before measuring frequency
		timer0_millis += 2000L;
		// To do:  check whether can shorten sleep to reduce power consumption

		set_sleep_mode(SLEEP_MODE_PWR_SAVE); // sleep while timer2 counts

		setPrescaler(preScalerSelect[j]);
		// enable INT1 interrupt on change
		EICRA = 0x0c;  // INT1 – rising edge on SCL
		EIMSK = 0x02;  // enable only int1

		uint32_t sumX = 0;
		uint32_t sumX2 = 0;
		uint32_t maxCounts = ( 1 << nCount ); // keep as powers of 2

		for (uint32_t i = 0; i < maxCounts; i++)
		{
			cli();
			sleep_enable();
			TCNT2 = 0; //reset Timer2 counter.  Not necessary?
			sei();
			sleep_cpu();
			//LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_ON, TIMER2_ON);
			uint8_t oldCount = count;
			sei();
			sleep_cpu();
			//LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_ON, TIMER2_ON);
			uint8_t newCount = count;
			// store result
			uint8_t delta = newCount - oldCount;
			sumX += delta;
			sumX2 += delta * delta;
		}
		sleep_disable();
		sei();
		EIMSK = 0x00;  // disable all external interrupts
		digitalWrite(pinPower, LOW); // power-down oscillator
		digitalWrite(pinPowerCounter, LOW); // power-down counter board


		// Calculate mean and variation

		meanResults[j] = sumX; // Store mean * n
		variationResults[j] = sumX2; // Require nCount =< 16 and count < 256, otherwise this will overflow
		if (nCount < 9)
		{
			variationResults[j] -= ((sumX * sumX) >> nCount); // Calculate variation with bitshift divide.  Store variation * (n - 1)
		}
		else
		{
			variationResults[j] -= ((sumX  >> (nCount >> 1)) * (sumX  >> (nCount - (nCount >> 1)))); // to avoid overflow
		}
		if (variationResults[j] & 0x80000000) variationResults[j] = 0;  // trap negative value of variance
		variationResults[j] = bitDiv(variationResults[j] << 4, nCount);  // report 16 * variance

		if (variationResults[j] & 0xffffff00) variationResults[j] = 0xff;  // trap large value of variance; require variation less than 256


		if (!coarse) // Calculate final results and send
		{
			// merge the coarse and fine measurements into finalCount giving precedence to bits from the fine measurement
			int8_t m = preScaler[preScalerSelect[0]] - preScaler[preScalerSelect[1]];
			uint32_t finalCount = meanResults[0] << m;  // shift the coarse measurement to the left
			finalCount &= (0xffffffff << (8 + nCount)); // clear the last 8 + nCount bits of the coarse measurement
			finalCount |= meanResults[1];  // merge

			/*
			locate the binary point for the count:  8 + nCount bits in total; the binary point is to the right of the nCount bit
			with enumeration of bits 0, 1, 2, 3 . . .
			*/

			//to convert to microsec: divide count by clockFrequency; divide by nCount; multiply by prescaler
			payloadTime.bin2usCoarse = clockFrequency +  nCount - preScaler[preScalerSelect[0]];
			payloadTime.bin2usFine = clockFrequency +  nCount - preScaler[preScalerSelect[1]];

			uint32_t time2Measure = meanResults[0] << (preScaler[preScalerSelect[0]] - clockFrequency + 1);// approximate estimate in usec; +1 since omit alternate cycles
			timer0_millis += roundDiv(time2Measure , 1000L);

			#if USE_SER

			//print result
			Serial.begin(115200);
			delay(1000); // no need for sleep to save power since running serial

			for (uint8_t i = 0; i < 2; i++)
			{
				Serial.print(F("Variation= "));
				Serial.print(variationResults[i]);
				Serial.print(F(" "));
			}

			/*
			Converting binary fractions to decimal ones
			power of 10 determines number of decimal places
			n decimal ~ m binary * log10(2)
			divide number of binary places by 3 to find power of 10
			*/

			//First print coarse measurement
			int8_t convert2Microsec = payloadTime.bin2usCoarse;

			Serial.print(F("time= "));
			Serial.print(timer0_millis);
			Serial.print(F(" finalCoarse= "));
			Serial.print(meanResults[0] >> convert2Microsec);
			Serial.print(F("."));
			Serial.print(((meanResults[0] & ((1L << convert2Microsec) - 1L)) * 100) >> convert2Microsec);

			//Now for the fine measurement
			convert2Microsec = payloadTime.bin2usFine;

			Serial.print(F(" finalHex= "));
			Serial.print(finalCount, HEX);
			Serial.print(F(" final= "));
			Serial.print(finalCount >> convert2Microsec);
			Serial.print(F("."));
			Serial.println(((finalCount & ((1L << convert2Microsec) - 1L)) * 10000) >> convert2Microsec);
			Serial.flush();
			Serial.end();




			#endif // USE_SER


			// assemble packet
			// with the original measurements and the positions of the binary point to convert them to microsec
			// 14 bytes in packet
			payloadTime.nodeId = (node | ((byte)measureOsc1 << 4)); // bit 5
			payloadTime.varCoarse = variationResults[0];
			payloadTime.varFine = variationResults[1];
			payloadTime.coarseTime = meanResults[0];
			payloadTime.fineTime = finalCount;
			sendTimePacket();








			// switch pins ready for next measurement
			measureOsc1 = !measureOsc1;
			pinPower = (measureOsc1 ? pinPower1 : pinPower2);
			if (measureOsc1)
			{
				periodOscReport += 30000L;
				if (periodOscReport > 240000L) periodOscReport = 30000L;
			}
		}
		coarse = !coarse; // alternate coarse and fine
	} // end measureOsc

} // end loop()


//////////////////////////////////end///////////////////////////////////////
