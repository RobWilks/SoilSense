/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */



/*


forked from sketchV10
adds overrun interrupt for TIMER2
replaces INT1 with direct monitoring of D3

v2 modified for pin out of MC1648 board
wire pins 8 and 9 together to boost output current
circuit uses direct power from ports which is not recommended practice!

v3 combines status and freq count packets
*/

#include <OneWire.h>
// use RadioHead to transmit messages
// with a simple ASK transmitter
// Implements a simplex (one-way) transmitter with an TX-C1 module

#include <RH_ASK.h>
#include <LowPower.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#define INC_COUNT() overFlowCount += 1
#define USE_SER 0






//Note: an object of this class was already pre-instantiated in the .cpp file of this library, so you can simply access its methods (functions)
//      directly now through the object name "timer2"
//eRCaGuy_Timer2_Counter timer2;  //this is what the pre-instantiation line from the .cpp file looks like

volatile uint32_t overFlowCount;

/*
pinOutputCounter = 3;
pinPowerCounter = 10;
pinPowerOsc = 9;
pinLed = 13;
*/
const byte mask = (1 << PIND3); //mask for input pin to frequency counter
const byte receive_pin = -1;
const byte transmit_pin = 12;
const byte digT_pin = 11;



uint8_t preScaler[8] = {0, 0, 3, 5, 6, 7, 8, 10}; // powers of 2
const uint8_t clockFrequency = 4; // i.e. 2^4 in MHz
uint8_t preScalerSelect = 1; // i.e.divide by 1
const uint8_t nCount = 8; // number to average is 2^nCount
const uint8_t convert2Microsec = clockFrequency + nCount - preScaler[preScalerSelect];
//to convert to microsec: divide count by clockFrequency; divide by nCount; multiply by prescaler

bool initialized = false; // when false can use for debug/learn
// store results here
uint32_t meanResults;



const int baud = 500;
const byte node = 13;

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
const uint32_t periodMeasurement = 120000L; // changed to check effect of polarisation
uint32_t nextStatusReport = 300000L;
uint32_t nextMeasurement = periodMeasurement;
uint32_t now = 0L; // beginning of time


uint32_t roundDiv(uint32_t a, uint32_t b);
uint32_t bitDiv(uint32_t a, byte b);

////////////////////////////////////struct PayloadItem/////////////////////////////////////
typedef struct {
	byte nodeId; //store this nodeId
	byte count;
	int temp;
	int Vcc;
	unsigned long timeStamp;
	unsigned long periodOfSignal;
} PayloadItem;
// count keeps track of the # packets sent out

PayloadItem payload;

//////////////////////////////////Initialise the ISR///////////////////////////////////////

ISR(TIMER2_OVF_vect) //Timer2's counter has overflowed
{
	INC_COUNT();
}

//////////////////////////////////advanceTimer0///////////////////////////////////////
void advanceTimer0(uint32_t deltaTime)
{
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	timer0_millis += deltaTime;
	SREG = oldSREG;
}

//////////////////////////////////advanceTimer///////////////////////////////////////
void advanceTimer(uint32_t deltaTime)
{
	extern volatile unsigned long timer0_millis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		timer0_millis += deltaTime;
	}
	
}

//////////////////////////////////flashLED///////////////////////////////////////
void flashLED(byte nTimes)
{
	for (uint8_t i = 0; i < nTimes; i++)
	{
		LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
		//digitalWrite(pinLed, HIGH);
		PORTB ^= (1 << PB5);
		LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
		//digitalWrite(pinLed, LOW);
		PORTB ^= (1 << PB5);
	}

}
//////////////////////////////////sendPacket///////////////////////////////////////

void sendPacket()
{
	// send packet
	driver.send((uint8_t*)(&payload), sizeof(payload));
	driver.waitPacketSent();
	++payload.count;
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
	TIMSK0 = 1; // turn timer0 back on
	ADCSRA &= ~(1 << ADEN);// turn-off the ADC
	uint32_t noCycles = (3 << (noSamples - 1)) * 13L + 12L; //prescale factor x (number of measurements x 13.5 + 12)
	noCycles *= (1 << (ADCSRA & 0x07)); // multiply by prescaler
	uint32_t missedTime = roundDiv((noCycles >> 4) , 1000L) + 20L;// divide by clock frequency; convert to msec; add time for settling
	advanceTimer(missedTime);

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


	uint32_t result = (1 << nVcc) * 1119483L / readADC( muxVal, nVcc); // Calculate Vcc (in mV); estimated as intRef_Volts*1023*1000 = 1125300L
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
		advanceTimer(1000L);
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



//////////////////////////////////setup///////////////////////////////////////

void setup()
{


	
	// PORTB maps pins 8 to 13 of Pro Mini
	// PORTD maps Rxd, Txd ,2, 3, 4, 5, 6, 7 of Pro Mini
	
	DDRB &= 0b11110111; // pin 11 input
	PORTB = 0b00001000; // pull-up pin 11
	DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB5); // pins 8, 9, 10, 13 outputs
	DDRD &= 0b00000011;  // pins 2, 3, 4, 5, 6, 7 inputs
	PORTD = 0b11110100; // pull-up all pins except Rxd, Txd, 3

	/*
	pinOutputCounter = 3;
	pinPowerCounter = 10;
	pinPowerOsc = 9;
	pinLed = 13;
	const byte receive_pin = -1;
	const byte transmit_pin = 12;
	const byte digT_pin = 8;
	*/

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
	setPrescaler(preScalerSelect);

	
	// Initialise the IO
	if (!driver.init())
	{
		
		flashLED(10);
		while (true) {
			;; // do nothing forever
		}
	}


	payload.count = 0;
	payload.nodeId = node | (convert2Microsec << 4);

	flashLED(7);
	
	#if USE_SER

	//print result
	Serial.begin(115200);
	delay(1);
	if (!initialized)
	{
		Serial.println("Initialised");
		initialized = true;
		Serial.flush();
		Serial.end();
	}
	#endif

}

//////////////////////////////////loop///////////////////////////////////////
void loop()
{
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // wait to measure next port
	advanceTimer(8000L);
	now = millis();
	if (now > nextStatusReport)
	{
		nextStatusReport = now + periodStatusReport;
		payload.temp = measureTemp();
		payload.Vcc = measureVcc();
		//will send when new frequency data are available
	}
	if (now > nextMeasurement)
	{
		// measurePeriodOfSignal
		nextMeasurement = now + periodMeasurement;


		// start measuring using power save
		
		// power-up oscillator
		// power-up counter board
		PORTB ^= ((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (USE_SER << PORTB5));
		
		LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF); // allow oscillator to stabilise before measuring frequency
		
		advanceTimer(500L);
		// To do:  check whether can shorten sleep to reduce power consumption

		uint16_t maxCounts = ( 1 << nCount ); // keep as powers of 2
		uint32_t result = 0L;
		overFlowCount = 0; // volatile
		//any interrupt will affect the accuracy of TIMER2 count but cannot screen all interrupts so mask individually
		TIMSK0 = 0; // turn off timer0.  Used by millis()
		byte old_TIMSK1 = TIMSK1;
		TIMSK1 = 0; // turn off timer1. Used by RH_ASK
		byte old_EIMSK = EIMSK;
		EIMSK = 0; // no external interrupts

		byte state = PIND & mask;
		while((PIND & mask) == state) {;;} // wait for transition
		TCNT2 = 0; //reset Timer2 counter
		TIMSK2 |= (1 << TOIE2);  // enable overflow interrupt
		
		
		for (uint16_t i = 0; i < maxCounts; i++)
		{
			while((PIND & mask) != state) {;;} // wait for transition
			while((PIND & mask) == state) {;;} // wait for transition
		}
		cli();
		byte count = TCNT2;
		result = overFlowCount;
		if( TIFR2 & (1 << TOV2)); //grab the timer2 overflow flag value; see datasheet pg. 160
		{
			result++; //increment the overflow count
			TIFR2 |= (1 << TOV2);	// the flag is cleared by writing a logical one to it
		}
		TIMSK2 &= ~(1 << TOIE2);  // mask overflow interrupt; ensures no overflow interrupt can occur at the moment the counter is reset
		sei();
		// reset all other interrupt masks
		TIMSK0 = 1; // turn timer0 back on
		TIMSK1 = old_TIMSK1;
		EIMSK = old_EIMSK;
		result = result << 8;
		result |= count;
		
		

		// power-down oscillator
		// power-down counter board
		PORTB ^= ((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (USE_SER << PORTB5));

		/*
		locate the binary point for the count:  8 + nCount bits in total; the binary point is to the right of the nCount bit
		with enumeration of bits 0, 1, 2, 3 . . .
		*/
		
		
		uint32_t time2Measure = result << (preScaler[preScalerSelect] - clockFrequency);// approximate estimate in usec
		advanceTimer(roundDiv(time2Measure , 1000L));

		#if USE_SER

		//print result
		Serial.begin(115200);
		delay(1); // no need for sleep to save power since running serial
		
		/*
		Converting binary fractions to decimal ones
		power of 10 determines number of decimal places
		n decimal ~ m binary * log10(2)
		divide number of binary places by 3 to find power of 10
		*/

		Serial.print(F("temp= "));
		Serial.print(payload.temp);
		Serial.print(F(" Vcc= "));
		Serial.print(payload.Vcc);
		Serial.print(F(" timestamp= "));
		Serial.print(millis());
		Serial.print(F(" period= "));
		Serial.print(result >> convert2Microsec);
		Serial.print(F("."));
		Serial.println(((result & ((1L << convert2Microsec) - 1L)) * 100) >> convert2Microsec);
		Serial.flush();
		Serial.end();
		
		#endif // USE_SER
		
		
		// assemble packet
		// with the original measurements and the positions of the binary point to convert them to microsec
		// 14 bytes in packet
		payload.timeStamp = millis();
		payload.periodOfSignal = result;
		sendPacket();
		
		
	} // end measurePeriodOfSignal
	
} // end loop()


//////////////////////////////////end///////////////////////////////////////




