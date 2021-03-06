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
v8 distribute soil probe measurements uniformly
improve the estimate of measurement time

adds overrun interrupt for TIMER2
replaces INT1 with direct monitoring of D3
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

const int pinOutputCounter = 3;
const int pinPowerCounter = 5;
const int pinPowerOsc1 = 6;
const int pinPowerOsc2 = 7;
const int pinLed = 13;
const byte receive_pin = -1;
const byte transmit_pin = 12;
const byte digT_pin = 8;



uint8_t preScaler[8] = {0, 0, 3, 5, 6, 7, 8, 10}; // powers of 2
const uint8_t clockFrequency = 4; // i.e. 2^4 in MHz
uint8_t preScalerSelect = 1; // i.e.divide by 1
const uint8_t nCount = 8; // number to average is 2^nCount

int pinPower = pinPowerOsc1;
bool measureOsc1 = true; // first measurement is for osc1
bool initialized = false; // when false can use for debug/learn
// store results here
uint32_t meanResults;



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
uint32_t periodOscReport = 120000L; // changed to check effect of polarisation
uint32_t nextStatusReport = periodStatusReport;
uint32_t nextOscReport = periodOscReport;
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
	delay(1);
	Serial.print(F("temp= "));
	Serial.print(payloadStatus.temp);
	Serial.print(F(" Vcc= "));
	Serial.print(payloadStatus.Vcc);
	Serial.print(F(" millisec= "));
	Serial.println(payloadStatus.millisec);
	Serial.flush();
	Serial.end();
	#endif
	
}


//////////////////////////////////setup///////////////////////////////////////

void setup()
{


	//pinMode(pinOutputCounter, INPUT);
	//pinMode(pinPowerOsc1, OUTPUT);
	//pinMode(pinPowerOsc2, OUTPUT);
	//pinMode(pinPowerCounter, OUTPUT);
	//pinMode(pinLed, OUTPUT);
	DDRB &= 0b11110000; // pins 8, 9, 10 and 11 inputs
	PORTB = 0b00001111; // pull-up pins 8 to 11
	DDRB |= (1 << DDB5); // pin13 output
	DDRD &= 0b11100011;  // pin 2, 3, 4 inputs
	PORTD = 0b00010100; // pull-up pins 2 and 4
	DDRD |= ((1 << DDD5) | (1 << DDD6) | (1 << DDD7)); //pins 5, 6 and 7 outputs

	/*
	const int pinOutputCounter = 3;
	const int pinPowerCounter = 5;
	const int pinPowerOsc1 = 6;
	const int pinPowerOsc2 = 7;
	const int pinLed = 13;
	const byte receive_pin = -1;
	const byte transmit_pin = 12;
	const byte digT_pin = 8;
	*/

	// turn-off the ADC
	ADCSRA &= ~(1 << ADEN);
	
	// turn-off timer1
	TIMSK1 = 0;

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


	payloadTime.count = 0;
	payloadTime.nodeId = node;
	payloadStatus.count = 0;
	payloadStatus.nodeId = node | 0x20; // bit 5 set to indicate status packet
	payloadStatus.empty = 0L;

	flashLED(3);
	
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
		sendStatus();
	}
	if (now > nextOscReport)
	{
		// measure osc
		nextOscReport = now + periodOscReport;


		// start measuring using power save
		
		//digitalWrite(pinPower, HIGH); // power-up oscillator
		//digitalWrite(pinPowerCounter, HIGH); // power-up counter board
		PORTD ^= ((1 << DDD5) | (1 << pinPower));
		
		LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); // allow oscillator to stabilise before measuring frequency
		
		advanceTimer(2000L);
		// To do:  check whether can shorten sleep to reduce power consumption

		uint16_t maxCounts = ( 1 << nCount ); // keep as powers of 2
		uint32_t result = 0L;
		overFlowCount = 0;
		byte mask = (1 << PIND3);
		byte state = PIND & mask;
		
		while((PIND & mask) == state) {;;} // wait for transition
		TCNT2 = 0; //reset Timer2 counter
		TIMSK2 |= (1 << TOIE2);  // enable overflow interrupt
		TIMSK0 = 0; // turn off timer0 for lower jitter.  Note disables millis()
		// assume that timer1 is not enabled
		
		
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
			result++; //force the overflow count to increment
			TIFR2 |= (1 << TOV2);	// the flag is cleared by writing a logical one to it
		}
		TIMSK2 &= ~(1 << TOIE2);  // mask overflow interrupt
		sei();
		TIMSK0 = 1; // turn timer0 back on
		result = result << 8;
		result |= count;
		
		

		//digitalWrite(pinPower, LOW); // power-down oscillator
		//digitalWrite(pinPowerCounter, LOW); // power-down counter board
		PORTD ^= ((1 << DDD5) | (1 << pinPower));

		/*
		locate the binary point for the count:  8 + nCount bits in total; the binary point is to the right of the nCount bit
		with enumeration of bits 0, 1, 2, 3 . . .
		*/
		
		//to convert to microsec: divide count by clockFrequency; divide by nCount; multiply by prescaler
		payloadTime.bin2usCoarse = clockFrequency +  nCount - preScaler[preScalerSelect];
		payloadTime.bin2usFine = 0;
		
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

		
		Serial.print(F("time= "));
		Serial.print(millis());
		Serial.print(F(" finalCoarse= "));
		Serial.print(result >> payloadTime.bin2usCoarse);
		Serial.print(F("."));
		Serial.println(((result & ((1L << payloadTime.bin2usCoarse) - 1L)) * 100) >> payloadTime.bin2usCoarse);
		Serial.flush();
		Serial.end();
		
		#endif // USE_SER
		
		
		// assemble packet
		// with the original measurements and the positions of the binary point to convert them to microsec
		// 14 bytes in packet
		payloadTime.nodeId = (node | ((byte)measureOsc1 << 4)); // bit 5
		payloadTime.varCoarse = 0;
		payloadTime.varFine = 0;
		payloadTime.coarseTime = result;
		payloadTime.fineTime = 0;
		sendTimePacket();
		
		





		
		// switch pins ready for next measurement
		measureOsc1 = !measureOsc1;
		pinPower = (measureOsc1 ? pinPowerOsc1 : pinPowerOsc2);
	} // end measureOsc
	
} // end loop()


//////////////////////////////////end///////////////////////////////////////




