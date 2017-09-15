#include "BSPs/ATTiny24a_bsp.h"

#include <stdlib.h> // abs()
#include <math.h>   // round()

#include "avr/io.h"
#include "util/delay.h"

#include "avr/sleep.h"
#include "avr/wdt.h"
#include <avr/interrupt.h>
#include "delayUs.h"

#include <avr/eeprom.h>

#define SET_BIT(port, bit) do { (port) |= (1UL << (bit)); } while(0)
#define CLR_BIT(port, bit) do { (port) &= ~(1UL << (bit)); } while(0)

#define ACTIVATE_433		SET_BIT(PORTA,PORTA2)
#define DEACTIVATE_433		CLR_BIT(PORTA,PORTA2)

#define ACTIVATE_LM35		SET_BIT(PORTB,PORTB0)
#define DEACTIVATE_LM35  	CLR_BIT(PORTB,PORTB0)

#define RADIO_HIGH			SET_BIT(PORTA,PORTA3)
#define RADIO_LOW			CLR_BIT(PORTA,PORTA3)

#define LED_ON				SET_BIT(PORTA,PORTA7)
#define LED_OFF				CLR_BIT(PORTA,PORTA7)

#define TEMP_SENSOR_ID		0
#define VOLTAGE_SENSOR_ID	1
#define DEVICE_ID			0x01

#define TEMP_SAMPLE_COUNT 10


volatile uint8_t sleep_interval=0;

// the sync pulse amount for transmitting and receiving.
// a pulse means : HI,LO   or  LO,HI
// usually SYNC_PULSE_MAX >= SYNC_PULSE_DEF + 2
//         SYNC_PULSE_MIN <= SYNC_PULSE_DEF + 2
//  consider the pulses rising when starting transmitting.
//  SYNC_PULSE_MIN should be much less than SYNC_PULSE_DEF
//  all maximum of 255
#define     SYNC_PULSE_MIN  1
#define     SYNC_PULSE_DEF  3
#define     SYNC_PULSE_MAX  5
#define		DECOUPLING_MASK 0b11001010
/*
//setup timing for receiver
#define MinCount        33  //pulse lower count limit on capture
#define MaxCount        65  //pulse higher count limit on capture
#define MinLongCount    66  //pulse lower count on double pulse
#define MaxLongCount    129 //pulse higher count on double pulse
*/

/*
Signal timing, we take sample every 8 clock ticks

ticks:   [0]-[8]--[16]-[24]-[32]-[40]-[48]-[56]-[64]-[72]-[80]-[88]-[96][104][112][120][128][136]
samples: |----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|----|
single:  |                    [--------|----------]
double:  |                                         [-----------------|--------------------]
signal:  |_____________________________                               ______________________
|                                      |_____________________________|

*/

// setup for 4800bps
//
// sampling each 32ms
// pulse duration is 32ms * 6 = 192ms. its also 3072/4², also (HALF_BIT_INTERVAL >> speedFactor)
// various delays :
// sendZero = 2 instructions + delay1 + 3 instructions + delay2 + 1
// sendOne  = 2 instructions + delay1 + 3 instructions + delay2 + 1

uint8_t	delay1 = 1;
uint8_t	delay2 = 1;

void sendZero(void)
{
	delayMicroseconds(delay1);
	RADIO_HIGH;
	
	delayMicroseconds(delay2);
	RADIO_LOW;

}


void sendOne(void)
{
	delayMicroseconds(delay1);
	RADIO_LOW;
	
	delayMicroseconds(delay2);
	RADIO_HIGH;
}



ISR(WDT_vect)
{
	wdt_reset();
	sleep_interval++;
	// Re-enable WDT interrupt
	_WD_CONTROL_REG |= (1<<WDIE);
}



// Enable watchdog interrupt, set prescaling to 1 sec
void init_wdt()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// Disable interrupts
	cli();
	
	MCUSR &= ~(1 << WDRF);
	
	// Start timed sequence
	// Set Watchdog Change Enable bit
	_WD_CONTROL_REG |= (1<<WDCE) | (1<<WDE);

	// Set new prescaler (1 sec), unset reset enable
	// enable WDT interrupt
	//_WD_CONTROL_REG = (1<<WDP3)|(1<<WDP0); // 8 sec
	_WD_CONTROL_REG = (1<<WDP1)|(1<<WDP2); // 1 sec
	_WD_CONTROL_REG |= (1 << WDIE);
	sei();
}

// Puts MCU to sleep for specified number of seconds using
// WDT to wake every second and track number of seconds
void sleep(uint8_t s)
{
	s /= 8;
	if(s == 0 ) s = 1;
	sleep_interval = 0;
	while (sleep_interval < s) {
		sleep_mode();
		sleep_disable();
	}
}
/*
The 433.92 Mhz receivers have AGC, if no signal is present the gain will be set
to its highest level.

In this condition it will switch high to low at random intervals due to input noise.
A CRO connected to the data line looks like 433.92 is full of transmissions.

Any ASK transmission method must first sent a capture signal of 101010........
When the receiver has adjusted its AGC to the required level for the transmission
the actual data transmission can occur.

We send 14 0's 1010... It takes 1 to 3 10's for the receiver to adjust to
the transmit level.

The receiver waits until we have at least 10 10's and then a start pulse 01.
The receiver is then operating correctly and we have locked onto the transmission.
*/
void transmitArray(uint8_t numBytes, uint8_t *data)
{

#if SYNC_BIT_VALUE
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++) //send capture pulses
  {
    sendOne(); //end of capture pulses
  }
  sendZero(); //start data pulse
#else
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++) //send capture pulses
  {
    sendZero(); //end of capture pulses
  }
  sendOne(); //start data pulse
#endif
 
  // Send the user data
  for (uint8_t i = 0; i < numBytes; i++)
  {
    uint16_t mask = 0x01; //mask to send bits
    uint8_t d = data[i] ^ DECOUPLING_MASK;
    for (uint8_t j = 0; j < 8; j++)
    {
      if ((d & mask) == 0)
        sendZero();
      else
        sendOne();
      mask <<= 1; //get next bit
    }//end of byte
  }//end of data

  // Send 3 terminatings 0's to correctly terminate the previous bit and to turn the transmitter off
#if SYNC_BIT_VALUE
  sendOne();
  sendOne();
  sendOne();
#else
  sendZero();
  sendZero();
  sendZero();
#endif
}//end of send the data


void ATTiny24a_bsp::Setup()
{
	cli();
	
	//delay1 = eeprom_read_byte((uint8_t*)1);
	//delay2 = eeprom_read_byte((uint8_t*)2);
	
	// Comparator is off
	ACSR = 0;

	// disable input buffer for unused ADCs	
	DIDR0 = (1<<ADC7D) | (1<<ADC6D) |(1<<ADC5D) |(1<<ADC4D) |(1<<ADC3D) |(1<<ADC2D);

	// ADC Enable and prescaler of 128 (7Khz..)
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	// ADC Enable and prescaler of 8 (125Khz)
	//ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);

	// ADC Enable and prescaler of 16 (62.5Khz)
	//ADCSRA = (1<<ADEN)|(1<<ADPS2);
	
	// define PA7 as output pin (control LED)
	// define PA2 as output pin (periph control)
	// define PA3 as output pin (433 tx)
	//        76543210
	DDRA  |= (1<<DDA7)|(1 << DDA2)|(1<< DDA3);
	
	//        76543210
	//PORTA = 0b00000000;
	
	// define PB0 as output pin (sensor vcc)
	// define PB1 as output pin (external signal)
	//       76543210
	DDRB |= (1 << DDB0) | (1 << DDB1);
	//
	//PORTB = 0b0000000;

	sei();
	
	// shutdown external signal
	_delay_ms(100);
	SET_BIT(PORTB,PORTB1);
	
	/*
	PB0 : Vcc for LM35
	PB1 : floating
	PB2 : floating
	PB3 : reset
	
	PA0 : Sensor value (analog)
	PA1 : sensor ground reference (analog)
	PA2 : peripherals ground
	PA3 : 433 signal
	PA4 : SCK
	PA5 : MISO
	PA6 : MOSI
	PA7 : LED

	*/
	
	init_wdt();

	//Serial.begin(4800);   // set the highest standard baud rate of 115200 bps (mini pro uses x2)
}


void ATTiny24a_bsp::SenseAndSave()
{
/*
	uint16_t sensorVoltage = 0;
	uint16_t refVoltage = 0;
	uint8_t admux = 0;

	// 1. Read the actual Vcc
	
	// internal 1.1V reference against AVcc
	admux = (1 << MUX5) | (1 << MUX0); 
	float vcc = ADCRead(admux);
	// Back-calculate AVcc in mV
	vcc = 1125.3 / vcc; // 1023 * 1.1
	
	// ADC0 against Vcc
	admux = 0;
	sensorVoltage = ADCRead(admux);

	// ADC1 against Vcc
	admux = (1 << MUX0); 
	refVoltage = ADCRead(admux);
	float temperature_celcius = (sensorVoltage - refVoltage) * ( vcc / 1023.0); // last *10 to allow integer/decimal separation below
	
	bool isNegative = (temperature_celcius < 0);

	uint16_t temperature_celcius2 = abs(round(temperature_celcius));
	uint8_t  integer = temperature_celcius2/10; // ie 26 in this sample
	uint8_t  fractional = temperature_celcius2 % 10; // ie 4 in this sample
*/
	SENSOR_DATA[TEMP_SENSOR_ID][MESSAGE_SIZE_POS] = MESSAGE_SIZE;

	SENSOR_DATA[TEMP_SENSOR_ID][DEV_ID_POS]    = DEVICE_ID;
	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_ID_POS] = TEMP_SENSOR_ID;

	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_1_POS] = delay1;
	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_2_POS] = delay2;
/*
	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_1_POS] = integer;
	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_2_POS] = fractional;
	SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_3_POS] = (isNegative) ? 1 : 0;


	temperature_celcius2 = abs(round(vcc*100));
	integer = temperature_celcius2/100; // ie 26 in this sample
	fractional = temperature_celcius2 % 100; // ie 4 in this sample
	
	SENSOR_DATA[VOLTAGE_SENSOR_ID][MESSAGE_SIZE_POS] = MESSAGE_SIZE;
	SENSOR_DATA[VOLTAGE_SENSOR_ID][DEV_ID_POS]    = DEVICE_ID;
	SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_ID_POS] = VOLTAGE_SENSOR_ID;
	SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_BYTE_1_POS] = integer;
	SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_BYTE_2_POS] = fractional;
*/
}

void ATTiny24a_bsp::PrintInfo(){}


uint16_t ATTiny24a_bsp::ADCRead(uint8_t admux)
{
	ADMUX = admux;
	
	// first discarded read
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	
	return ADC;
}

void ATTiny24a_bsp::ActivatePeripherals()
{
	LED_ON;
	ACTIVATE_433;

	// ADC Enable
	ADCSRA |= (1<<ADEN);
	
	// http://www.ti.com/lit/ds/symlink/lm35.pdf p.12
	// (LM35 startup response)
	ACTIVATE_LM35;
	_delay_us(30);

}

/*!
* Cut power of the peripherals : sensors, 433Mhz emitter
*/
void ATTiny24a_bsp::DeactivatePeripherals()
{
	ADCSRA &= ~(1<<ADEN);
	
	DEACTIVATE_433;
	DEACTIVATE_LM35;
	LED_OFF;
}

void ATTiny24a_bsp::Sleep()
{
	//sleep(8);
	_delay_ms(100);

}

/*!
* Send data in internal storage
*/
void ATTiny24a_bsp::SendData433()
{
	// temperature
	transmitArray(SENSOR_DATA[0][0], SENSOR_DATA[0]);
	_delay_ms(30);

	transmitArray(SENSOR_DATA[0][0], SENSOR_DATA[0]);
	_delay_ms(30);

	transmitArray(SENSOR_DATA[0][0], SENSOR_DATA[0]);
	_delay_ms(30);
	
	if(PINB & (0x01 << 2))
	{
		eeprom_write_byte((uint8_t*)1, delay1);
		eeprom_write_byte((uint8_t*)2, delay2);
		// good timing. keep green
		SET_BIT(PORTB,PORTB1);
	}
	else
	{
	
		if(delay1 < 254)
		{
			if(delay2 < 254)
			{
				delay2++;	
			}
			else
			{
				delay2 = 1;
				delay1++;
			}
			
		}
		else
		{
			// bad news. no synchro found
			delay1 = 1;
			delay2 = 1;
		}
	}
	
	// voltage
	//transmitArray(SENSOR_DATA[1]);

}
