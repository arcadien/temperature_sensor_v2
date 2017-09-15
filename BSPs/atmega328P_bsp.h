#ifndef ATMEGA328P_HAL_H
#define ATMEGA328P_HAL_H

#include <SensorV2_API.h> // integer types

// Should be defined by -D command line flag
#ifndef F_CPU
# warning "F_CPU was not set on the command line, force 1Mhz"
#define		F_CPU 1000000L
#endif

#include "SerialBridge.h"
SerialBridge Serial;

class ATMega328P_bsp : public SensorV2_API
{
	public:

	/*!
	* Hardware initial setup
	*/
	void Setup() ; // override

	/*!
	*
	* Display information about the device
	*
	*/
	void PrintInfo() ; // override


	/*!
	*
	* Return voltage on Vcc, millivolts
	*
	*/
	uint16_t ReadVcc() ; // override

	/*!
	* Bring power to the peripherals : sensors, 433Mhz emitter
	*/
	void ActivatePeripherals() ; // override


	/*!
	* Cut power of the peripherals : sensors, 433Mhz emitter
	*/
	void DeactivatePeripherals() ; // override

	/*!
	* Read voltage at LM35 output pin
	*/
	uint16_t AnalogReadLM35() ; // override

	/*!
	* Read voltage at LM35 ground pin, before level shifter module
	*/
	uint16_t AnalogReadRef() ; // override

	/*!
	* Store a temperature sample in internal storage
	*/
	void StoreTempData(bool isNegative, uint8_t integer, uint8_t fractional) ; // override

};

#endif
