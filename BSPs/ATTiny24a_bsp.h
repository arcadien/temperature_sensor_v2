#ifndef ATTINY24A_HAL_H
#define ATTINY24A_HAL_H

#include <stdint.h>
#include "protocol.h"

// Should be defined by -D command line flag
#ifndef F_CPU
# warning "F_CPU was not set on the command line, force 1Mhz (8M osc and /8 divider)"
#define		F_CPU 8000000UL
#endif

//#include "SerialBridge.h"
//SerialBridge Serial;

#define SENSOR_COUNT        2

class ATTiny24a_bsp
{
public:
	void Sleep();

	void Setup() ;

	void PrintInfo() ; 

	void ActivatePeripherals() ; 

	void DeactivatePeripherals() ;
	
	void SenseAndSave();
	
	void SendData433(); 
	
	uint16_t ADCRead(uint8_t admux);

private:

	uint8_t SENSOR_DATA[SENSOR_COUNT][MESSAGE_SIZE];

};


#endif
