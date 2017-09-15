/*
 * IncFile1.h
 *
 * Created: 17/08/2017 00:13:41
 *  Author: aurelien
 */ 


#ifndef SerialBridge_H_
#define SerialBridge_H_


class __FlashStringHelper;

class SerialBridge
{
	public:

	void begin(uint16_t speed);
	
	void print(int value);
	void println(int value);
	
	void println(uint16_t value);
	void print(uint16_t value);
	
	void print(uint8_t value);
	void println(uint8_t value);
	
	void print(const __FlashStringHelper*);
	void println(const __FlashStringHelper*);
	
	void print(const char *);
	void println(const char *);

};



#endif /* INCFILE1_H_ */