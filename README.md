# Temperature sensor v2
Second temperature sensor implementation for a ATTiny24a + LM35 + low cost 433Mhz transmitter.
This version targets a very constrained device, the [ATTiny24a](http://www.microchip.com/wwwproducts/en/ATtiny24A), whis has only 2k of flash.
The source code does not use the Arduino framework, nor the QP framework as in the first version.
I used Atmel Studio to develop this version.
All the ADC/Sensor part works. The emmiter does not transmit information i was able to receive with a 433Mhz receiver. There is a problem with the Manchester timings : the third version is there to try to bruteforce them.
