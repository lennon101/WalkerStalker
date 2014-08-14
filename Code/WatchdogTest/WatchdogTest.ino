#include "avr/wdt.h"

void setup()
{
	wdt_reset();
	wdt_disable();

	Serial.begin(57600);
	delay(1000);
	
	Serial.println("Watchdog test - Start");
	
	wdt_enable(WDTO_2S);
	wdt_reset();
}

void loop()
{
	wdt_reset();
	Serial.println("Dog fed");
	
	delay(500);
	
	if (Serial.available()){
		delay(2500);
	}
}
