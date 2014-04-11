#include <Wire.h>
#include <DS3231.h>
#include <avr/sleep.h>

DS3231 RTC;
bool ledOnFlag = false;

void setup()
{

	PORTD |= 0x04;
	DDRD &=~ 0x04;
	

	// Establish comms
	Serial.begin(57600);
	Wire.begin();
	RTC.begin();

set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_enable();

	// Start interrupts
	attachInterrupt(0, periodicISR, FALLING);
	RTC.enableInterrupts(EveryMinute);
}

void loop()
{
	RTC.clearINTStatus();
	sleep_mode();
	Serial.println("Bep");
	delay(100);
	
	

}

void periodicISR(){
	Serial.write("Beep");
}
