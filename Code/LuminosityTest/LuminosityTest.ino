#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_TSL2561_U.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void setup()
{
	Serial.begin(9600);
	Wire.begin();
}

void loop()
{

	sensors_event_t event;
	tsl.getEvent(&event);
	
	Serial.print("TSL - Lux: ");
	Serial.println(event.light);
	
	int ldrReading = analogRead(1);
	Serial.print("LDR: ");
	Serial.println(ldrReading);
	
	delay(500);

}
