#include <Wire.h>
#include <DS3231.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSOR_PIN 9
OneWire oneWire(SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

DS3231 RTC;

unsigned long startTime;
unsigned long conversionTime;

void setup()
{
	Wire.begin();
	
	Serial.begin(57600);
	Serial.println("Temperature test");

	tempSensor.begin();
	RTC.begin();
}

void loop()
{
	startTime = millis();
	
	// Send requests early
	tempSensor.requestTemperatures();
	RTC.convertTemperature();
	
	// Get all recorded values	
	float dsTemp1 = tempSensor.getTempCByIndex(0);
	float dsTemp2 = tempSensor.getTempCByIndex(1);
	float rtcTemp = RTC.getTemperature();
	
	// Stop the clock
	conversionTime = millis() - startTime;
	Serial.print("Conversion Time: ");
	Serial.print(conversionTime, DEC);
	Serial.println(" ms");
	
	// Print all the things
	Serial.print("1) DS18B20: ");
	Serial.print(dsTemp1, 2);
	Serial.println(" C");
	
	Serial.print("2) DS18B20: ");
	Serial.print(dsTemp2, 2);
	Serial.println(" C");
	
	Serial.print("3) DS3231 (RTC): ");
	Serial.print(rtcTemp, 2);
	Serial.println(" C");
	
	delay(1000);
}
