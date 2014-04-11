#include <SHT15.h>
#include <HTU21D.h>
#include <DHT.h>
#include <Wire.h>

#define RHT_PIN 3
#define SHT_CLK_PIN 6
#define SHT_DATA_PIN 5

DHT humiditySensor1(RHT_PIN, DHT22, 3);
HTU21D humiditySensor2;
SHT15 humiditySensor3(SHT_CLK_PIN,SHT_DATA_PIN);

unsigned long startTime;
unsigned long conversionTime;

void setup()
{
	Serial.begin(9600);
	Wire.begin();
	
	humiditySensor1.begin();
	humiditySensor2.begin();
}

void loop()
{
	startTime = millis();
	
	float temp1 = humiditySensor1.readTemperature();
	float humidity1 = humiditySensor1.readHumidity();
	
	float temp2 = humiditySensor2.readTemperature();
	float humidity2 = humiditySensor2.readHumidity();
	humidity2 = getCorrectHTU21DHumidity(temp2, humidity2);
	
	float temp3 = humiditySensor3.readTemperature();
	float humidity3 = humiditySensor3.readHumidity();
	
	// Conversion time
	conversionTime = millis() - startTime;
	Serial.print("Conversion Time: ");
	Serial.print(conversionTime, DEC);
	Serial.println(" ms");

	Serial.print("RHT03: \n\tT: ");
	Serial.print(temp1, 2);
	Serial.print(" C\n\tRH: ");
	Serial.print(humidity1, 2);
	Serial.println(" %");

	Serial.print("HTU21D: \n\tT: ");
	Serial.print(temp2, 2);
	Serial.print(" C\n\tRH: ");
	Serial.print(humidity2, 2);
	Serial.println(" %");

	Serial.print("SHT15: \n\tT: ");
	Serial.print(temp3, 2);
	Serial.print(" C\n\tRH: ");
	Serial.print(humidity3, 2);
	Serial.println(" %");

	delay(1000);
}

float getCorrectHTU21DHumidity(float temperature, float humidity){
	float correctedHumidity = humidity + (25 - temperature)*-0.15;
	return correctedHumidity;
}
