#include <XBee.h>
#include <Adafruit_Sensor.h>
#include <pgmspace.h>
#include <Adafruit_TSL2561_U.h>
#include <SHT15.h>
#include <HTU21D.h>
#include "Arduino.h"
#include <DS3231.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_TMP006.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char FILENAME[] = "DATALOG.txt";
uint8_t payload[48];

XBee xbee = XBee();

XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x403e0f30);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

#define AIR_TEMP_PIN 9
OneWire oneWire(AIR_TEMP_PIN);
DallasTemperature airTempSensors(&oneWire);
Adafruit_TMP006 surfaceTempSensor;

#define RHT03_PIN 3
#define RHT03_TYPE DHT22
#define RHT03_COUNT 3
DHT humiditySensor03(RHT03_PIN, RHT03_TYPE, RHT03_COUNT);

#define SHT15_CLK_PIN 6
#define SHT15_DATA_PIN 5
SHT15 humiditySensor15(SHT15_CLK_PIN, SHT15_DATA_PIN);

#define LDR_PIN 2
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);

#define MIC_PIN 0
#define MIC_SAMPLE_PERIOD 100

DS3231 RTC;

#define CHIP_SELECT_PIN 10

HTU21D humiditySensor21;

void setup()
{
	// Comms
	Serial.begin(57600);
	Wire.begin();
	
	Serial.println("Awake");
	
	RTC.begin();
	
	airTempSensors.begin();
	surfaceTempSensor.begin(TMP006_CFG_1SAMPLE);

	humiditySensor03.begin();
	humiditySensor21.begin();

	lightSensor.begin();
}

void loop()
{
	// Request Values
	airTempSensors.requestTemperatures();

	// Read values
	float airTemp1 = airTempSensors.getTempCByIndex(0);
	float airTemp2 = airTempSensors.getTempCByIndex(1);
	
	float surfaceTemp = surfaceTempSensor.readObjTempC();
	
	float temp03 = humiditySensor03.readTemperature();
	float humidity03 = humiditySensor03.readHumidity();
	
	float temp21 = humiditySensor21.readTemperature();
	float humidity21 = humiditySensor21.readHumidity();
	humidity21 = getCorrectHTU21DHumidity(temp21, humidity21);
	
	float temp15 = humiditySensor15.readTemperature();
	float humidity15 = humiditySensor15.readHumidity();
	
	sensors_event_t event;
	lightSensor.getEvent(&event);
	int lightLevel = event.light;
	int ldrLightLevel = analogRead(LDR_PIN);
	
	int soundLevel = getSoundLevel(MIC_SAMPLE_PERIOD);
	
	// Print values
	DateTime timeStamp = RTC.now();
	printTimeStamp(timeStamp);
	
	Serial.println("\n--- Temperature");
	Serial.print("DS18B20 1 (C): ");
	Serial.println(airTemp1, 2);
	
	Serial.print("DS18B20 2 (C): ");
	Serial.println(airTemp2);
	
	Serial.print("TMP006 - Surface (C): ");
	Serial.println(surfaceTemp);
	
	Serial.print("RHT03 (C): ");
	Serial.println(temp03, 2);
	
	Serial.print("HTU21D (C): ");
	Serial.println(temp21, 2);
	
	Serial.print("SHT15 (C): ");
	Serial.println(temp15, 2);
	
	Serial.println("\n--- Humidity");
	
	Serial.print("RHT03 (%): ");
	Serial.println(humidity03, 2);
	
	Serial.print("HTU21D (%): ");
	Serial.println(humidity21, 2);
	
	Serial.print("SHT15 (%): ");
	Serial.println(humidity15, 2);
	
	Serial.println("\n--- Illuminance");
	
	Serial.print("TSL2561 (lux): ");
	Serial.println(lightLevel);
	
	Serial.print("LDR (counts): ");
	Serial.println(ldrLightLevel);
	
	Serial.println("\n--- Sound Pressure");
	
	Serial.print("Mic (counts): ");
	Serial.println(soundLevel);
	
	Serial.println("----------------------------------");


	// Write to SD card
	delay(1000);
}




/**
* Print the current time in hh:mm:ss
*/
void printTimeStamp(DateTime timeStamp){
	
	Serial.print("\n-- ");
	Serial.print(timeStamp.hour(), DEC);
	Serial.print(":");
	Serial.print(timeStamp.minute(), DEC);
	Serial.print(":");
	Serial.print(timeStamp.second(), DEC);
	Serial.println(" --");
}


/**
* Get the corrected relative humidity after apply conversion coefficients
*
* @param temperature The temperature of the HTU21D sensor in °C
* @param humidity The relative humidity reading of the HTU21D sensor in %
* @return The corrected humidity value in %
*/
float getCorrectHTU21DHumidity(float temperature, float humidity){
	float correctedHumidity = humidity + (25 - temperature)*-0.15;
	return correctedHumidity;
}


/**
* Get the maximum sound level over the specified sampling period
*
* @param samplePeriod Listening period for the sampling in ms.
* @return Maximum sound level in 10-bit counts
*/
int getSoundLevel(int samplePeriod){
	unsigned long startTime = millis();
	int maxSoundLevel = 0;
	
	while (millis() < (startTime + samplePeriod) && samplePeriod > 0){
		int soundLevel = analogRead(MIC_PIN);
		
		if (soundLevel > maxSoundLevel){
			maxSoundLevel = soundLevel;
		}
	}
	
	return maxSoundLevel;
}

