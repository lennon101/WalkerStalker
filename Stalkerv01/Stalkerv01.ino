#include <EmonLib.h>
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <XBee.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_TMP006.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <BH1750FVI.h>
#include <SHT15.h>
#include <HTU21D.h>
#include <DHT.h>
#include <DS3231.h>

const char filename[] = "log.csv";
const char headings[] = "Timestamp,Air 1,Air 2,Temp 03,Temp 21,Temp 15,Surface,Humidity 03,Humidity 21,Humidity 15,Light 1,Light 2,LDR,Sound,Current";

#define XBEE_PWR_PIN 5

#define AIR_TEMP_PIN 9
OneWire oneWire(AIR_TEMP_PIN);
DallasTemperature airTempSensors(&oneWire);
Adafruit_TMP006 surfaceTempSensor;

#define RHT03_PIN 3
#define RHT03_TYPE DHT22
#define RHT03_COUNT 3
DHT humiditySensor03(RHT03_PIN, RHT03_TYPE, RHT03_COUNT);

#define SHT15_CLK_PIN 7
#define SHT15_DATA_PIN 6
SHT15 humiditySensor15(SHT15_CLK_PIN, SHT15_DATA_PIN);

#define LDR_PIN 2
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
BH1750FVI lightSensor2;

#define MIC_PIN 0
#define MIC_SAMPLE_PERIOD 100

DS3231 RTC;

#define CHIP_SELECT_PIN 10
File dataFile;

HTU21D humiditySensor21;

#define CURRENT_SENSE_PIN 1
#define TRANSFORMER_RATIO 2000
#define BURDEN_RESISTOR 220
const float CURRENT_CALIBRATION_FACTOR = TRANSFORMER_RATIO/BURDEN_RESISTOR;
EnergyMonitor currentSensor;

void setup()
{
	// Comms
	Serial.begin(9600);
	Serial.println("Awake");
	
	initialiseDatalog();
	
	Wire.begin();
	
	RTC.begin();
	
	airTempSensors.begin();
	
	surfaceTempSensor.begin(TMP006_CFG_1SAMPLE);

	humiditySensor03.begin();
	humiditySensor21.begin();	

	lightSensor.begin();
	lightSensor2.begin();
	lightSensor2.SetAddress(Device_Address_H);
	lightSensor2.SetMode(Continuous_H_resolution_Mode);
	
	currentSensor.current(CURRENT_SENSE_PIN, CURRENT_CALIBRATION_FACTOR);
	
	Serial.println(headings);
}

void loop()
{
	// Air temperature
	airTempSensors.requestTemperatures();
	float airTemp1 = airTempSensors.getTempCByIndex(0);
	float airTemp2 = airTempSensors.getTempCByIndex(1);
	
	// Surface temperature
	float surfaceTemp = surfaceTempSensor.readObjTempC();
	
	// Humidity/Temp
	float temp03 = humiditySensor03.readTemperature();
	float humidity03 = humiditySensor03.readHumidity();
	
	float temp21 = humiditySensor21.readTemperature();
	float humidity21 = humiditySensor21.readHumidity();
	humidity21 = getCorrectHTU21DHumidity(temp21, humidity21);
	
	float temp15 = humiditySensor15.readTemperature();
	float humidity15 = humiditySensor15.readHumidity();
	
	// Luminosity
	sensors_event_t event;
	lightSensor.getEvent(&event);
	int lightLevel = event.light;
	uint16_t lightLevel2 = lightSensor2.GetLightIntensity();
	int ldrLightLevel = analogRead(LDR_PIN);
	
	// Sound pressure level
	int soundLevel = getSoundLevel(MIC_SAMPLE_PERIOD);
	
	//Current
	float currentSense = currentSensor.calcIrms(1000);
	
	// Print values
	DateTime timeStamp = RTC.now();
	printTimeStamp(timeStamp);
	Serial.print(",");
	Serial.print(int(airTemp1*100));
	Serial.print(",");
	Serial.print(int(airTemp2*100));
	Serial.print(",");
	Serial.print(int(temp03*100));
	Serial.print(",");
	Serial.print(temp21, 2);
	Serial.print(",");
	Serial.print(temp15, 2);
	Serial.print(",");
	Serial.print(surfaceTemp, 2);
	Serial.print(",");
	Serial.print(humidity03, 2);
	Serial.print(",");
	Serial.print(humidity21, 2);
	Serial.print(",");
	Serial.print(humidity15, 2);
	Serial.print(",");
	Serial.print(lightLevel);
	Serial.print(",");
	Serial.print(lightLevel2);
	Serial.print(",");
	Serial.print(ldrLightLevel);
	Serial.print(",");
	Serial.print(soundLevel);
	Serial.print(",");
	Serial.println(currentSense);
	
	delay(200);
	
	dataFile.print(timeStamp.get());
	dataFile.print(",");
	dataFile.print(airTemp1, 2);
	dataFile.print(",");
	dataFile.print(airTemp2, 2);
	dataFile.print(",");
	dataFile.print(temp03, 2);
	dataFile.print(",");
	dataFile.print(temp21, 2);
	dataFile.print(",");
	dataFile.print(temp15, 2);
	dataFile.print(",");
	dataFile.print(surfaceTemp, 2);
	dataFile.print(",");
	dataFile.print(humidity03, 2);
	dataFile.print(",");
	dataFile.print(humidity21, 2);
	dataFile.print(",");
	dataFile.print(humidity15, 2);
	dataFile.print(",");
	dataFile.print(lightLevel);
	dataFile.print(",");
	dataFile.print(lightLevel2);
	dataFile.print(",");
	dataFile.print(ldrLightLevel);
	dataFile.print(",");
	dataFile.print(soundLevel);
	dataFile.print(",");
	dataFile.println(currentSense, 2);
	
	dataFile.flush();
	
	// Write to SD card
	delay(200);
}


/**
* Set up the SD card and data file
*/
void initialiseDatalog(){
	pinMode(SS, OUTPUT);
	
	// Initialise SD card
	while(!SD.begin(CHIP_SELECT_PIN)){
		Serial.println("Card failed or not present");
		delay(1000);
	}
	
	Serial.println("Card initialised...");
	
	// Initialse data file
	dataFile = SD.open(filename, O_WRITE |  O_APPEND | O_CREAT);
	if  (!dataFile) {
		Serial.println("error opening file");
		// Wait forever since we cant write data
		while (1) ;
	}
	
	
	dataFile.println(headings);
	
}


/**
* Print the current time in hh:mm:ss
*/
void printTimeStamp(DateTime timeStamp){
	
	Serial.print(timeStamp.date());
	Serial.print(".");
	Serial.print(timeStamp.month());
	Serial.print(".");
	Serial.print(timeStamp.year());
	Serial.print(" ");
	Serial.print(timeStamp.hour(), DEC);
	Serial.print(":");
	Serial.print(timeStamp.minute(), DEC);
	Serial.print(":");
	Serial.print(timeStamp.second(), DEC);
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

