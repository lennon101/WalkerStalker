#include <Wire.h>
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include <SHT15.h>
#include <HTU21D.h>
#include <DHT.h>
#include <OneWire.h>
#include <DS3231.h>

const char filename[] = "log.csv";
const char headings[] = "Timestamp,Temp 03,Temp 21,Temp 15,Humidity 03,Humidity 21,Humidity 15";

#define RHT03_PIN 3
#define RHT03_TYPE DHT22
#define RHT03_COUNT 3
DHT humiditySensor03(RHT03_PIN, RHT03_TYPE, RHT03_COUNT);

#define SHT15_CLK_PIN 7
#define SHT15_DATA_PIN 6
SHT15 humiditySensor15(SHT15_CLK_PIN, SHT15_DATA_PIN);

DS3231 RTC;

#define CHIP_SELECT_PIN 10
File dataFile;

HTU21D humiditySensor21;

void setup()
{
	// Comms
	Serial.begin(57600);
	Serial.println("Awake");
	
	initialiseDatalog();
	
	Wire.begin();
	RTC.begin();
	
	humiditySensor03.begin();
	humiditySensor21.begin();

	Serial.println(headings);
}

void loop()
{	
	// Humidity/Temp
	float temp03 = humiditySensor03.readTemperature();
	float humidity03 = humiditySensor03.readHumidity();
	
	float temp21 = humiditySensor21.readTemperature();
	float humidity21 = humiditySensor21.readHumidity();
	humidity21 = getCorrectHTU21DHumidity(temp21, humidity21);
	
	float temp15 = humiditySensor15.readTemperature();
	float humidity15 = humiditySensor15.readHumidity();
	
	// Print values
	DateTime timeStamp = RTC.now();
	printTimeStamp(timeStamp);
	Serial.print("  Th: ");
	Serial.print(temp03);
	Serial.print(", ");
	Serial.print(temp21);
	Serial.print(", ");
	Serial.print(temp15);
	
	Serial.print("  RH: ");
	Serial.print(humidity03);
	Serial.print(", ");
	Serial.print(humidity21);
	Serial.print(", ");
	Serial.print(humidity15);
	Serial.println();
	
	delay(200);
	
	dataFile.print(timeStamp.get());
	dataFile.print(",");
	dataFile.print(temp03, 2);
	dataFile.print(",");
	dataFile.print(temp21, 2);
	dataFile.print(",");
	dataFile.print(temp15, 2);
	dataFile.print(",");
	dataFile.print(humidity03, 2);
	dataFile.print(",");
	dataFile.print(humidity21, 2);
	dataFile.print(",");
	dataFile.print(humidity15, 2);
	dataFile.println();
	
	dataFile.flush();
	
	// Write to SD card
	delay(5000);
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
	
	// Add leading 0 if needed
	if(timeStamp.second() < 10){
		Serial.print("0");
	}
	
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




