#include "Arduino.h"
#include "avr/power.h"
#include "avr/sleep.h"
#include <EmonLib.h>
#include <SPI.h>
#include <SD.h>
#include <XBee.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_TMP006.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <HTU21D.h>  // I2C Timeout modified to allow slower clock rate; no longer produces errors
#include <DS3231.h>
#include <Battery.h>
#include <XBee.h>


//////////////////////////////////////////////////////////////////////////

const char filename[] = "log.csv";

// Pin Assignments
#define XBEE_PWR_PIN 5	// XBee radio power
#define TF_PWR_PIN 4	// Trans-flash (SD) card power
#define AIR_TEMP_PIN 9
#define CHIP_SELECT_PIN 10	// SD Card
#define CURRENT_SENSE_PIN 1	// Analog - A1
#define MIC_PIN 0			// Analog - A0

// Temperature
#define AIR_TEMP_ID 0
#define WALL_TEMP_ID 1
OneWire oneWire(AIR_TEMP_PIN);
DallasTemperature airTempSensors(&oneWire);
Adafruit_TMP006 wallTemperatureSensor(0x44);

// Humidity
HTU21D humiditySensor21;

// Light - Pick one
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
//BH1750FVI lightSensor2;

// Sound
#define MIC_SAMPLE_PERIOD 100	// Sampling window for microphone in milliseconds

// Power
#define TRANSFORMER_RATIO 2000	// The ratio of the current transformer
#define BURDEN_RESISTOR 220		// Value of the burden resistor in Ohms
#define CURRENT_SAMPLES 1000	// Number of ADC samples taken when measuring current
const float CURRENT_CALIBRATION_FACTOR = TRANSFORMER_RATIO/BURDEN_RESISTOR;
EnergyMonitor currentSensor;

// Misc
#define DEFAULT_DECIMAL_PLACES 2	// Number of decimal places conserved in float>>int conversions
#define COMMS_DELAY 300				// Wait time after sending packets through serial or SPI
#define XBEE_WAKE_DELAY 3000
#define SD_CARD_WAIT_DELAY 1000		// Length of time between checking that the SD card is present during initialization
#define SAMPLE_PERIOD 10	// Number of minutes between samples
#define PACKET_BUFFER_SIZE 100		// Number of bytes in the packet buffer

// Transmit packet buffer
byte packetBuffer[PACKET_BUFFER_SIZE];
byte bufferPutter;
byte bufferGetter;

DS3231 RTC;
Battery battery;
const int INTERRUPT_NUM = 0;
File dataFile;

XBee xbee = XBee();
XBeeAddress64 coordinatorAddress = XBeeAddress64(0x0013A200, 0x040AA1A3E);
ZBTxRequest zbTx = ZBTxRequest(coordinatorAddress, packetBuffer, bufferPutter);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();


// Data variables
int airTemp;	// DS18B20
int wallTemp1;	// DS18B20
int wallTemp2;	// TMP006
int caseTemp;	// DS3231
int humidity;	// HTU21D
int lightLevel;	// TSL2561
int soundLevel;	// Freetronics MIC
int currentConsumption;	// Split current transformer
int batteryCapacity;	// Capacity of LiPo battery in percent



//////////////////////////////////////////////////////////////////////////

/**
* Initialization stage
* Runs once when power is active
*/
void setup()
{
	// Communications
	Serial.begin(57600);
	Wire.begin();
	initialiseXBee();
	RTC.begin();
	
	// Start logging
	resetBuffer();
	initialiseDatalog();
	
	// Start sensors
	initialiseTemperatureSense();
	initialiseHumiditySense();
	initialiseLightSense();
}


/**
* Main Loop - runs indefinitely
*/
void loop()
{
	readTemperature();
	readHumidity();
	readLuminosity();
	readSound();
	readCurrent();
	readBatteryCapacity();
	
	// Prepare the data for transmission
	resetBuffer();
	writeDataToPacketBuffer();
	
	// Transmit data to the various mediums
	transmitData();
	//writeDataToSerial();
	writeDataToLog();
	
	sleepNow();
	
	// Sleep Point //
	disableSleep();	
	sleepLoop();	// Sleep ends after 10 minutes
	
	
	wakeUp();
}

//////////////////////////////////////////////////////////////////////////


/**
* Set up all temperature sensors
*/
void initialiseTemperatureSense()
{
	airTempSensors.begin();
	wallTemperatureSensor.begin(TMP006_CFG_1SAMPLE);
	RTC.begin();
}


/**
* Set up all humidity sensors
*/
void initialiseHumiditySense()
{
	humiditySensor21.begin();
}


/**
* Set up all light sensors
*/
void initialiseLightSense()
{
	lightSensor.begin();
}


/**
* Set up the XBee radio
*/
void initialiseXBee(){
	pinMode(XBEE_PWR_PIN, OUTPUT);
	powerUpXbee();
	
	xbee.setSerial(Serial);
}


/**
* Set up the SD card and data file
*/
void initialiseDatalog(){
	pinMode(SS, OUTPUT);
	
	// Initialise SD card - Keep trying until SD is found
	while(!SD.begin(CHIP_SELECT_PIN)){
		Serial.println("Card error");
		delay(SD_CARD_WAIT_DELAY);
	}
	
	// Initialse data file
	dataFile = SD.open(filename, O_WRITE |  O_APPEND | O_CREAT);
	if  (!dataFile) {
		Serial.println("File error");
		// Wait forever since we can not write data
		while (1) ;
	}
}

void sleepLoop()
{
	while (RTC.now().minute() % SAMPLE_PERIOD != 0){
		sleepNow();
		disableSleep();
	}
}

/**
* Take readings from all temperature sensors
* Results are pushed to global variables
*/
void readTemperature()
{
	// DS18B20 Readings
	airTempSensors.requestTemperatures();
	airTemp = floatToInt(airTempSensors.getTempCByIndex(AIR_TEMP_ID), DEFAULT_DECIMAL_PLACES);
	wallTemp1 = floatToInt(airTempSensors.getTempCByIndex(WALL_TEMP_ID), DEFAULT_DECIMAL_PLACES);
	
	// TMP006
	wallTemp2 = floatToInt(wallTemperatureSensor.readObjTempC(), DEFAULT_DECIMAL_PLACES);
	
	// DS3231
	RTC.convertTemperature();
	caseTemp = floatToInt(RTC.getTemperature(), DEFAULT_DECIMAL_PLACES);
}


/**
* Take readings from all humidity sensors
* Results are pushed to global variables
*/
void readHumidity()
{
	// HTU21D
	float temp21 = humiditySensor21.readTemperature();
	float humidity21 = humiditySensor21.readHumidity();
	humidity = floatToInt(getCorrectHTU21DHumidity(temp21, humidity21), DEFAULT_DECIMAL_PLACES);
}


/**
* Take readings from all luminosity sensors
* Results are pushed to global variables
*/
void readLuminosity()
{
	// TSL2561
	sensors_event_t event;
	lightSensor.getEvent(&event);
	lightLevel = event.light;
}


/**
* Take readings from all sound sensors
* Results are pushed to global variables
*/
void readSound()
{
	soundLevel = getSoundLevel(MIC_SAMPLE_PERIOD);
}


/**
* Take readings from all current sensors
* Results are pushed to global variables
*/
void readCurrent()
{
	currentConsumption = floatToInt(currentSensor.calcIrms(CURRENT_SAMPLES), DEFAULT_DECIMAL_PLACES);
}


/**
* Take readings from the battery charger
* Results are pushed to global variables
*/
void readBatteryCapacity(){
	battery.update();
	batteryCapacity = battery.getPercentage();
}


/**
* Convert a floating point decimal number into an int
* The decimal is shifted prior to conversion to preserve precision, but reduce range.
* e.g. floatToInt(12.34, 2) = 1234
*
* @return decimal-shifted integer
* @param num Float number to convert
* @param decimalShift Number of decimal shifts (1-4)
*/
int floatToInt(float num, int decimalShift){
	long decimalMultiplier = 1;
	
	if(decimalShift > 0 && decimalShift < 5){
		decimalMultiplier = pow(10, decimalShift);
	}
	
	return (int)(num*decimalMultiplier);
}


/**
* Print the current time in hh:mm:ss
*/
void printTimeStamp(DateTime timeStamp){
	Serial.print(timeStamp.hour());
	Serial.print(":");
	Serial.print(timeStamp.minute());
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
* @param temperature The temperature of the HTU21D sensor in ï¿½C
* @param humidity The relative humidity reading of the HTU21D sensor in %
* @return The corrected humidity value in %
*/
float getCorrectHTU21DHumidity(float temperature, float humidity){
	float correctedHumidity = humidity + (25 - temperature)*-0.15;
	return correctedHumidity;
}


/**
* Get the average sound level over the specified sampling period
*
* @param samplePeriod Listening period for the sampling in ms.
* @return Average sound level in 10-bit counts
*/
int getSoundLevel(int samplePeriod){
	unsigned long startTime = millis();
	long total = 0;
	long count = 0;
	
	while (millis() < (startTime + samplePeriod) && samplePeriod > 0){
		int soundLevel = analogRead(MIC_PIN);
		total += soundLevel;
		count += 1;

	}
	
	int average = int(total/count);
	return average;
}


/**
* Put the microcontroller in a low power state
* Peripherals are powered down
*/
void sleepNow(){
	powerDownXbee();
	powerDownTF();
	
	sleepController();
}


void wakeUp(){
	disableSleep();
	
	powerUpXbee();
	powerUpTF();
}


void disableSleep(){
	detachInterrupt(INT0);
	sleep_disable();
}


/**
* Put the chip to sleep
* Only a reset, or a button interrupt can wake up the chip
*/
void sleepController()
{
	/*Initialize INT0 for accepting interrupts */
	PORTD |= 0x04;
	DDRD &=~ 0x04;
	
	// Lowest level sleep - Highest power savings
	// The microcontroller can only be woken by reset or external interrupt
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	RTC.clearINTStatus();
	
	// Take the safety off
	sleep_enable();
	
	// Enable RTC interrupts
	attachInterrupt(INT0, clockInterrupt, LOW);
	RTC.enableInterrupts(EveryMinute);
	
	sleep_mode();
}


/**
* Stop power to the XBee radio
*/
void powerDownXbee(){
	digitalWrite(XBEE_PWR_PIN, LOW);
}


/**
* Supply power to the XBee radio
*/
void powerUpXbee(){
	digitalWrite(XBEE_PWR_PIN, HIGH);
	delay(XBEE_WAKE_DELAY);
}


/**
* Cut power to the SD card
*/
void powerDownTF(){
	digitalWrite(TF_PWR_PIN, LOW);
}


/**
* Supply power to the SD card
*/
void powerUpTF(){
	digitalWrite(TF_PWR_PIN, HIGH);
}


void writeDataToLog(){
	dataFile.write(packetBuffer, bufferPutter);
	dataFile.flush();
	delay(COMMS_DELAY);
}


void writeDataToSerial(){
	Serial.write(packetBuffer, bufferPutter);
	
}


void transmitData(){
	zbTx = ZBTxRequest(coordinatorAddress, packetBuffer, bufferPutter);
	xbee.send(zbTx);
	
	// Wait for response
	if (xbee.readPacket(500)) {

		// The packet should be have a response api id
		if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
			xbee.getResponse().getZBTxStatusResponse(txStatus);

			// get the delivery status, the fifth byte
			if (txStatus.getDeliveryStatus() == SUCCESS) {
				// success.  time to celebrate
				
				} else {
				// Remote unit had trouble receiving
			}
		}
		
		} else if (xbee.getResponse().isError()) {
		
		} else {
		// Timeout
		
	}

}


void writeDataToPacketBuffer(){
	writeTimeToBuffer();
	toBuffer(airTemp);
	toBuffer(wallTemp1);
	toBuffer(wallTemp2);
	toBuffer(caseTemp);
	toBuffer(humidity);
	toBuffer(lightLevel);
	toBuffer(soundLevel);
	toBuffer(currentConsumption);
	toBuffer(batteryCapacity);
	toBuffer((byte)0x0A);	// Newline
}


/**
* Send the current timestamp to the buffer
* The data is entered into the buffer as 4 separate bytes
*/
void writeTimeToBuffer(){
	long timeStamp = RTC.now().get();
	
	byte b1 = (byte) timeStamp;
	byte b2 = (byte) (timeStamp >> 8 & 0xFF);
	byte b3 = (byte) (timeStamp >> 16 & 0xFF);
	byte b4 = (byte) (timeStamp >> 24 & 0xFF);
	

	toBuffer(b4);
	toBuffer(b3);
	toBuffer(b2);
	toBuffer(b1);
}


/**
* Periodic interrupt ISR
* Intentionally left blank
*/
void clockInterrupt(){
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

void toBuffer(unsigned int num){
	toBuffer(num);
}

void toBuffer(int num){
	byte high = highByte(num);
	byte low = lowByte(num);
	
	toBuffer(high);
	toBuffer(low);
}

void toBuffer(byte b){
	if (bufferPutter < PACKET_BUFFER_SIZE){
		packetBuffer[bufferPutter] = b;
		bufferPutter++;
	}
}

byte fromBuffer(){
	byte b = packetBuffer[bufferGetter];
	
	if (bufferGetter < PACKET_BUFFER_SIZE){
		bufferGetter++;
	}
	
	return b;
}

void resetBuffer(){
	bufferPutter = 0;
	bufferGetter = 0;
}
