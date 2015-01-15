//////////////////////////////////////////////////////////////////////////
// Walker Stalker v20 [11/4/2014 Leenix]
//////////////////////////////////////////////////////////////////////////
//
//
// Changelog:
// Added auto-retry to XBee transmissions [11/4/2014 Leenix]
//
//////////////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include "avr/power.h"
#include "avr/sleep.h"
#include "avr/wdt.h"
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
#include <DHT.h>
#include <DS3231.h>
#include <Battery.h>
#include <XBee.h>

//////////////////////////////////////////////////////////////////////////
// Unit-specific variables

#define UNIT_NUMBER 8
DeviceAddress airTempAddress =  {0x28, 0x55, 0x9F, 0x22, 0x05, 0x00, 0x00, 0x41};
DeviceAddress wallTempAddress = {0x28, 0x47, 0xA1, 0x22, 0x05, 0x00, 0x00, 0x52};

//////////////////////////////////////////////////////////////////////////
// Configuration
const char filename[] = "log.txt";

// Pin Assignments
#define SENSOR_POWER_PIN A2
#define XBEE_PWR_PIN 5	// XBee radio power
#define TF_PWR_PIN 4	// Trans-flash (SD) card power
#define AIR_TEMP_PIN 9
#define CHIP_SELECT_PIN 10	// SD Card
#define CURRENT_SENSE_PIN 1	// Analog - A1
#define MIC_PIN 0			// Analog - A0
#define RHT_PIN 3

// Temperature
#define TEMPERATURE_PRECISION 12
OneWire oneWire(AIR_TEMP_PIN);
DallasTemperature airTempSensors(&oneWire);
Adafruit_TMP006 wallTemperatureSensor(0x44);

// Humidity
#define DHT_TYPE DHT22
DHT humiditySensor03(RHT_PIN, DHT_TYPE, 3);

// Light
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);

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
#define XBEE_WAKE_DELAY 1000
#define SD_CARD_WAIT_DELAY 1000		// Length of time between checking that the SD card is present during initialization
#define SAMPLE_PERIOD 10 // Number of minutes between samples
#define PACKET_BUFFER_SIZE 100		// Number of bytes in the packet buffer
#define SAMPLE_UPTIME 5	// Length of time that the system stays awake after a sample (for transmission reasons) in seconds
#define XBEE_MAX_RETRIES 3	// Number of retries possible
#define XBEE_ACK_TIMEOUT 1000	// Timeout in milliseconds

// Transmit packet buffer
byte packetBuffer[PACKET_BUFFER_SIZE];
byte bufferPutter;
byte bufferGetter;

DS3231 RTC;
Battery battery;
const int INTERRUPT_NUM = 0;
File dataFile;

XBee xbee = XBee();
XBeeAddress64 coordinatorAddress = XBeeAddress64(0x0, 0x0);	// Send to coordinator
ZBTxRequest zbTx = ZBTxRequest(coordinatorAddress, packetBuffer, bufferPutter);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();


// Data variables
int airTemp;	// DS18B20
int wallTemp;	// DS18B20
int surfaceTemp;	// TMP006
int caseTemp;	// DS3231
int humidity;	// HTU21D
int lightLevel;	// TSL2561
int soundLevel;	// Freetronics MIC
int currentConsumption;	// Split current transformer
int batteryCapacity;	// Capacity of LiPo battery in percent


//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initialization stage
* Runs once when power is active
*/
void setup(){
	disableWatchdog();
	
	// Communications
	Serial.begin(57600);
	Wire.begin();
	initialiseXBee();
	RTC.begin();
	
	// Start logging
	resetBuffer();
	
	// Start sensors
	initialiseSensors();
	
	showDebugStartMessage();
}


/**
* Main Loop - runs indefinitely
*/
void loop(){
	showDebugWakeMessage();
	enableWatchdog();
	
	getSensorReadings();
	showDebugSensorReadings();
	
	// Pet the dog
	wdt_reset();
	
	// Transmit the recorded data
	resetBuffer();
	writeDataToPacketBuffer();
	
	// Offset delay to avoid collisions
	delay(COMMS_DELAY * UNIT_NUMBER);
	transmitData();
	showDebugTransmitNotification();
	
	disableWatchdog();
	
	// Hold-off time
	delay(SAMPLE_UPTIME * 1000);
	
	showDebugSleepMessage();
	enterSleep();	// Sleep ends after 10 minutes
	
	wakeUp();
}

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Buffer

/**
* Send an integer to the buffer
* The value will be stored in its raw, byte form
*/
void toBuffer(unsigned int num){
	toBuffer(num);
}


/**
* Send an integer to the buffer
* The value will be stored in its raw, byte form
*/
void toBuffer(int num){
	byte high = highByte(num);
	byte low = lowByte(num);
	
	toBuffer(high);
	toBuffer(low);
}


/**
* Store the input byte into the buffer
* The putter position is incremented after a successful entry
*/
void toBuffer(byte b){
	if (bufferPutter < PACKET_BUFFER_SIZE){
		packetBuffer[bufferPutter] = b;
		bufferPutter++;
	}
}


/**
* Receive a character from the buffer
* The buffer getter will increment upon successful retrieval of a byte
*/
byte fromBuffer(){
	byte b = packetBuffer[bufferGetter];
	
	if (bufferGetter < PACKET_BUFFER_SIZE){
		bufferGetter++;
	}
	
	return b;
}


/**
* Ready the buffer to accept new data
*/
void resetBuffer(){
	bufferPutter = 0;
	bufferGetter = 0;
}


/**
* Records all sampled sensor data to the buffer in raw form
*/
void writeDataToPacketBuffer(){
	toBuffer(byte(UNIT_NUMBER));
	writeTimeToBuffer();
	toBuffer(airTemp);
	toBuffer(wallTemp);
	toBuffer(surfaceTemp);
	toBuffer(caseTemp);
	toBuffer(humidity);
	toBuffer(lightLevel);
	toBuffer(soundLevel);
	toBuffer(currentConsumption);
	toBuffer((byte)batteryCapacity);
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


//////////////////////////////////////////////////////////////////////////
// Communications - XBee

/**
* Set up the XBee radio
*/
void initialiseXBee(){
	pinMode(XBEE_PWR_PIN, OUTPUT);
	powerUpXbee();
	
	xbee.setSerial(Serial);
}


/**
* Send the recorded packet over the XBee
* Uses API mode transmission
*/
void transmitData(){
	zbTx = ZBTxRequest(coordinatorAddress, 0xFFFE, 0, 0x0, packetBuffer, bufferPutter, UNIT_NUMBER);
	int retries = 0;
	bool packetSent = false;
	
	// Attempt to send until the packet transmits or times out
	while(!packetSent && retries <= XBEE_MAX_RETRIES){
		retries++;
		wdt_reset();
		
		xbee.send(zbTx);
					
		// Check for acknowledgment
		if (xbee.readPacket(XBEE_ACK_TIMEOUT)) {
			Serial.println("Response received");

			// should be a znet tx status
			if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
				xbee.getResponse().getZBTxStatusResponse(txStatus);

				// get the delivery status, the fifth byte
				if (txStatus.getDeliveryStatus() == SUCCESS) {
					showDebugTransmitSuccess();
					packetSent = true;
					
					} else {
					// Packet not sent
				}
			}
		}
		
		else if (xbee.getResponse().isError()) {
			Serial.println("Error receiving packet");
		}
		
		else {
			// Local XBee did not return a response (very unusual)
			Serial.print("No response from XBee");
		}
	}
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


//////////////////////////////////////////////////////////////////////////
// Debug Notifications

void showDebugStartMessage(){
	Serial.print("\nWalker Stalker v20 - Unit ");
	Serial.println(int(UNIT_NUMBER));
}

void showDebugSensorReadings(){
	Serial.print("Air Temp:\t");
	Serial.println(airTemp);
	Serial.print("Wall Temp:\t");
	Serial.println(wallTemp);
	Serial.print("Surface Temp:\t");
	Serial.println(surfaceTemp);
	Serial.print("Case Temp:\t");
	Serial.println(caseTemp);
	Serial.print("Humidity:\t");
	Serial.println(humidity);
	Serial.print("Light Level:\t");
	Serial.println(lightLevel);
	Serial.print("Sound Level:\t");
	Serial.println(soundLevel);
	Serial.print("Current:\t");
	Serial.println(currentConsumption);
}

void showDebugTransmitNotification(){
	Serial.println("Packet transmitted");
}

void showDebugTransmitSuccess(){
	Serial.println("Acknowledgment received - successful packet");
}

void showDebugSleepMessage(){
	Serial.println("Sleep time");
}

void showDebugWakeMessage(){
	Serial.print("\nAwake: ");
	printTimestamp();
}

void showDebugWakePing(){
	Serial.print("Still alive - Battery: ");
	
	battery.update();
	Serial.print(battery.getPercentage());
	Serial.print("% ~ ");
	Serial.println(battery.getChStatus());
	Serial.flush();
}

void printTimestamp(){
	DateTime timestamp = RTC.now();
	
	Serial.print(timestamp.year());
	Serial.print("-");
	if(timestamp.month() < 10){Serial.print(0);}
	Serial.print(timestamp.month());
	Serial.print("-");
	if(timestamp.date() < 10){Serial.print(0);}
	Serial.print(timestamp.date());
	Serial.print(" ");
	if(timestamp.hour() < 10){Serial.print(0);}
	Serial.print(timestamp.hour());
	Serial.print(":");
	if(timestamp.minute() < 10){Serial.print(0);}
	Serial.print(timestamp.minute());
	Serial.print(":");
	if(timestamp.second() < 10){Serial.print(0);}
	Serial.println(timestamp.second());
}

//////////////////////////////////////////////////////////////////////////
// Sensors

/**
* Initialise all the sensors
*/
void initialiseSensors(){
	initialiseTemperatureSense();
	initialiseHumiditySense();
	initialiseLightSense();
}


/**
* Set up all temperature sensors
*/
void initialiseTemperatureSense(){
	airTempSensors.begin();
	airTempSensors.setResolution(TEMPERATURE_PRECISION);
	wallTemperatureSensor.begin(TMP006_CFG_2SAMPLE);
}


/**
* Set up all humidity sensors
*/
void initialiseHumiditySense(){
	humiditySensor03.begin();
}


/**
* Set up all light sensors
*/
void initialiseLightSense(){
	lightSensor.begin();
	lightSensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
}


/**
* Read in data from all the sensors
*/
void getSensorReadings(){
	readHumidity();
	readTemperature();
	readLuminosity();
	readSound();
	readCurrent();
	readBatteryCapacity();
}


/**
* Take readings from all temperature sensors
* Results are pushed to global variables
*/
void readTemperature(){
	// DS18B20 Readings
	airTempSensors.requestTemperatures();
	airTemp = floatToInt(airTempSensors.getTempC(airTempAddress), DEFAULT_DECIMAL_PLACES);
	wallTemp = floatToInt(airTempSensors.getTempC(wallTempAddress), DEFAULT_DECIMAL_PLACES);
	
	// TMP006
	surfaceTemp = floatToInt(wallTemperatureSensor.readObjTempC(), DEFAULT_DECIMAL_PLACES);
	
	// DS3231
	RTC.convertTemperature();
	caseTemp = floatToInt(RTC.getTemperature(), DEFAULT_DECIMAL_PLACES);
}


/**
* Take readings from all humidity sensors
* Results are pushed to global variables
*/
void readHumidity(){
	// RHT03
	humidity = floatToInt(humiditySensor03.readHumidity(), DEFAULT_DECIMAL_PLACES);
}


/**
* Take readings from all luminosity sensors
* Results are pushed to global variables
*/
void readLuminosity(){
	// TSL2561
	sensors_event_t event;
	lightSensor.getEvent(&event);
	lightLevel = event.light;
}


/**
* Take readings from all sound sensors
* Results are pushed to global variables
*/
void readSound(){
	soundLevel = getSoundLevel(MIC_SAMPLE_PERIOD);
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
* Take readings from all current sensors
* Results are pushed to global variables
*/
void readCurrent(){
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
* Switch the sensors off
* WalkerStalker v2.3 needed
*/
void powerDownSensors(){
	digitalWrite(SENSOR_POWER_PIN, LOW);
}


/**
* Switch the sensors on
* WalkerStalker v2.3 needed
*/
void powerUpSensors(){
	pinMode(SENSOR_POWER_PIN, OUTPUT);
	digitalWrite(SENSOR_POWER_PIN, HIGH);
	Wire.begin();
}


//////////////////////////////////////////////////////////////////////////
// Sleep

/**
* Put the microcontroller into sleep mode until the sample period has elapsed
* Relies on the RTC using everyMinute interrupts
*/
void enterSleep(){
	sleepNow();
	
	// Sleep Point //
	
	disableSleep();
	while ((RTC.now().minute() % SAMPLE_PERIOD) != 0){
		showDebugWakePing();
		sleepNow();
		disableSleep();
	}
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


/**
* Wake the microcontroller and peripherals from sleep
*/
void wakeUp(){
	disableSleep();
	
	powerUpXbee();
	//powerUpTF();
	powerUpSensors();
}


/**
* Disable sleep from occuring
* Pin interrupts are detached and the safety is put back on
*/
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
	
	// Enable RTC interrupts
	RTC.clearINTStatus();
	attachInterrupt(INT0, clockInterrupt, LOW);
	RTC.enableInterrupts(EveryMinute);
	
	// Take the safety off
	sleep_enable();
	
	// Put the controller to sleep
	sleep_mode();
}


/**
* Periodic interrupt ISR
* Intentionally left blank
*/
void clockInterrupt(){
	
}


//////////////////////////////////////////////////////////////////////////
// Watchdog

/**
* Enable the watchdog timer for 8 second timeouts
*/
void enableWatchdog(){
	cli();
	
	wdt_reset();
	
	wdt_enable(WDTO_8S);

	sei();
}


/**
* Disable the watchdog timer
*/
void disableWatchdog(){
	cli();
	
	wdt_reset();
	
	wdt_disable();
	
	sei();
}


//////////////////////////////////////////////////////////////////////////
// SD Card - Obsolete (Delete next iteration)

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
	}
}


/**
* Write the stored packet to the SD card log file
*/
void writeDataToLog(){
	dataFile.write(packetBuffer, bufferPutter);
	dataFile.flush();
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
