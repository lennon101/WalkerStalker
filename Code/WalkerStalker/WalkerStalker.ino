#include <ArduinoCommand.h>

//////////////////////////////////////////////////////////////////////////
const int WALKERSTALKER_VERSION = 2;
//////////////////////////////////////////////////////////////////////////
// WalkerStalker
//
// WSN node for monitoring the external environment of buildings in an
// urban setting.
//
//////////////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include "avr/power.h"
#include "avr/sleep.h"
#include "avr/wdt.h"
#include <StraightBuffer.h>
#include <Logging.h>
#include <EmonLib.h>
#include <JsonGenerator.h>
#include <SPI.h>
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
#include "StalkerConfig.h"

using namespace ArduinoJson::Generator;

//////////////////////////////////////////////////////////////////////////
// Soft config
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Sensors

// Temperature
OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature temperatureProbes(&oneWire);
Adafruit_TMP006 thermopileSensor(TMP006_ADDRESS);

// Humidity - DHT22
// The '3' fixes errors at the 8MHz clock rate. Cannot remember why
DHT humiditySensor(RHT_PIN, DHT_TYPE, 3);

// Light - TSL2561
Adafruit_TSL2561_Unified lightSensor = Adafruit_TSL2561_Unified(TSL2561_ADDRESS);

// Power - Split current clamp
EnergyMonitor currentSensor;

// Battery Monitor - Stalker built-in
Battery battery;

JsonObject<11> sensorData;

//////////////////////////////////////////////////////////////////////////
// Communication

// Transmit packet buffer
StraightBuffer sendBuffer(PACKET_BUFFER_SIZE);
StraightBuffer readBuffer(PACKET_BUFFER_SIZE);

// Wireless - XBee
XBee xbee = XBee();
ZBTxRequest zbTx = ZBTxRequest(COORDINATOR_ADDRESS, sendBuffer.getBufferAddress(), PACKET_BUFFER_SIZE);
XBeeResponse response = XBeeResponse();
ZBRxResponse zbRx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// Commands
ArduinoCommand commandHandler;


//////////////////////////////////////////////////////////////////////////
// RTC

DS3231 RTC;


//////////////////////////////////////////////////////////////////////////
// Main Functions
//////////////////////////////////////////////////////////////////////////

/**
* Initialization stage
* Runs once when power is active
*/
void setup(){
	
	// Disable the watchdog timer to stop restart loops
	disableWatchdog();
	
	Log.Init(LOG_LEVEL_DEBUG, SERIAL_BAUD);
	Log.Info("WalkerStalker - Unit %d. Firmware ver. %d\n", UNIT_NUMBER, WALKERSTALKER_VERSION);

	Wire.begin();
	startXBee();
	RTC.begin();
	startSensors();
}


/**
* Main Loop - runs indefinitely
*/
void loop(){
	
	// Enable watchdog to reset the Stalker if it gets stuck reading a sensor
	enableWatchdog();
	readSensors();
	
	// Transmit the recorded data
	wdt_reset();
	prepareDataPacket();
	transmitData();
	disableWatchdog();
	
	// Hold-off time
	delay(SAMPLE_UPTIME * 1000);
	
	enterSleep();	// Sleep ends after 10 minutes
	wakeUp();
}


//////////////////////////////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////////////////////////////
// Communications - XBee

/**
* Set up the XBee radio
*/
void startXBee(){
	pinMode(XBEE_PWR_PIN, OUTPUT);
	powerUpXbee();
	
	xbee.setSerial(Serial);
}

void addCommands(){
	commandHandler.addDefaultHandler(unknown);	
}

/**
* Write sensor data to buffer for sending via XBee
*/
void prepareDataPacket(){
	sendBuffer.reset();
	
	sendBuffer.write(UNIT_NUMBER);
	sendBuffer.writeLong(RTC.now().get()); // Timestamp
	sendBuffer.writeInt(int(float(sensorData["t_air"]) * 100));
	sendBuffer.writeInt(int(float(sensorData["t_wall"]) * 100));
	sendBuffer.writeInt(int(float(sensorData["t_surf"]) * 100));
	sendBuffer.writeInt(int(float(sensorData["t_case"]) * 100));
	sendBuffer.writeInt(int(float(sensorData["humidity"]) * 100));
	sendBuffer.writeInt(sensorData["illuminance"]);
	sendBuffer.writeInt(sensorData["sound"]);
	sendBuffer.writeInt(int(float(sensorData["current"]) * 100));
	sendBuffer.write(int(sensorData["battery"]));
	sendBuffer.write(int(sensorData["version"]));
}


/**
* Send the recorded packet over the XBee
* Uses API mode transmission
*/
void transmitData(){
	zbTx = ZBTxRequest(COORDINATOR_ADDRESS, 0xFFFE, 0, 0x0, sendBuffer.getBufferAddress(), sendBuffer.getWritePosition(), UNIT_NUMBER);
	int retries = 0;
	bool packetSent = false;
	
	// Attempt to send until the packet transmits or times out
	while(!packetSent && retries <= XBEE_MAX_RETRIES){
		retries++;
		wdt_reset();
		
		xbee.send(zbTx);
		
		// Check for acknowledgment
		if (xbee.readPacket(XBEE_ACK_TIMEOUT)) {
			Log.Debug("Response received");

			// should be a znet tx status
			if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
				xbee.getResponse().getZBTxStatusResponse(txStatus);

				// get the delivery status, the fifth byte
				if (txStatus.getDeliveryStatus() == SUCCESS) {
					Log.Info("Packet delivery successful");
					packetSent = true;
					
					} else {
					// Packet not sent
				}
			}
		}
		
		else if (xbee.getResponse().isError()) {
			Log.Error("Could not received packet");
		}
		
		else {
			// Local XBee did not return a response - Happens when serial is in use
			Log.Error("No response from XBee (Serial in use)\n");
		}
	}
}

void readXBee(){
	// Listen for packets
	if (xbee.readPacket(XBEE_READ_TIMEOUT)){
		
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE || xbee.getResponse().getApiId() == ZB_EXPLICIT_RX_RESPONSE) {
			// Received a data packet
			
			xbee.getResponse().getZBRxResponse(zbRx);
			
			for (int i = 0; i < zbRx.getDataLength(); i++){
				
			}
			
			
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
// Sensors

// All

/**
* Initialise all the sensors
*/
void startSensors(){
	initialiseTemperature();
	initialiseHumidity();
	initialiseIlluminance();
	initialiseSound();
	initialiseCurrent();
	initialiseBattery();
	
	sensorData["id"] = UNIT_ID.c_str();
	sensorData["version"] = WALKERSTALKER_VERSION;
	Log.Info("Sensors started...\n");
}


/**
* Read in data from all the sensors
*/
void readSensors(){
	
	sensorData["t_air"] = readAirTemperature();
	sensorData["t_wall"] = readWallTemperature();
	sensorData["t_surf"] = readSurfaceTemperature();
	sensorData["t_case"] = readCaseTemperature();
	sensorData["humidity"] = readHumidity();
	sensorData["illuminance"] = readIlluminance();
	sensorData["sound"] = readSound();
	sensorData["current"] = readCurrent();
	sensorData["battery"] = readBatteryLevel();
	
	Serial.println(sensorData);
}


// Temperature

/**
* Set up all temperature sensors
*/
void initialiseTemperature(){
	temperatureProbes.begin();
	temperatureProbes.setResolution(TEMPERATURE_PRECISION);
	thermopileSensor.begin(TMP006_CFG_2SAMPLE);
}


/**
* Read the air temperature
* @return Air temperature in °C
*/
float readAirTemperature(){
	temperatureProbes.requestTemperatures();
	return temperatureProbes.getTempC(airTemperatureAddresses[UNIT_NUMBER]);
}


/**
* Read the temperature radiating off the wall
* @return Radiant wall temperature in °C
*/
float readWallTemperature(){
	temperatureProbes.requestTemperatures();
	return temperatureProbes.getTempC(wallTemperatureAddresses[UNIT_NUMBER]);
}


/**
* Read the surface temperature using the IR thermopile sensor
* @return Surface temperature in °C
*/
float readSurfaceTemperature(){
	return thermopileSensor.readObjTempC();
}


/**
* Read the case temperature of the unit
* @return Case temperature in °C
*/
float readCaseTemperature(){
	RTC.convertTemperature();
	return RTC.getTemperature();
}


// Humidity

/**
* Set up all humidity sensors
*/
void initialiseHumidity(){
	humiditySensor.begin();
}


/**
* Take readings from all humidity sensors
* Results are pushed to global variables
*/
float readHumidity(){
	return humiditySensor.readHumidity();
}


// Illuminance

/**
* Set up all light sensors
*/
void initialiseIlluminance(){
	Wire.begin();
	lightSensor.begin();
	lightSensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
}


/**
* Take readings from all luminosity sensors
* Results are pushed to global variables
*
*@return Illuminance in lux
*/
int readIlluminance(){
	sensors_event_t event;
	lightSensor.getEvent(&event);
	return event.light;
}


// Sound Pressure

/**
* Initialise
*/

void initialiseSound(){
	pinMode(MIC_PIN, INPUT);
}


/**
* Get the average sound level over the specified sampling period
*
* @param samplePeriod Listening period for the sampling in ms.
* @return Average sound level in 10-bit counts
*/
int readSound(){
	unsigned long startTime = millis();
	long total = 0;
	long count = 0;
	
	while (millis() < (startTime + MIC_SAMPLE_PERIOD)){
		int soundLevel = analogRead(MIC_PIN);
		total += soundLevel;
		count += 1;
	}
	
	int average = int(total/count);
	return average;
}


// Current


void initialiseCurrent(){
	pinMode(CURRENT_SENSE_PIN, INPUT);
}


/**
* Take readings from all current sensors
* Results are pushed to global variables
*/
float readCurrent(){
	return currentSensor.calcIrms(CURRENT_SAMPLES);
}


// Battery

/**
* Start the on-board battery monitor
*/
void initialiseBattery(){
	readBatteryLevel();
}


/**
* Take readings from the battery charger
* Results are pushed to global variables
*/
int readBatteryLevel(){
	battery.update();
	return battery.getPercentage();
}


//////////////////////////////////////////////////////////////////////////
// Sleep & Power Control

/**
* Put the microcontroller into sleep mode until the sample period has elapsed
* Relies on the RTC using everyMinute interrupts
*/
void enterSleep(){
	sleepNow();
	
	// Sleep Point //
	
	disableSleep();
	while ((RTC.now().minute() % SAMPLE_PERIOD) != 0){
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
	sleepController();
}


/**
* Wake the microcontroller and peripherals from sleep
*/
void wakeUp(){
	DateTime timestamp = RTC.now();
	Log.Info("Awake - %d:%d", timestamp.hour(), timestamp.minute());
	
	disableSleep();
	
	powerUpXbee();
	powerUpSensors();
}


/**
* Disable sleep from occuring
* Pin interrupts are detached and the safety is put back on
*/
void disableSleep(){
	detachInterrupt(RTC_INTERRUPT);
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

