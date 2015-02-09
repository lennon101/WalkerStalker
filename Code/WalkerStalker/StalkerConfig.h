/*
* StalkerConfig.h
*
* Created: 15/01/2015 15:29:46
*  Author: Leenix
*/
#include "Arduino.h"

#ifndef STALKERCONFIG_H_
#define STALKERCONFIG_H_

//////////////////////////////////////////////////////////////////////////
// Unit-specific variables

const byte UNIT_NUMBER = 12;

// Minutes between samples
const byte SAMPLE_PERIOD = 10;

// Serial output level
const byte LOGGER_LEVEL = LOG_LEVEL_DEBUG;


//////////////////////////////////////////////////////////////////////////
// Pin Assignments

const byte SENSOR_POWER_PIN = A2;
const byte XBEE_PWR_PIN = 5;	// XBee radio power
const byte TF_PWR_PIN = 4;	// Trans-flash (SD) card power
const byte TEMP_PROBE_PIN = 9;
const byte CHIP_SELECT_PIN = 10;	// SD Card
const byte CURRENT_SENSE_PIN = A1;	// Analog - A1
const byte MIC_PIN = A0;			// Analog - A0
const byte RHT_PIN = 3;

//////////////////////////////////////////////////////////////////////////
// Sensor settings

// Temperature - DS18B20
const byte TEMPERATURE_PRECISION = 12;	// Precision of temperature readings in bits


// Temperature - TMP006
const byte TMP006_ADDRESS = 0x44;


//Humidity


// Sound
const int MIC_SAMPLE_PERIOD = 100;	// Sampling window for microphone in milliseconds


// Illuminance
const byte TSL2561_ADDRESS = TSL2561_ADDR_FLOAT;


// Current
const int TRANSFORMER_RATIO = 2000;	// The ratio of the current transformer
const int BURDEN_RESISTOR = 220;		// Value of the burden resistor in Ohms
const int CURRENT_SAMPLES = 1000;	// Number of ADC samples taken when measuring current
const float CURRENT_CALIBRATION_FACTOR = TRANSFORMER_RATIO/BURDEN_RESISTOR;


//////////////////////////////////////////////////////////////////////////
// Communication

// Wired - Serial
const long SERIAL_BAUD = 57600;

const long LOGGER_BAUD = 9600;
const byte LOGGER_SERIAL_TX = 12;
const byte LOGGER_SERIAL_RX = 11;

// Wireless - XBee
XBeeAddress64 COORDINATOR_ADDRESS = XBeeAddress64(0x0013a200, 0x40abbade);
const byte XBEE_MAX_RETRIES = 5;	// Number of retries possible
const int XBEE_ACK_TIMEOUT = 1000;	// Timeout in milliseconds
const int XBEE_READ_TIMEOUT = 1000;

// Misc 
const byte PACKET_BUFFER_SIZE = 100;		// Number of bytes in the packet buffer
const byte COMMAND_CACHE_SIZE = 32;

const byte DEFAULT_DECIMAL_PLACES = 2;	// Number of decimal places conserved in float>>int conversions
const int COMMS_DELAY = 300;				// Wait time after sending packets through serial or SPI
const int XBEE_WAKE_DELAY = 1000;

const byte SAMPLE_UPTIME = 5;	// Length of time that the system stays awake after a sample (for transmission reasons) in seconds

// Commands
const char HOLD_AWAKE = 'H';

//////////////////////////////////////////////////////////////////////////
// Power/Sleep

const byte RTC_INTERRUPT = INT0;

//////////////////////////////////////////////////////////////////////////
// DS18B20 Address assignments

byte airTemperatureAddresses[][8] = {
	{0x28, 0x7C, 0xD0, 0x30, 0x05, 0x00, 0x00, 0xD8},	// Unit 00
	{0x28, 0x47, 0xC9, 0xF7, 0x04, 0x00, 0x00, 0x56},	// Unit 01
	{0x28, 0xCC, 0xD8, 0x12, 0x05, 0x00, 0x00, 0x96},	// Unit 02
	{0x28, 0x89, 0x39, 0x22, 0x05, 0x00, 0x00, 0x57},	// Unit 03
	{0x28, 0x55, 0x69, 0x22, 0x05, 0x00, 0x00, 0x7B},	// Unit 04
	{0x28, 0xCD, 0x13, 0x23, 0x05, 0x00, 0x00, 0x34},	// Unit 05
	{0x28, 0xB3, 0x17, 0x22, 0x05, 0x00, 0x00, 0x2F},	// Unit 06
	{0x28, 0x59, 0x68, 0x22, 0x05, 0x00, 0x00, 0xCB},	// Unit 07
	{0x28, 0x55, 0x9F, 0x22, 0x05, 0x00, 0x00, 0x41},	// Unit 08
	{0x28, 0xAC, 0xBC, 0x22, 0x05, 0x00, 0x00, 0x13},	// Unit 09
	{0x28, 0x05, 0xBD, 0x22, 0x05, 0x00, 0x00, 0x14},	// Unit 10
	{0x28, 0xCE, 0x17, 0x22, 0x05, 0x00, 0x00, 0xFD},	// Unit 11
	{0x28, 0x24, 0x5B, 0x22, 0x05, 0x00, 0x00, 0xD3},	// Unit 12
	{0x28, 0x0E, 0x50, 0x22, 0x05, 0x00, 0x00, 0xDA},	// Unit 13
	{0x28, 0xF8, 0x92, 0x22, 0x05, 0x00, 0x00, 0xBB}	// Unit 14
};

DeviceAddress wallTemperatureAddresses[] = {
	{0x28, 0xC6, 0x3C, 0x04, 0x05, 0x00, 0x00, 0xAD},	// Unit 00
	{0x28, 0xBF, 0x4A, 0x22, 0x05, 0x00, 0x00, 0x2B},	// Unit 01
	{0x28, 0x3C, 0xA1, 0x22, 0x05, 0x00, 0x00, 0x32},	// Unit 02
	{0x28, 0x26, 0x31, 0x22, 0x05, 0x00, 0x00, 0x11},	// Unit 03
	{0x28, 0x27, 0x00, 0x23, 0x05, 0x00, 0x00, 0xE0},	// Unit 04
	{0x28, 0x91, 0x52, 0x22, 0x05, 0x00, 0x00, 0xCC},	// Unit 05
	{0x28, 0x65, 0x87, 0x22, 0x05, 0x00, 0x00, 0xEE},	// Unit 06
	{0x28, 0x97, 0x9E, 0x22, 0x05, 0x00, 0x00, 0x7D},	// Unit 07
	{0x28, 0x47, 0xA1, 0x22, 0x05, 0x00, 0x00, 0x52},	// Unit 08
	{0x28, 0x3C, 0x58, 0x22, 0x05, 0x00, 0x00, 0x67},	// Unit 09	
	{0x28, 0x9E, 0x58, 0x22, 0x05, 0x00, 0x00, 0x55},	// Unit 10
	{0x28, 0x2B, 0x47, 0x22, 0x05, 0x00, 0x00, 0xAA},	// Unit 11
	{0x28, 0x51, 0x47, 0x22, 0x05, 0x00, 0x00, 0xFD},	// Unit 12
	{0x28, 0x81, 0x1A, 0x22, 0x05, 0x00, 0x00, 0x40},	// Unit 13
	{0x28, 0xB8, 0xF6, 0x22, 0x05, 0x00, 0x00, 0xC0}	// Unit 14
};

//////////////////////////////////////////////////////////////////////////

const String UNIT_CLASS = "Walker";
const String UNIT_ID = UNIT_CLASS + UNIT_NUMBER;

#endif /* STALKERCONFIG_H_ */