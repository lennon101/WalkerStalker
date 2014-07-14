/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Pro or Pro Mini w/ ATmega328 (3.3V, 8 MHz), Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 8000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
//
void enableWatchdog();
void disableWatchdog();
void initialiseSensors();
void initialiseTemperatureSense();
void initialiseHumiditySense();
void initialiseLightSense();
void initialiseXBee();
void initialiseDatalog();
void enterSleep();
void readTemperature();
void readHumidity();
void readLuminosity();
void readSound();
void readCurrent();
void readBatteryCapacity();
int floatToInt(float num, int decimalShift);
void printTimeStamp(DateTime timeStamp);
int getSoundLevel(int samplePeriod);
void sleepNow();
void wakeUp();
void disableSleep();
void sleepController();
void powerDownXbee();
void powerUpXbee();
void powerDownTF();
void powerUpTF();
void powerDownSensors();
void powerUpSensors();
void writeDataToLog();
void writeDataToSerial();
void transmitData();
void writeDataToPacketBuffer();
void writeTimeToBuffer();
void clockInterrupt();
void toBuffer(unsigned int num);
void toBuffer(int num);
void toBuffer(byte b);
byte fromBuffer();
void resetBuffer();

#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h" 
#include "c:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\arduino.h"
#include "E:\Dropbox\Projects\WalkerStalker\Stalkerv20\Stalkerv20.ino"
