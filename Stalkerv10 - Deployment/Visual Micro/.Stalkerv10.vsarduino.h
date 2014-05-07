/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: ATtiny85 @ 8 MHz  (internal oscillator; BOD disabled), Platform=avr, Package=tiny
*/

#define __AVR_ATtiny85__
#define __AVR_ATTINY85__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 8000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
//
void initialiseTemperatureSense();
void initialiseHumiditySense();
void initialiseLightSense();
void initialiseXBee();
void initialiseDatalog();
void readTemperature();
void readHumidity();
void readLuminosity();
void readSound();
void readCurrent();
void readBatteryCapacity();
int floatToInt(float num, int decimalShift);
void printTimeStamp(DateTime timeStamp);
float getCorrectHTU21DHumidity(float temperature, float humidity);
int getSoundLevel(int samplePeriod);
void sleepNow();
void wakeUp();
void disableSleep();
void sleepController();
void powerDownXbee();
void powerUpXbee();
void powerDownTF();
void powerUpTF();
void writeDataToLog();
void writeDataToSerial();
void clockInterrupt();

#include "c:\Program Files (x86)\Arduino\hardware\tiny\avr\cores\tiny\arduino.h"
#include "E:\Dropbox\Projects\WalkerStalker\Stalkerv10\Stalkerv10.ino"
