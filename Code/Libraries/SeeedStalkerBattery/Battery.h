/* 
Battery.cpp (Version 0.2) - Library for Battery infos on Seeeduino Stalker V2.3 
Created by Stefan, March 2013.

Notes: small library to:
 - read lipo battery voltage -> analog pin 7
 - current capacity (in %) 
 - charging status -> analog pin 6
 - flashing LED for battery indication 
*/

#ifndef Battery_h
#define Battery_h

#include "Arduino.h"

class Battery
{
  public:
    void update();
    void ledflashStatus(uint8_t pin=13,uint8_t flashfull=5);    
    float getVoltage();
    uint8_t getPercentage();
    char* getChStatus();
    bool isCharging();
  private:
    int batteryValue;    
};

#endif
