/* 
Battery.cpp (Version 0.2) - Library for Battery infos on Seeeduino Stalker V2.3 
Created by Stefan, March 2013.

Notes: 
 - read lipo battery voltage -> analog pin 7
 - current capacity (in %) 
 - charging status -> analog pin 6
 - flashing LED for battery indication 
*/

#include <Battery.h>

int LEDPin=13;
int flashesforfull=10; // 1 blink =10%
int chcnt=0;

Battery battery;

void setup(){
  Serial.begin(57600);
  Serial.println("Battery Library for Seeeduino Stalker V2.3");
}

void loop(){
  battery.update();
  battery.ledflashStatus(LEDPin,flashesforfull);
  float voltage = battery.getVoltage();
  int percentage = battery.getPercentage();
  char* CS = battery.getChStatus();
  bool ch = battery.isCharging();
  if(ch) chcnt++;
  
  Serial.print("battery: ");
  Serial.print(voltage);
  Serial.print("V  -> ");
  Serial.print(percentage);
  Serial.print("%     Charge Status: ");
  Serial.print(CS);
  Serial.print("     charging counter: ");
  Serial.println(chcnt);
  delay(2000);
  
}


