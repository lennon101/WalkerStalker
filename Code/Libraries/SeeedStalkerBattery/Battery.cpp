/* 
 Battery.cpp (Version 0.2) - Library for Battery infos on Seeeduino Stalker V2.3 
Created by Stefan, March 2013.

Notes: small library to:
 - read lipo battery voltage -> analog pin 7
 - current capacity (in %) 
 - charging status -> analog pin 6
 - flashing LED for battery indication 

Voltage:
sensorValue * (1.1 / 1024)* (10+2)/2 = 1/155  ->Umax=6.6V

**********************************************************
For better precision measure your AREF Pin with a Voltmeter
myAREF=1.093
->myfact1=(1.093/1024)=0.0010673828125

for percentagefact = myfact1*100 = 0.10673828125
for voltfact = myfact1*(12/2) = 0.006404296875
for MinBatVoltage=3.6/voltfact = 562
for flashesfact=myfact*10=0.010673828125
**********************************************************


Percentage:
formula from http://www.westknoxrc.com/index.php?topic=182.0

(BatVoltage - MinBatVoltage)*100/(MaxBatVoltage-MinBatVoltage)
= (BatteryVoltage -3.6)/(4.2-3.6)*100
=(BattValue-(3.6/voltfact)) * (voltfact*100)/(0.6)
*********************************************************
*/

#include "Battery.h"

/////// UPDATE ///////
void Battery::update(){
	analogReference(INTERNAL);
	batteryValue=analogRead(A7);
	return;
}

//////// BATTERY VOLTAGE ////////
float Battery::getVoltage() {
	return (batteryValue*0.006404296875); 
}

//////// BATTERY PERCENTAGE /////////
uint8_t Battery::getPercentage(){
	return (batteryValue-562)*1.06738;	
}

//////// CH_STATUS /////////
char* Battery::getChStatus()
{
  analogReference(INTERNAL);
  int ADC6=analogRead(A6);
  if(ADC6>900)
  {
    return "off" ;//sleeping
  }
  else if(ADC6>550)
  {
    return "charging" ;//charging
  }
  else if(ADC6>350)
  {
    return "done";//done
  }
  return "err";//error
 }

////////// CHARGING ///////////
bool Battery::isCharging(){
	analogReference(INTERNAL);
	int ADC6=analogRead(A6);
	if(ADC6>550 && ADC<900) return true;
	else return false;
}
   

//////// LED BATTERY INDICATION /////////
void Battery::ledflashStatus(uint8_t pin,uint8_t flashfull){
	pinMode(pin,OUTPUT);
	uint8_t flashes =(batteryValue-562)*0.01067*flashfull;
	for(uint8_t k=0;k<flashes;k++){ 
		digitalWrite(pin, HIGH);
		delay(50);  //50
		digitalWrite(pin, LOW);
		delay(100);  //100
		if(k&1) delay(250);	
	}
	return;
}
