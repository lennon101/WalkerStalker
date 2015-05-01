// Date and time functions using a RX8025 RTC connected via I2C and Wire lib

#include <Wire.h>
#include "DS3231.h"

DS3231 RTC; //Create the DS3231 object

char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

//year, month, date, hour, min, sec and week-day(starts from 0 and goes to 6)
//writing any non-existent time-data may interfere with normal operation of the RTC.
//Take care of week-day also.
DateTime dt(2015, 1, 30, 11, 02, 0, 5);


void setup ()
{
	Serial.begin(57600);
	Wire.begin();
	RTC.begin();
	
	Serial.print("Setting time to: ");
	Serial.print(dt.year(), DEC);
	Serial.print('/');
	Serial.print(dt.month(), DEC);
	Serial.print('/');
	Serial.print(dt.date(), DEC);
	Serial.print(' ');
	Serial.print(dt.hour(), DEC);
	Serial.print(':');
	Serial.print(dt.minute(), DEC);
	Serial.print(':');
	Serial.print(dt.second(), DEC);
	Serial.println();
	Serial.print(weekDay[dt.dayOfWeek()]);
	Serial.println();
	Serial.println("\nHit any key to apply this time");
	
	// Wait until character received before resetting time
	while(!Serial.available()){
	}
	
	RTC.adjust(dt); //Adjust date-time as defined 'dt' above
	
}

void loop ()
{
	DateTime now = RTC.now(); //get the current date-time
	Serial.print(now.year(), DEC);
	Serial.print('/');
	Serial.print(now.month(), DEC);
	Serial.print('/');
	Serial.print(now.date(), DEC);
	Serial.print(' ');
	Serial.print(now.hour(), DEC);
	Serial.print(':');
	Serial.print(now.minute(), DEC);
	Serial.print(':');
	Serial.print(now.second(), DEC);
	Serial.println();
	Serial.print(weekDay[now.dayOfWeek()]);
	Serial.println();
	delay(1000);
}
