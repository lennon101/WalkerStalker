#include <SPI.h>
#include <SD.h>

#define CS_PIN 10

File dataFile;
const char filename[] = "nope.csv";

uint8_t num = 0;

void setup()
{
	pinMode(CS_PIN, OUTPUT);
	Serial.begin(9600);

	while(!SD.begin(CS_PIN)){
		Serial.println("Fook'd cap'n");
		delay(1000);
	}

	dataFile = SD.open(filename, O_WRITE | O_APPEND | O_CREAT);
	if (!dataFile){
		Serial.println("Still fook'd cap'n");
		while(1);
	}
	
	dataFile.println("Things,Not Things");

}

void loop()
{
	dataFile.print(num);
	dataFile.print(',');
	dataFile.println(num*num);
	
	dataFile.flush();
	
	num += 1;

}
