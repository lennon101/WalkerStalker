#include <SHT15.h>

#define CLOCK_PIN 6
#define DATA_PIN 7
SHT15 humiditySensor = SHT15(CLOCK_PIN, DATA_PIN);

void setup()
{
Serial.begin(57600);
Serial.println("SHT15 Test");

}

void loop()
{

float temperature = humiditySensor.readTemperature();
float humidity = humiditySensor.readHumidity();

Serial.print("Temperature: ");
Serial.println(temperature, 2);
Serial.print("Humidity: ");
Serial.println(humidity);

delay(500);
}
