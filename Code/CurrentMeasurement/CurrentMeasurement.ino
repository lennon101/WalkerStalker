
#define CURRENT_PIN A0
#define VOLTAGE_REF 0.9855
#define HIGH_VOLTAGE 4.902

void setup()
{
	Serial.begin(115200);
	pinMode(CURRENT_PIN, INPUT);
	analogReference(INTERNAL1V1);
}

void loop()
{
	long now = millis();
	
	int currentReading = analogRead(CURRENT_PIN);
	float current = (float(currentReading)/1023)*VOLTAGE_REF;
	Serial.print(now);
	Serial.print(",");
	Serial.print(currentReading);
	Serial.print(",");
	Serial.println(current, 4);
	Serial.flush();
}
