const int MIC_PIN = A0;
const int SAMPLE_WINDOW = 200;

int pressureLevel;

unsigned long sampleStartTime;

void setup()
{
	Serial.begin(57600);

}

void loop()
{
	int maxLevel = 0;
	
	sampleStartTime = millis();
	while ((millis() - sampleStartTime) < SAMPLE_WINDOW){
		pressureLevel = analogRead(MIC_PIN);
		
		if(pressureLevel > maxLevel){
			maxLevel = pressureLevel;
		}
	}

	
	Serial.print("Sound Pressure Level: ");
	Serial.print(maxLevel, DEC);
	Serial.println(" counts");
}
