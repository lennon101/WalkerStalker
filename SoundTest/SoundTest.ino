#define MIC_PIN 0
#define SAMPLE_PERIOD 200

int longAverageCount = 0;
long longAverageMaxTotal = 0;
long longAverageAverageTotal = 0;

void setup()
{
	Serial.begin(9600);
	Serial.println("--------- Sound Test ---------\nStart");
}

void loop()
{
	long startTime = millis();
	int maxLevel = 0;
	long average;
	long total = 0;
	int count = 0;
	
	while(millis() < startTime + SAMPLE_PERIOD){
		int soundLevel = analogRead(MIC_PIN);
		
		total += soundLevel;
		count += 1;
		
		average = (average + float(soundLevel))/2;
		
		if (soundLevel > maxLevel){
			maxLevel = soundLevel;
		}
	}
	
	average = total/count;
	
	Serial.print("Max: ");
	Serial.print(maxLevel);
	Serial.print("    Average: ");
	Serial.print(average);
	Serial.print("    Counts: ");
	Serial.println(count);
	
	if (maxLevel > 0){
		longAverageCount += 1;
		longAverageMaxTotal += maxLevel;
		longAverageAverageTotal += average;
	}
	
	if (longAverageCount >= 20){
		int averageMax = longAverageMaxTotal/longAverageCount;
		int averageAverage = longAverageAverageTotal/longAverageCount;
		Serial.print("-------------------------------------------------");
		Serial.print("Max: ");
		Serial.print(averageMax);
		Serial.print("    Average: ");
		Serial.print(averageAverage);
		Serial.println();
		Serial.println();
		
		longAverageCount = 0;
		longAverageAverageTotal = 0;
		longAverageMaxTotal = 0;
	}
	delay(50);
}
