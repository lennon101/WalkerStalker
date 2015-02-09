const int micSensor = A1;   // the SPL output is connected to analog pin 0
unsigned long MicroSeconds;

void setup() {
 Serial.begin(115200);
}

void loop() {
  MicroSeconds = micros();
  Serial.print(MicroSeconds);
  Serial.print(",");
  Serial.println(analogRead(micSensor), DEC);
  //delay(10);
}
