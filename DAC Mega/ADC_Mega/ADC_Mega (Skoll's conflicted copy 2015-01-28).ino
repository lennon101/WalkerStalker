/*
ADC Calibrator for Arduino Mega2560

Written by Dr Scott Mills 
31 May 2014

This sketch returns ADC readings as actual millivolt inputs using a calibrated intercept and slope.
The intercept and slope must be determined from a high precision multimeter.
To do this create 2 voltage dividers tied to the 3V3 rail of an Arduino.

Voltage divider 1
The first will use a 3K9 resistor and 25 turn 100 Ohm variable resistor.
This will give a suitable output range for testing the lower input voltages of the ADC.

Voltage divider 2
The second will use a 3K9 resistor and a 25 Turn 2K variable resistor.
This will give a suitable output range for testing the upper input voltages of the ADC.

The output of these voltage dividers must be fed to an analogue input pin and a multimeter.
To calibrate your ADC determine the midpoint of the 0 and 1023 readings as follows:

Voltage at the midpoint between 0 and 1 ADC readings
Turn the trim of voltage divider 1 and record the transition point between an ADC reading of 0 and 1 in mV using the multimeter, we will call this L1.
Now record the transition point between an ADC reading of 1 and 2 in mV using the multimeter, we will call this L2.
Your estimate of the midpoint between the ADC offset and the point at which the ADC transitions to 1 in mV (MPL)can be calculated as MPL=L1-(L2-L1)/2)

Now do the same for the high point

Turn the trim of voltage divider 2 and record the transition point between an ADC reading of 1022 and 1023 in mV using the multimeter, we will call this L1023.
Now record the transition point between an ADC reading of 1021 and 1022 in mV using the multimeter, we will call this L1022.
Your estimate of the midpoint of the highest ADC reading and the voltage reference (MPH) can be calculated as MPH=L1023+(L1023-L1022)/2)

So you should now have the following data:
In the middle of ADC reading 0 the actual number of mV is MPL
In the middle of ADC reading 1023 the actual number of mV is MPH

For example

0 = 5.16mV
1023 = 1071.1mV

Now calculate the slope and intercept for these values.

slope = (5.16-1071.1)/(0-1023)
Slope = 1.0419745845552297165200391006843

The intercept is a little easier as I already know the value at an ADC reading of 0

Intercept = 5.16mV 

Why, I hear you ask, did I calculate the mV in the middle of the ADC reading? 
Well when I put my calibration in I want to know that I will return the midpoint of that ADC reading.
That way I'm only ever out by half and ADC reading so my reading should be close to +/- 0.5mV of the actual reading. 

In practice the precision of the calibrated value will be affected by:
Stability of the voltage reference used by the ADC
Non-linearity of the ADC
The accuracy and resolution of your mV measurements

It is advisable to place a 200nF capacitor between Vref and GND.

*/
  
unsigned long MicroSeconds;
int DAC0 = 0;
int DACVal = 0;
float mV= 0;
const float Intercept = 5.16; //Place your estimated intercept value here
const float Slope = 1.041974585; //Place your estimated slope value here
const int Intercept1 = 516;
const int Slope1 = 10419;

void setup() {
  Serial.begin(115200); // Open up the serial port at full speed
  analogReference(INTERNAL1V1); // Set the ADC voltage reference to 1.1 volts
}

void loop() {
  MicroSeconds = micros();
  DACVal = analogRead(DAC0);
  mV = ((DACVal*Slope1)/10000+(Intercept1/100),1);
  Serial.print(MicroSeconds);// Time stamps each mV reading so that you can accurately plot out how the voltage changes over time
  //Serial.print(","); //CSV, uncomment this code during calibration so you can view the raw ADC reading
  //Serial.print(DACVal); //Uncomment this code during calibration so you can view the raw ADC reading
  Serial.print(","); //CSV
  Serial.print(mV);
  Serial.print(",");
  if ((DACVal< 1023) && (DACVal > 0)) //Look to see if the DAC input is greater than 0 but less than 1023 otherwise we will send a descriptive message to let us know we are at the edges of the ADC range
  {
    Serial.print((DACVal*Slope)+Intercept,1); //Correction for slope and intercept with the number of decimal places set to 1
  }
  else if (DACVal >= 1023) // If the reading is 1023 then we will output High to the serial port so that we know we are at the upper limit of the ADC 
  {
    Serial.print("High");
  }
  else if (DACVal <= 0) // If the reading is 0 then we will output Low to the serial port so that we know we are at the Lower limit of the ADC
  {
    Serial.print("Low");
  }
  Serial.println();
  delay(0);// This sketch will make 840 readings per second to the serial port change this value to decrease the sample rate.
}
