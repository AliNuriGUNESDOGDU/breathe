/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground through 220 ohm resistor

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int iter = 0;
int send = 0;
int sensorArray[8] = {0,0,0,0,0,0,0,0};

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 124;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  sensorArray[iter] = analogRead(analogInPin);
  if(iter > 7)
  {
    iter = 0;
    send = 1;
  }
  else
  {
    iter ++;
  }
}

void loop() {
  if(send)
  {
    for(int i = 0; i<7; i++)
    {
      Serial.print(sensorArray[i]);
      Serial.print(",");
    }
    Serial.println(sensorArray[7]);
    send = 0;
  }
  //delay(2);
}
