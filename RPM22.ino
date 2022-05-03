#include <LiquidCrystal.h>

#include <U8g2lib.h>


/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/
#include <LiquidCrystal_I2C.h>

//U8G2_KS0108_128X64_1 u8g2(U8G2_R2, 4, 5, 6, 7, 8, 9, 10, 11, 3, 2, 12, 13, A7);

//#define High_Beam_Indicator_Monochrome_12x7_width 12
//#define High_Beam_Indicator_Monochrome_12x7_height 7
//const uint8_t High_Beam[] U8X8_PROGMEM = {
//   0xef, 0x03, 0x20, 0x06, 0x2f, 0x0c, 0x20, 0x08, 0x2f, 0x0c, 0x20, 0x06,
//   0xef, 0x03 };
//
//const uint8_t Battery_Icon[] U8X8_PROGMEM = {
//   0x0c, 0x3f, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x3f, 0x00 };

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
const unsigned long sampleTime = 100 ;
int KM;
int mag = 0;


//#define RangeBaseX 44
//#define RangeBaseY 45
//#define RangeFrameWidth 40
//#define RangeFrameHeight 14

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
   u8g2.begin();
}

int rpm()
{
  int count = 0;
  boolean countFlag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  while (currentTime <= sampleTime)
  {
    if (analogRead(analogInPin) >= 300)
    {
      countFlag = HIGH;
    }
    if (analogRead(analogInPin) < 300 && countFlag == HIGH)
    {
      count++;
      countFlag=LOW;
    }
    currentTime = millis() - startTime;
  }
  //count = int(count/16);
  //int countRpm = int(60000/float(sampleTime))*count/16;
  //Serial.print("lclsdcjkl     ");
  //Serial.println(count/32);
  return count;
}
int temp = 0;
int total = 0;
void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);
  temp = rpm();
  int countRpm = int(60000/float(sampleTime))*temp/16;
  total += temp;
KM = 0.0861*countRpm;




//  boolean countFlag = LOW;
//  unsigned long currentTime = 0;
//  unsigned long startTime = millis();
//  while (currentTime <= sampleTime)
//  {
//    if (analogRead(analogInPin) >= 300)
//    {
//      countFlag = HIGH;
//      digitalWrite(LED_BUILTIN, HIGH);
//    }
//    if (analogRead(analogInPin) < 300 && countFlag == HIGH)
//    {
//      mag++;
//      digitalWrite(LED_BUILTIN, LOW);
//      countFlag=LOW;
//    }
//    currentTime = millis() - startTime;
//  }



  // print the results to the Serial Monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t rpm = ");
 Serial.print(countRpm);
 //Serial.print("       ");
 Serial.print("\tKm/h =");
 Serial.println(KM);
 Serial.print("\tKM =");
 Serial.println(((total/16)*1.51)/1000);
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);

Serial.print("|");
Serial.print((total)/16);

Serial.println("|");


//u8g2.setDrawColor(0);
//    u8g2.drawBox(RangeBaseX, RangeBaseY, RangeFrameWidth, RangeFrameHeight);
//    u8g2.setDrawColor(1);
//    //u8g2.drawFrame(RangeBaseX, RangeBaseY, RangeFrameWidth, RangeFrameHeight);
//    u8g2.setFont(u8g2_font_helvB08_tr);
//    char *KM = " km";
//    strcpy
}
