#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial BTSerial(2,3);

//TM1637
#define CLK 6
#define DIO 7
TM1637Display display(CLK, DIO);

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0; 

float beatsPerMinute;
int beatAvg;
int beatAvglast=0;

int buzzerPin = 2;
int redPin =11;
int greenPin =12;
int bluePin=13;

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  Serial.println("Initializing...");
  display.setBrightness(0x0f); //밝기를 최대로 해라
  display.showNumberDec(0, false);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  lcd.init();
  lcd.backlight();
}

void loop()
{
  long irValue = particleSensor.getIR();
  if(BTSerial.available()){
    Serial.write(BTSerial.read());
  }
  if(Serial.available()){
    BTSerial.write(Serial.read());
  }

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      

      if (irValue < 50000){
        display.showNumberDec(0, false);
      }else{
        display.showNumberDec(beatAvg, false);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.println(beatAvglast);
        beatAvglast = beatAvg;
        if(beatAvg < 60 || beatAvg >100){
          // 코드 채워넣어야함 ! 비정상일 때 어떻게 해야 하는지...
        }
      }
    }
  }
}

void setColor (int red, int green, int blue){
  analogWrite(redPin,red);
  analogWrite(greenPin,green);
  analogWrite(bluePin,blue);
}
