#include <Arduino.h>
#include "SevSeg.h"
SevSeg sevseg; // Instantiate a seven segment controller object
int buttonPin = A0;
int checkbt = 0;
int buttonState = 0;
int total = 120;
void setup()
{
  Serial.begin(115200);
  byte numDigits = 4;
  byte digitPins[] = {2, 3, 4, 5};
  byte segmentPins[] = {6, 7, 8, 9, 10, 11, 12, 13};
  bool resistorsOnSegments = false;   
  byte hardwareConfig = COMMON_ANODE; 
  bool updateWithDelays = false;      
  bool leadingZeros = false;          
  bool disableDecPoint = false;       
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
               updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(90);
}

void loop()
{
  int buzzer = 0;
  checkbt = analogRead(buttonPin);// read sensor
  if (checkbt == 0)//if the value is 0
  {
    buttonState = 1;
    total = 120;// set in seconds
  }
  if (buzzer == 3)
  {
    tone(14, 262, 250);
    delay(300);
    buzzer = 0;
  }
  if (buttonState == 1)
  {
    static unsigned long timer = millis();
    static int deciSeconds = 0;
    int minute = 0;
    int second = 0;

    if (millis() - timer >= 1000)
    {
      timer += 1000;
      deciSeconds++; // 100 milliSeconds is equal to 1 deciSecond

      if (deciSeconds == 1000)
      { // Reset to 0 after counting for 1000 seconds.
        deciSeconds = 0;
      }
      total--;

      minute = (total / 60) * 100;
      second = total % 60;
      int ans = minute + second;
      sevseg.setNumber(ans, 0);
      if (ans <= 0)
      {
        buzzer = 3;
        buttonState = 0;
        total = 120;
      }
    }
    sevseg.refreshDisplay();
  }
}

/// END ///
