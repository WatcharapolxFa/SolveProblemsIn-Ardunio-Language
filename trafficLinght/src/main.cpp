#include <Arduino.h>

#define analogMin 521
#define analogMax 525
int relay = 8; // กำหนดขาควบคุม Relay
int Senser = A0;
int val = 0;
int count = 0;
int analogAverage = (analogMin + analogMax) / 2;
void onoffLed()
{
  digitalWrite(relay, HIGH);
  int sensorValue = analogRead(Senser);

  if (sensorValue >= analogMin && sensorValue <= analogMax)
  {
    sensorValue = analogAverage;
  }

  Serial.println(sensorValue);

  if (sensorValue > 900)
  {
    count = count + 1;
  }
  if (count == 1)
  {
    digitalWrite(relay, LOW);
  }
  if (count == 2)
  {
    digitalWrite(relay, HIGH);
    delay(500);
    count = 0;
  }
  delay(100);
}
void setup()
{
  Serial.begin(9600);
  pinMode(relay, OUTPUT);
  pinMode(Senser, INPUT);
}
void loop()
{
  onoffLed();
}
