#include <Arduino.h>

#include <Servo.h>
#include <NewPing.h>
#define TRIGGER_PIN A0  // ตั้งค่าขา pin Arduino ต่อกับขา Sensor Triger
#define ECHO_PIN A1     // ตั้งค่าขา  pin Arduino ต่อกับขา Sensor Echo
#define MAX_DISTANCE 20 // ตั้งค่าระยะการตรวจจับ หน่วยเป็น CM
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

Servo myservo;
int State = 0;
int IrPin = 8;
int LedRed = 1;
int LedYellow = 2;
int LedGreen = 3;

void setup()
{
  Serial.begin(9600);
  myservo.attach(A4);
  pinMode(IrPin, INPUT);
  pinMode(LedRed, OUTPUT);
  pinMode(LedGreen, OUTPUT);
  pinMode(LedYellow, OUTPUT);
  myservo.write(0);
  digitalWrite(LedRed, LOW);
  digitalWrite(LedGreen, LOW);
  digitalWrite(LedYellow, LOW);
}
void loop()
{
  int Sr = sonar.ping_cm();
  Serial.print("Sr = ");
  Serial.println(State);
  int readIr = analogRead(IrPin);
  Serial.print("Ir = ");
  Serial.println(readIr);

  if (readIr >= 100 && State == 0)
  {
    digitalWrite(LedRed, LOW);
    digitalWrite(LedGreen, HIGH);
    digitalWrite(LedYellow, LOW);
  }
  else if (readIr <= 100 && readIr >= 50 && State == 0)
  {
    digitalWrite(LedRed, LOW);
    digitalWrite(LedGreen, LOW);
    digitalWrite(LedYellow, HIGH);
  }
  else if (readIr >= 0 && readIr < 49 && State == 0)
  {
    digitalWrite(LedRed, HIGH);
    digitalWrite(LedGreen, LOW);
    digitalWrite(LedYellow, LOW);
  }

  if (State == 0)
  {
    if (Sr <= 5)
    { // ถ้าระยะเซ็นเซอร์น้อยกว่าหรือเท่ากับ 5 ฝาถังปิด
      myservo.write(0);
      delay(300);
    }
    else if (Sr >= 6)
    { // ถ้าระยะเซ็นเซอร์มากกว่าหรือเท่ากับ 6 ฝาถังเปิด
      myservo.write(160);
      delay(300);
      State = 1;
    }
  }
  if (State == 1)
  {
    delay(3000);
    State = 0;
  }
}