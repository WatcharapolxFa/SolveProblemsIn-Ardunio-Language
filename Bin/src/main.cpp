#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
Servo myservo;
// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;

int State = 0;
int IrPin = 8;
int LedRed = 1;
int LedYellow = 2;
int LedGreen = 3;

bool state = false;
bool stateLed = false;

void notification();
void servo();
void readDistance();

void setup()
{
  Serial.begin(115200); // Starts the serial communication
  myservo.attach(A4);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  myservo.write(0);
  pinMode(IrPin, INPUT);
  pinMode(LedRed, OUTPUT);
  pinMode(LedGreen, OUTPUT);
  pinMode(LedYellow, OUTPUT);
  digitalWrite(LedRed, LOW);
  digitalWrite(LedGreen, LOW);
  digitalWrite(LedYellow, LOW);
}
void loop()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  if (distance <= 45)
  {
    Serial.println("Bin on");
    digitalWrite(LedRed, LOW);
    digitalWrite(LedGreen, LOW);
    digitalWrite(LedYellow, HIGH);
    myservo.write(160);
    delay(5000);
    Serial.println("Bin off");
    digitalWrite(LedRed, LOW);
    digitalWrite(LedGreen, LOW);
    digitalWrite(LedYellow, LOW);
    myservo.write(0);
    delay(300);
  }
  int readIr = analogRead(IrPin);
  Serial.print("Ir = ");
  Serial.println(readIr);
  if (readIr == 0)
  {
    digitalWrite(LedRed, HIGH);
    digitalWrite(LedGreen, LOW);
    digitalWrite(LedYellow, LOW);
  }

  else if (readIr == 1)
  {
    digitalWrite(LedRed, LOW);
    digitalWrite(LedGreen, HIGH);
    digitalWrite(LedYellow, LOW);
  }
}
