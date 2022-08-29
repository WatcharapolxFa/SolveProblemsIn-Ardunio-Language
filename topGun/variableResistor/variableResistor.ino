int led = 3;
int vr = A5;
void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  int val = analogRead(vr); 
  int output = map(val, 0, 1023, 0, 255); 
  Serial.println(output);
  analogWrite(led, output);
  delay(300);
}
