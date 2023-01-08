#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>

const char *ssid = "iamsurapa24";
const char *pass = "240348131095";

// Your Line notify token
#define LINE_TOKEN "hke5VvMESKm2hjnjtXcjzJjJ07wGMDnAptVXGpT13tL"

const int pingPin = D1;
int inPin = D2;
void setup()
{
 Serial.begin(9600);
 //pinMode(Buzzer_pin, OUTPUT);
 //digitalWrite(Buzzer_pin, BUZZER_OFF);
 
 WiFi.begin(ssid, pass);
 
 while (WiFi.status() != WL_CONNECTED)
 {
   // ทำการ Print "Connectiong..." ทุก 1000ms
   Serial.println("Connecting...  ");
   Serial.printf("Connection Status: %d\n", WiFi.status());
   delay(1000);
 }
 
 Serial.print("Wi-Fi connected.");
 Serial.print("IP Address : ");
 Serial.println(WiFi.localIP());
 
 // กำหนด Line Token
 LINE.setToken(LINE_TOKEN);
 
 // Welcome message
 LINE.notify("เครื่องแจ้งเตือนน้ำท่วมเชื่อมต่อ WiFi แล้ว");
 //flooding_level = LEVEL_0;
}

void loop()
{
    long duration, cm;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
    pinMode(inPin, INPUT);
    duration = pulseIn(inPin, HIGH);
    cm = microsecondsToCentimeters(duration);
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    delay(100);
}
long microsecondsToCentimeters(long microseconds)
{
    // ความเร็วเสียงในอากาศประมาณ 340 เมตร/วินาที หรือ 29 ไมโครวินาที/เซนติเมตร
    // ระยะทางที่ส่งเสียงออกไปจนเสียงสะท้อนกลับมาสามารถใช้หาระยะทางของวัตถุได้
    // เวลาที่ใช้คือ ระยะทางไปกลับ ดังนั้นระยะทางคือ ครึ่งหนึ่งของที่วัดได้
    return microseconds / 29 / 2;
}