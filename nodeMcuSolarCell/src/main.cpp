#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);


const char *ssid = "NameWifi";
const char *pass = "PassWifi";

// Your Line notify token
#define LINE_TOKEN "LINE_TOKEN"

void setup()
{
  Serial.begin(9600);
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
  LINE.notify("เครื่องแจ้งเตือนการชาร์จ แบตเตอรี่สำรองพร้อมแล้ว");
    Serial.begin(9600);
  Serial.println("test");
  tcs.begin();
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
}
}

void loop(void) {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);
  delay(1000);
  String msg;
  if (b > 2000  ) // level up
  {
    msg = "แบตเตอรี่สำรองพร้อมแล้ว" ;
  }
  else if (b >= 1900 && b <= 2000)
  {
    msg = "แบตเตอรี่สำรองใกล้เต็มแล้วอย่าลืมมาดูน้า" ;
  }
  
  LINE.notify(msg);
  delay(1000);
}
