#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>

const int pingPin = D1;
int inPin = D2;
#define Buzzer_pin D3

#define BUZZER_OFF 1
#define BUZZER_ON 0

#define LEVEL_0 0
#define LEVEL_1 1
#define LEVEL_2 2
#define LEVEL_3 3
#define LEVEL_4 4
#define LEVEL_5 5

unsigned char flooding_level;

// ################# What you need to modify #########################

// Your WiFi credentials.
const char *ssid = "iamsurapa24";
const char *pass = "240348131095";

// Your Line notify token
#define LINE_TOKEN "hke5VvMESKm2hjnjtXcjzJjJ07wGMDnAptVXGpT13tL"

/* ตั้งค่าระดับน้ำ (cm) ->  index 0 : ไม่มีน้ำท่วม
                      index 1 : น้ำท่วมระดับ 1
                      index 2 : น้ำท่วมระดับ 2
                      index 3 : น้ำท่วมระดับ 3
                      index 4 : น้ำท่วมระดับ 4
                      index 5 : น้ำท่วมระดับ 5  */
const long flood_level_cm[6] = {0, 10, 20, 30, 40, 50};

#define SENSOR_DISTANCE (81) //(100) // ระยะจากพื้นถึง sensor (cm)

// ###################################################################

void setup()
{
    Serial.begin(9600);
    pinMode(Buzzer_pin, OUTPUT);
    digitalWrite(Buzzer_pin, BUZZER_OFF);

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
    flooding_level = LEVEL_0;
}

void loop()
{
    long duration, cm;
    unsigned long ms_dif;

    ms_dif = millis() - ms_buf;

    if (ms_dif >= 200) // 200 ms
    {
        ms_buf = millis();
        pinMode(pingPin, OUTPUT);
        digitalWrite(pingPin, LOW);
        delayMicroseconds(2);
        digitalWrite(pingPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pingPin, LOW);
        pinMode(inPin, INPUT);
        duration = pulseIn(inPin, HIGH);
        cm = microsecondsToCentimeters(duration);
        /*  Serial.print(cm);
          Serial.println("cm");*/
        level_mng(cm);
    }
}
