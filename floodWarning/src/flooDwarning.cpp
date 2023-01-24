#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>
//ประกาศตัวแปร
const int pingPin = D1;
int inPin = D2;
int Buzzer_pin = D3 ;

// ประกาศตัวแปรใช้แทนระดับน้ำ
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
const long flood_level_cm[6] = {0, 15, 20 , 25, 30, 35};

#define SENSOR_DISTANCE (32) // ระยะจากพื้นถึง sensor (cm)

// ###################################################################

void setup()
{
  Serial.begin(9600);
  pinMode(Buzzer_pin, OUTPUT);
  digitalWrite(Buzzer_pin, LOW);

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

unsigned char buzzer_timer;

void level_mng(long cm)
{
  // ประกาศตัวแปร
  long level_dif;
  unsigned char flooding_level_buf = flooding_level;
  bool flag_line_send = 0;
  String msg;

  level_dif = SENSOR_DISTANCE - cm;

  Serial.print("Flooding Level: ");
  Serial.print(level_dif);
  Serial.println(" cm");

  if (level_dif < 0)// เป็น 0 
    flooding_level = LEVEL_0;
  else
  {
    // อันนี้ ระดับ  5 
    if (flooding_level == LEVEL_5)
    {
      digitalWrite(Buzzer_pin, LOW);
      if (level_dif <= flood_level_cm[LEVEL_4])
        flooding_level = LEVEL_4;
    }
    // อันนี้ ระดับ  4
    if (flooding_level == LEVEL_4)
    {
      digitalWrite(Buzzer_pin, LOW);
      if (level_dif >= flood_level_cm[LEVEL_5])
        flooding_level = LEVEL_5;
      else if (level_dif <= flood_level_cm[LEVEL_3])
        flooding_level = LEVEL_3;
    }
    // อันนี้ ระดับ  3
    if (flooding_level == LEVEL_3)
    {
      digitalWrite(Buzzer_pin, LOW);
      if (level_dif >= flood_level_cm[LEVEL_4])
        flooding_level = LEVEL_4;
      else if (level_dif <= flood_level_cm[LEVEL_2])
        flooding_level = LEVEL_2;
    }
    // อันนี้ ระดับ  2
    if (flooding_level == LEVEL_2)
    {
      digitalWrite(Buzzer_pin, HIGH);
      if (level_dif >= flood_level_cm[LEVEL_3])
        flooding_level = LEVEL_3;
      else if (level_dif <= flood_level_cm[LEVEL_1])
        flooding_level = LEVEL_1;
    }
    // อันนี้ ระดับ  1
    if (flooding_level == LEVEL_1)
    {
      digitalWrite(Buzzer_pin, HIGH);
      if (level_dif >= flood_level_cm[LEVEL_2])
        flooding_level = LEVEL_2;
      else if (level_dif <= flood_level_cm[LEVEL_0])
        flooding_level = LEVEL_0;
    }

    if (flooding_level == LEVEL_0)
    {
      for (int i = 0; i < 6; i++)
      {
        if (level_dif >= flood_level_cm[i])
          flooding_level = i;
      }
    }
  }
  // ส่่งแจ้งเตือนผ่านไลน์
  if (flooding_level_buf != flooding_level)
  {
    // set status ว่าจะส่งหรือไม่ส่ง
    flag_line_send = 1;
  }
    // พอ = 1 
  if (flag_line_send)
  {
    // ทำเป็น 0 ไม่ให้ส่งรัวๆ
    flag_line_send = 0;
    //  เช็คระดับน้ำแล้วก็ส่งตามที่วัดได้จริง
    if (flooding_level_buf < flooding_level) // level up น้ำขึ้น
    {
      msg = "ระดับน้ำเพิ่มขึ้นเป็นระดับ " + String(flooding_level) + "\r\n" +
            "สูงจากพื้น " + String(level_dif) + " เซ็นติเมตร";
    }
    else // level down น้ำลง
    {
      // อ่านค่าได้ 0 คือไม่มีน้ำแล้ว
      if (flooding_level == 0)
      {
        msg = "น้ำแห้งแล้ว";
      }
      
      else
      {
        msg = "ระดับน้ำลดลงเป็นระดับ " + String(flooding_level) + "\r\n" +
              "สูงจากพื้น " + String(level_dif) + " เซ็นติเมตร";
      }
    }
    // ส่งข้อความ
    LINE.notify(msg);
  }
}

unsigned long ms_buf;
void loop()
{
  // ให้เสียงเงียบ
  digitalWrite(Buzzer_pin, HIGH);
  long duration, cm;
  unsigned long ms_dif;

  ms_dif = millis() - ms_buf;
  // ทำงาน โดยมี mills
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

long microsecondsToCentimeters(long microseconds)
{
  // ความเร็วเสียงในอากาศประมาณ 340 เมตร/วินาที หรือ 29 ไมโครวินาที/เซนติเมตร
  // ระยะทางที่ส่งเสียงออกไปจนเสียงสะท้อนกลับมาสามารถใช้หาระยะทางของวัตถุได้
  // เวลาที่ใช้คือ ระยะทางไปกลับ ดังนั้นระยะทางคือ ครึ่งหนึ่งของที่วัดได้
  return microseconds / 29 / 2;
}
