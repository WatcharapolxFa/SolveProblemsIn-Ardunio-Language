#include <ESP8266WiFi.h>
#include <TridentTD_LineNotify.h>

const char *ssid = "iamsurapa24";
const char *pass = "240348131095";

// Your Line notify token
#define LINE_TOKEN "hke5VvMESKm2hjnjtXcjzJjJ07wGMDnAptVXGpT13tL"

const int pingPin = D1;
int inPin = D2;

int Relay1 = D3;
#define BUZZER_OFF 1
#define BUZZER_ON 0

unsigned char flooding_level;

/* ตั้งค่าระดับน้ำ (cm) ->  index 0 : ไม่มีน้ำท่วม
                     index 1 : น้ำท่วมระดับ 1
                     index 2 : น้ำท่วมระดับ 2
                     index 3 : น้ำท่วมระดับ 3
                     index 4 : น้ำท่วมระดับ 4
                     index 5 : น้ำท่วมระดับ 5  */
const long flood_level_cm[6] = {0, 10, 20, 30, 40, 50};

#define SENSOR_DISTANCE (40) //(100) // ระยะจากพื้นถึง sensor (cm)

void setup()
{
  Serial.begin(9600);
  pinMode(Relay1, OUTPUT);
  digitalWrite(Relay1, BUZZER_OFF);
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
  flooding_level = 0;
}

unsigned char buzzer_timer;

void level_mng(long cm)
{
  long level_dif;
  // flooding_level_buf = 0
  unsigned char flooding_level_buf = flooding_level;
  bool flag_line_send = 0;
  String msg;
  // SENSOR_DISTANCE = ระยะห่างของ sensor ที่เราตั้งไว้(ความสูง)
  level_dif = SENSOR_DISTANCE - cm;

  Serial.print("Flooding Level: ");
  // แสดงผลบรรทัดที่ 62
  Serial.print(level_dif);
  Serial.println(" cm");
  // ไม่่มีทางเข้าเงื่อนไขนี้เพราะค่าเริ่มต้นเป็น 0 ไม่มีทางน้อยกว่า
  if (level_dif < 0)
    flooding_level = 0;
  else
  {
    if (flooding_level == 5)
    {
      if (level_dif <= flood_level_cm[4])
        flooding_level = 4;
    }

    if (flooding_level == 4)
    {
      if (level_dif >= flood_level_cm[5])
        flooding_level = 5;
      else if (level_dif <= flood_level_cm[3])
        flooding_level = 3;
    }

    if (flooding_level == 3)
    {
      if (level_dif >= flood_level_cm[4])
        flooding_level = 4;
      else if (level_dif <= flood_level_cm[2])
        flooding_level = 2;
    }

    if (flooding_level == 2)
    {
      if (level_dif >= flood_level_cm[3])
        flooding_level = 3;
      else if (level_dif <= flood_level_cm[1])
        flooding_level = 1;
    }

    if (flooding_level == 1)
    {
      if (level_dif >= flood_level_cm[2])
        flooding_level = 2;
      else if (level_dif <= flood_level_cm[0])
        flooding_level = 0;
    }

    if (flooding_level == 0)
    {
      for (int i = 0; i < 6; i++)
      {
        if (level_dif >= flood_level_cm[i])
          flooding_level = i;
      }
    }
  }

  if (flooding_level_buf != flooding_level)
  {
    flag_line_send = 1;
  }

  if (flooding_level == 5)
  {
    if (buzzer_timer <= 5) // 1 sec
    {
      digitalWrite(Relay1, BUZZER_ON);
    }
    else if (buzzer_timer <= 10)
    {
      digitalWrite(Relay1, BUZZER_OFF);
    }
    else
      buzzer_timer = 0;

    buzzer_timer++;
  }
  else
  {
    digitalWrite(Relay1, BUZZER_OFF);
    buzzer_timer = 0;
  }

  if (flag_line_send)
  {
    flag_line_send = 0;

    if (flooding_level_buf < flooding_level) // level up
    {
      msg = "ระดับน้ำเพิ่มขึ้นเป็นระดับ " + String(flooding_level) + "\r\n" +
            "สูงจากพื้น " + String(level_dif) + " เซ็นติเมตร";
    }
    else // level down
    {
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
    LINE.notify(msg);
  }
}

unsigned long ms_buf;
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
    // cm คือค่าที่อ่านได้จาก sensor แล้วเราส่งค่าเข้าไปที่ function level_mng
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
