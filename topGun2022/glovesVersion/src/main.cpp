#include <Arduino.h>
#include <M5StickCPlus.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
// #include <Adafruit_ADS1X15.h>
// #include <Adafruit_I2CDevice.h>
#include <M5StickCPlus.h>
#include <PubSubClient.h>
#include <Arduino_JSON.h>
#include "config.h"
#include <WiFi.h>

// millis
long long lastSend;
long long sendDelay = 1000; // 100 ms
long long lastMqttReconnect;
long long mqttReconnectDelay = 5000; // 5 s
long long lastReadPotentiometer;
long long readPotentiometerDelay = 100; // 100 ms

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwmServo = Adafruit_PWMServoDriver();
#define SERVOMIN 100  // Minimum pulse length count out of 4096.
#define SERVOMAX 2400 // Maximum pulse length count out of 4096.
int servoNo = 3;      // Defines a counter for servos. count0-n
int servoNoMax = 3;   // maximum n servo
int checkStatusServo = 0;
float accX = 0.0F, accY = 0.0F, accZ = 0.0F, gyroX = 0.0F, gyroY = 0.0F, gyroZ = 0.0F, pitch = 0.0F, roll = 0.0F, yaw = 0.0F;
float remindGyroX = 0.0F, remindGyroY = 0.0F, remindGyroZ = 0.0F;
float calGyroX = 0.0F, calGyroY = 0.0F, calGyroZ = 0.0F;
int freqServo = 60;
static float temp = 0;
float amplify = 5;

// WIFI
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWD;
const char *mqtt_server = MQTT_SERVER;

// MQTT client
WiFiClient m5Client;
PubSubClient mqtt(m5Client);

// MQTT Payload as JSON
JSONVar packageGyro;
JSONVar packagePotential;

// MQTT Function
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);

void sendGyroMQTT(float, float, float);
void sendPotentialMQTT();
void sendIsGrasp();

bool isGrasp = false;

// Read data
void readGyro();
void displayLCD();
void calibrateGyro();
void readPotential();

// ADS
// Adafruit_ADS1115 ads;     อย่าลืมเอา Comment ออก !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
int finger0 = 0, finger1 = 0, finger2 = 0, finger3 = 0;
int finger0min = 15, finger1min = 15, finger2min = 15, finger3min = 15;
int finger0max = 11000, finger1max = 13000, finger2max = 9000, finger3max = 9220;
int16_t adc0, adc1, adc2, adc3;

// Collision
void isLockServo();

String isCollision;

void setup()
{
  M5.begin();
  M5.Imu.Init();
  // while (!ads.begin()) อย่าลืมเอา Comment ออก !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  // {
  //   Serial.println("Failed to initialize ADS.");
  //   ads.begin();
  // }
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
  readGyro();
  remindGyroX = gyroX;
  remindGyroY = gyroY;
  remindGyroZ = gyroZ;
}

void loop()
{
  readGyro();
  displayLCD();
  if (!mqtt.connected())
  {
    reconnect();
    mqtt.subscribe(TOPIC_COLLISION);
  }
  mqtt.loop();
  calibrateGyro();
  readPotential();
  sendGyroMQTT(gyroX, gyroY, gyroZ);
  sendPotentialMQTT();
  isLockServo();
  sendIsGrasp();
  delay(100);
}
void calibrateGyro()
{
  remindGyroX = gyroX;
  remindGyroY = gyroY;
  remindGyroZ = gyroZ;
}

void readGyro()
{
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getTempData(&temp);
  delay(100);
}

void displayLCD()
{
  M5.Lcd.setRotation(3); // Rotate the screen. 将屏幕旋转
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(80, 15); // set the cursor location.  设置光标位置
  M5.Lcd.println("ESL KMITL");
  M5.Lcd.setCursor(40, 40);
  M5.Lcd.println("  X       Y       Z");
  M5.Lcd.setCursor(32, 60);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
  M5.Lcd.setCursor(170, 60);
  M5.Lcd.print("o/s");
  M5.Lcd.setCursor(30, 95);
  M5.Lcd.printf("Temperature : %.2f C", temp);
}

void setup_wifi()
{
  delay(10);
  // connect to WiFi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

String message;
void callback(char *topic, byte *payload, unsigned int length)
{
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }

  isCollision = message;
  message = "";
}

void reconnect()
{
  // Loop until it is connected
  while (!mqtt.connected())
  {

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendGyroMQTT(float gyroX, float gyroY, float gyroZ)
{
  gyroX *= amplify;
  gyroY *= amplify;
  gyroZ *= amplify;

  packageGyro["X"] = String(gyroX);
  packageGyro["Y"] = String(gyroY);
  packageGyro["Z"] = String(gyroZ);

  String jsonString = JSON.stringify(packageGyro);
  mqtt.publish(TOPIC_GYRO, jsonString.c_str());
}

void sendPotentialMQTT()
{
  packagePotential["finger0"] = String(finger3);
  packagePotential["finger1"] = String(finger2);
  packagePotential["finger2"] = String(finger1);
  packagePotential["finger3"] = String(finger0);

  String jsonString = JSON.stringify(packagePotential);
  mqtt.publish(TOPIC_FINGER, jsonString.c_str());
}
void readPotential()
{
  // adc0 = ads.readADC_SingleEnded(0);
  // adc1 = ads.readADC_SingleEnded(1);
  // adc2 = ads.readADC_SingleEnded(2);
  // adc3 = ads.readADC_SingleEnded(3);
  finger0 = 111;
  finger1 = 222;
  finger2 = 333;
  finger3 = 444;
  // finger0 = map(adc0, finger0min, finger0max, 0, 1024);
  // finger1 = map(adc1, finger1min, finger1max, 0, 1024);
  // finger2 = map(adc2, finger2min, finger2max, 0, 1024);
  // finger3 = map(adc3, finger3min, finger3max, 0, 1024);
}

void isLockServo()
{
  if (isCollision == "1")
  {
    // ใส่โค้ดล็อค Servo
    for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
      pwmServo.setPWM(servoNo, 0, pulselen);
    delay(300);
  }
  else if (isCollision == "0")
  {
    // ใส่โค้ดปลดล็อค Servo
    for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
      pwmServo.setPWM(servoNo, 0, pulselen);
    delay(300);
  }
}

void sendIsGrasp()
{
  if (finger0 > 100 && finger1 > 100 && finger2 > 100 && finger3 > 100 && checkStatusServo == 0)
  {
    mqtt.publish(TOPIC_GRASP, "1");
  }
  else if (finger0 < 100 && finger1 < 100 && finger2 < 100 && finger3 < 100 && checkStatusServo == 1)
  {
    mqtt.publish(TOPIC_GRASP, "0");
    checkStatusServo = 0;
    isCollision = "0";
  }
}