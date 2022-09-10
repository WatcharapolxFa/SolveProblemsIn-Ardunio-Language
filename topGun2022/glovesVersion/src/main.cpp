// lib
#include <Arduino.h>
#include <M5StickCPlus.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_I2CDevice.h>
#include <M5StickCPlus.h>
#include <PubSubClient.h>
#include <Arduino_JSON.h>
#include "config.h"
#include <WiFi.h>
#include "Madgwick_Quaternion.h"

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

// ADS
Adafruit_ADS1115 ads;
int finger0 = 0, finger1 = 0, finger2 = 0, finger3 = 0;
int finger0min = 15, finger1min = 15, finger2min = 15, finger3min = 15;
int finger0max = 11000, finger1max = 13000, finger2max = 9000, finger3max = 9220;
int16_t adc0, adc1, adc2, adc3;

// Collision
void isLockServo();

String isCollision;

// millis
long long lastSend;
long long lastSendFinger;
long long sendDelay = 200; // 100 ms
long long lastMqttReconnect;
long long mqttReconnectDelay = 5000; // 5 s
long long lastReadPotentiometer;
long long readPotentiometerDelay = 100; // 100 ms
long long lastReadIMU;
long long readIMUDelay = 10; // 10 ms
long long lastCheckGlove;
long long checkDelay = 700;

// Gyro Values
float pitch = 0.0F, roll = 0.0F, yaw = 0.0F;
float quat_w = 0.0F;
float quat_x = 0.0F;
float quat_y = 0.0F;
float quat_z = 0.0F;
float acc[3];
float accOffset[3];
float gyro[3];
float gyroOffset[3];
float mag[3];
float magOffset[3];
float magmax[3];
float magmin[3];
uint8_t setup_flag = 1;
uint8_t action_flag = 1;
float heading = 0;
uint8_t smoothen = 100;
float strength = 1;
// Servo Utils
Adafruit_PWMServoDriver pwmServo = Adafruit_PWMServoDriver();
#define SERVOMIN 100  // Minimum pulse length count out of 4096.
#define SERVOMAX 2400 // Maximum pulse length count out of 4096.    // Defines a counter for servos. count0-n
int checkStatusServo = 0;

void servoGo();
void servoBack();
// Gyro Functions
Madgwick_Quaternion M_Q_Filter;
void calibrate6886();
void calibrate_waiting(uint32_t);

// MQTT Functions
void sendMQTTGyro();
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void updateMQTT();

void sendGyroMQTT(float, float, float);
void sendPotentialMQTT();
void sendIsGrasp();
void updateIsGrasp();
void updateSendMQTTfinger();
void updateReadPotentiometer();

void readGyro();
void displayLCD();
void calibrateGyro();
void readPotential();

// MQTT Function
void update();
void IMU_Update();
void updateSendMQTTGyro();
void updateReadIMU();
void updateReCalibrate();

void setup()
{
  M5.begin();
  M5.Imu.Init();
  while (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    ads.begin();
  }
  setup_wifi();
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  // calibrate imu
  calibrate6886();

  // begin imu
  M_Q_Filter.begin(smoothen); // 100Hz

  // calibrate imu again
  calibrate_waiting(10);
}

void loop()
{
  update();
}

void update()
{
  M5.update();
  updateMQTT();
  updateReadIMU();
  updateReCalibrate();
  updateReadPotentiometer();
  updateSendMQTTfinger();
  updateSendMQTTGyro();
  updateIsGrasp();
  isLockServo();
}

void updateMQTT()
{
  if (!mqtt.connected() && WiFi.isConnected())
  {
    reconnect();
    mqtt.subscribe(TOPIC_COLLISION);
  }
  mqtt.loop();
}

void updateReadPotentiometer()
{
  if (((millis() - lastReadPotentiometer) >= readPotentiometerDelay))
  {
    readPotential();
    lastReadPotentiometer = millis();
  }
}

void readPotential()
{
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  finger0 = map(adc0, finger0min, finger0max, 0, 1024);
  finger1 = map(adc1, finger1min, finger1max, 0, 1024);
  finger2 = map(adc2, finger2min, finger2max, 0, 1024);
  finger3 = map(adc3, finger3min, finger3max, 0, 1024);
}

void sendMQTTfinger()
{
  packagePotential["finger0"] = String(finger3);
  packagePotential["finger1"] = String(finger2);
  packagePotential["finger2"] = String(finger1);
  packagePotential["finger3"] = String(finger0);

  String jsonString = JSON.stringify(packagePotential);
  mqtt.publish(TOPIC_FINGER, jsonString.c_str());
}

void updateSendMQTTfinger()
{
  if (((millis() - lastSendFinger) >= sendDelay))
  {
    sendMQTTfinger();
    lastSendFinger = millis();
  }
}

void updateSendMQTTGyro()
{
  if (((millis() - lastSend) >= sendDelay))
  {
    sendMQTTGyro();
    lastSend = millis();
  }
}

void sendMQTTGyro()
{
  // make json string
  JSONVar packageJSON;
  packageJSON["X"] = String(quat_x);
  packageJSON["Y"] = String(quat_y);
  packageJSON["Z"] = String(quat_z);
  String packageString = JSON.stringify(packageJSON);
  mqtt.publish(TOPIC_GYRO, packageString.c_str());
}

int lastM = 0;
void updateIsGrasp()
{ //&& finger3 > 100 | && finger3 < 100
  if (finger0 > 100 && finger1 > 100 && finger2 > 100)
  {
    if (lastM == 0)
    {
      mqtt.publish(TOPIC_GRASP, "1");
      lastM = 1;
    }
  }
  else if (finger0 < 100 && finger1 < 100 && finger2 < 100)
  {
    if (lastM == 1)
    {
      mqtt.publish(TOPIC_GRASP, "0");
      lastM = 0;
    }
    isCollision = "0";
  }
}

void isLockServo()
{
  if (isCollision == "1" && checkStatusServo == 0)
  {
    // ใส่โค้ดล็อค Servo
    servoGo();
    checkStatusServo = 1;
  }
  else if (isCollision == "0" && checkStatusServo == 1)
  {
    servoBack();
    checkStatusServo = 0;
  }
}
int servoNo0 = 3; // Defines a counter for servos. count0-n
int servoNo1 = 4; // Defines a counter for servos. count0-n
int servoNo2 = 5; // Defines a counter for servos. count0-n
int servoNo3 = 6; // Defines a counter for servos. count0-n

int servoNoMax = 6; // maximum n servo
void servoGo()
{

  for (int pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
  {
    pwmServo.setPWM(servoNo0, 0, pulselen);
    pwmServo.setPWM(servoNo1, 0, pulselen);
    pwmServo.setPWM(servoNo2, 0, pulselen);
    pwmServo.setPWM(servoNo3, 0, pulselen);
  }
  delay(300);
  if (servoNoMax < servoNo0)
  {
    servoNo0 = 3;
  }
  Serial.println("serVoGo");
}

void servoBack()
{

  for (int pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
  {
    pwmServo.setPWM(servoNo0, 0, pulselen);
    pwmServo.setPWM(servoNo1, 0, pulselen);
    pwmServo.setPWM(servoNo2, 0, pulselen);
    pwmServo.setPWM(servoNo3, 0, pulselen); // Drives each servo one at a time first
  }
  delay(300);
  if (servoNoMax < servoNo0)
  {
    servoNo0 = 3;
  }
  Serial.println("serBack");
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
  Serial.println(isCollision);
  message = "";
}

void reconnect()
{
  if (((millis() - lastMqttReconnect) >= mqttReconnectDelay))
  {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("connected");
      mqtt.publish("topgun/team1/connect", "1");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
    }
    lastMqttReconnect = millis();
  }
}

void calibrate6886()
{
  double gyroSum[3];
  double accSum[3];
  int counter = 5;

  Serial.println("Calibrating...");

  M5.Imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.Imu.getAccelData(&acc[0], &acc[1], &acc[2]);

  gyroOffset[0] = gyro[0];
  gyroOffset[1] = gyro[1];
  gyroOffset[2] = gyro[2];
  accOffset[0] = acc[0];
  accOffset[1] = acc[1];
  accOffset[2] = acc[2] - 1.0; // Gravitational Acceleration 1G, assuming that the M5 button is facing upward

  Serial.println("Calibration Done!");
  Serial.print("gyroOffset[0]: ");
  Serial.print(gyroOffset[0]);
  Serial.print(" | gyroOffset[1]: ");
  Serial.print(gyroOffset[1]);
  Serial.print(" | gyroOffset[2]: ");
  Serial.println(gyroOffset[2]);
  Serial.print("accOffset[0]: ");
  Serial.print(accOffset[0]);
  Serial.print(" | accOffset[1]: ");
  Serial.print(accOffset[1]);
  Serial.print(" | accOffset[2]: ");
  Serial.println(accOffset[2]);
}

void calibrate_waiting(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    if (digitalRead(M5_BUTTON_HOME) == LOW)
    {
      setup_flag = 1;
      while (digitalRead(M5_BUTTON_HOME) == LOW)
        ;
      break;
    }
    delay(100);
  }
}

void applycalibration()
{
  M5.Imu.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
  M5.Imu.getAccelData(&acc[0], &acc[1], &acc[2]);

  gyro[0] -= gyroOffset[0];
  gyro[1] -= gyroOffset[1];
  gyro[2] -= gyroOffset[2];
  acc[0] -= accOffset[0];
  acc[1] -= accOffset[1];
  acc[2] -= accOffset[2];

  // fake magnetometer data cuz MPU6886 doesn't come with BMM 150 chip
  mag[0] = 0;
  mag[1] = 0;
  mag[2] = 0;
}

void IMU_Update()
{
  applycalibration();

  heading = atan2(mag[0], mag[1]);
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;

  M_Q_Filter.update(gyro[0] * strength, gyro[1] * strength, gyro[2] * strength, acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);

  roll = M_Q_Filter.getRoll();
  pitch = M_Q_Filter.getPitch();
  yaw = M_Q_Filter.getYaw();

  // for quaternion output (by zeketan)
  quat_w = M_Q_Filter.getQuat_W();
  quat_x = M_Q_Filter.getQuat_X();
  quat_y = M_Q_Filter.getQuat_Y();
  quat_z = M_Q_Filter.getQuat_Z(); //-0.005 anti yaw drifting
}

void updateReadIMU()
{
  if ((millis() - lastReadIMU) >= readIMUDelay)
  {
    lastReadIMU = millis();
    IMU_Update();
  }
}

void updateReCalibrate()
{
  // Button A
  if (M5.BtnA.wasReleased())
  {
    // reset esp32
    Serial.println("Reset ESP32");
    ESP.restart();
  }
}