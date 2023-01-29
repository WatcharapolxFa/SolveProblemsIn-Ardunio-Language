#include <Arduino.h>

#include <Adafruit_LiquidCrystal.h>
#include "Wire.h"

Adafruit_LiquidCrystal lcd_1(0);

/********|
 * LED   |
 ********/
int LedOne = 8;
int LedTwo = 9;
int LedThree = 10;

/**********|
 * Button  |
 **********/
int BtRedOne = 11;
int BtRedTwo = 12;
int BtRedThree = 13;
int BtStart = 3;
int bt1 = 0, bt2 = 0, bt3 = 0;

int isActive = 0;
int reading;             // ค่าที่อ่านได้จากปุ่มกด (รวม bounce)
int counter = 0;         // จำนวน iteration ที่เราเห็นการกด หรือปล่อย
int current_state = LOW; // ค่าที่ได้หลังการทำ debounce (คือการกดหรือปล่อยจริงๆ)
long time = 0;           // เวลาล่าสุดที่มีการเก็บค่า
int debounce_count = 10; // จำนวนมิลลิวินาที/รอบการวนลูป ที่ต่ำสุดที่เชื่อได้ว่ามีการกด หรือปล่อยจริงๆ
int action_count = 0;    // ไว้แสดงจำนวนการกด

int ranLed = 0; // สุ่มเกิดแสง LED
int arr[3][2] = {
    {11, 0},
    {12, 0},
    {13, 0},
};

long countTimeOne = 0;
long countTimeTwo = 0;
long countTimeThree = 0;
long countTimeFour = 0;
long sumTime = 0;


int step = 0;

void setup()
{
  /**********|
   * Button  |
   **********/
  pinMode(BtRedOne, INPUT_PULLUP); // ตั้งค่าปุ่มกดเป็นแบบ active low (กด = logic 0 ปล่อย = logic 1)

  /************|
   * ตั้งค่า LED  |
   ************/
  pinMode(LedOne, OUTPUT);
  digitalWrite(LedOne, LOW);

  /***************|
   * ตั้งค่า Serial  |
   ***************/
  Serial.begin(9600);
  lcd_1.begin(16, 2);
  lcd_1.setCursor(2 , 0);
  lcd_1.print("Please Start");
}

void loop()
{
  /************|
   * Debounce  |
   ************/

  // เช็คทุกมิลลิวินาที
  if (millis() != time)
  {
    // อ่านค่าปุ่มกด
    if (isActive == 0)
    {
      reading = digitalRead(BtStart);
      // Serial.println(reading);
      if (reading == 1)
        isActive = 1;
      // Serial.println(isActive);
    }
    bt1 = digitalRead(BtRedOne);
    bt2 = digitalRead(BtRedTwo);
    bt3 = digitalRead(BtRedThree);

    // ถ้าค่าปุ่มกดเท่ากับค่าเดิม ให้ลดการนับคะแนน (อาจเกิดการ bounce หรือไม่มีactionอะไรเกิดขึ้น)
    if (reading == current_state && counter > 0)
    {
      counter--;
    }
    // ถ้าค่าที่อ่านได้ไม่เท่ากับค่าเดิม ให้เพิ่มคะแนนไปเรื่อยๆ
    if (reading != current_state)
    {
      counter++;
    }
    // ถ้าคะแนนมากกว่าค่าที่ตั้งไว้ ก็ให้รีเซ็ตคะแนนและเปลี่ยนสถานะว่ามีการกดหรือปล่อยจริงๆ พร้อมแสดงออกไปยัง LED
    if (counter >= debounce_count)
    {
      // ถ้าเป็นการกดจะแสดงผลจำนวนที่กดไปทาง serial monitor
      if (reading == LOW)
        // Serial.println(action_count++);
        counter = 0;

      current_state = reading;
      if (isActive)
      {
        // isActive = 1;
        ranLed = random(8, 11);
        digitalWrite(ranLed, HIGH);
        if(step == 0){
          countTimeOne = millis();
        }
        else if(step == 1){
          countTimeTwo = millis();
        }
        else if(step == 2){
          countTimeThree = millis();
        }
        else if(step == 3){
          countTimeFour = millis();
        }
        else if(step == 4){
          countTimeThree = 0 ;
          countTimeTwo = 0 ;
          countTimeOne = 0 ;
          countTimeFour = 0 ;
          step = 0;
        }

        // Serial.println(ranLed);
        arr[ranLed - 8][1] = 1;
      }
    }

    if (arr[0][1] == 1 && bt1 == 1 && isActive == 1)
    {
      digitalWrite(LedOne, LOW);
      arr[0][1] = 0;
      isActive = 0;
     if(step == 0 ){
        countTimeOne = millis() - countTimeOne;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeOne: ");
        lcd_1.print(countTimeOne);
        lcd_1.print(" mS ");
        Serial.print("countTimeOne : ");
        Serial.println(countTimeOne);
        step = 1 ; 
      }
      else if(step == 1){
        countTimeTwo = millis() - countTimeTwo;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeTwo: ");
        lcd_1.print(countTimeTwo);
        lcd_1.print(" mS ");
        Serial.print("countTimeTwo : ");
        Serial.println(countTimeTwo);
        step = 2;
      }
      else if(step == 2 ){
        countTimeThree = millis() - countTimeThree;
        Serial.print("countTimeThree : ");
        Serial.println(countTimeThree);
        lcd_1.print(" mS ");
        step = 3 ;
      }
      else if(step == 3 ){
        countTimeFour = millis() - countTimeFour;
        Serial.print("countTimeFour : ");
        Serial.println(countTimeFour);
        sumTime = (countTimeFour + countTimeTwo + countTimeThree)/3000 ; 
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("sumTime :  ");
        lcd_1.print(sumTime);
        lcd_1.print(" S ");
        Serial.print("sumTime : ");
        Serial.print(sumTime);
        Serial.println(" S ");
        countTimeThree = 0 ;
        countTimeTwo = 0 ;
        countTimeOne = 0 ;
        countTimeFour = 0 ;
        step = 4 ;
      }
    }
    else if (arr[1][1] == 1 && bt2 == 1 && isActive == 1)
    {
      digitalWrite(LedTwo, LOW);
      arr[1][1] = 0;
      isActive = 0;
           if(step == 0 ){
        countTimeOne = millis() - countTimeOne;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeOne: ");
        lcd_1.print(countTimeOne);
        lcd_1.print(" mS ");
        Serial.print("countTimeOne : ");
        Serial.println(countTimeOne);
        step = 1 ; 
      }
      else if(step == 1){
        countTimeTwo = millis() - countTimeTwo;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeTwo: ");
        lcd_1.print(countTimeTwo);
        lcd_1.print(" mS ");
        Serial.print("countTimeTwo : ");
        Serial.println(countTimeTwo);
        step = 2;
      }
      else if(step == 2 ){
        countTimeThree = millis() - countTimeThree;
        Serial.print("countTimeThree : ");
        Serial.println(countTimeThree);
        lcd_1.print(" mS ");
        step = 3 ;
      }
      else if(step == 3 ){
        countTimeFour = millis() - countTimeFour;
        Serial.print("countTimeFour : ");
        Serial.println(countTimeFour);
        sumTime = (countTimeFour + countTimeTwo + countTimeThree)/3000 ; 
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("sumTime :  ");
        lcd_1.print(sumTime);
        lcd_1.print(" S ");
        Serial.print("sumTime : ");
        Serial.print(sumTime);
        Serial.println(" S ");
        countTimeThree = 0 ;
        countTimeTwo = 0 ;
        countTimeOne = 0 ;
        countTimeFour = 0 ;
        step = 4 ;
      }
    }
    else if (arr[2][1] == 1 && bt3 == 1 && isActive == 1)
    {
      digitalWrite(LedThree, LOW);
      arr[2][1] = 0;
      isActive = 0;
            if(step == 0 ){
        countTimeOne = millis() - countTimeOne;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeOne: ");
        lcd_1.print(countTimeOne);
        lcd_1.print(" mS ");
        Serial.print("countTimeOne : ");
        Serial.println(countTimeOne);
        step = 1 ; 
      }
      else if(step == 1){
        countTimeTwo = millis() - countTimeTwo;
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("TimeTwo: ");
        lcd_1.print(countTimeTwo);
        lcd_1.print(" mS ");
        Serial.print("countTimeTwo : ");
        Serial.println(countTimeTwo);
        step = 2;
      }
      else if(step == 2 ){
        countTimeThree = millis() - countTimeThree;
        Serial.print("countTimeThree : ");
        Serial.println(countTimeThree);
        lcd_1.print(" mS ");
        step = 3 ;
      }
      else if(step == 3 ){
        countTimeFour = millis() - countTimeFour;
        Serial.print("countTimeFour : ");
        Serial.println(countTimeFour);
        sumTime = (countTimeFour + countTimeTwo + countTimeThree)/3000 ; 
        lcd_1.clear();
        lcd_1.setCursor(0 , 0);
        lcd_1.print("sumTime :  ");
        lcd_1.print(sumTime);
        lcd_1.print(" S ");
        Serial.print("sumTime : ");
        Serial.print(sumTime);
        Serial.println(" S ");
        countTimeThree = 0 ;
        countTimeTwo = 0 ;
        countTimeOne = 0 ;
        countTimeFour = 0 ;
        step = 4 ;
      }
    }
    time = millis();
  }
}