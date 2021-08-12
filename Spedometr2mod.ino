#include <SSD1306.h>  // Библиотека для работы с дисплеем
#include "UbloxGPS.h" // Библиотека для работы с GPS
#include "font.h"     // Шрифт Orbitron_Light_26, не моноширинный
#include "font50.h"   // Шрифт 
#include "font7seg.h" // Шрифт 50
#include "Arimo.h" //И еще Шрифт
#include "FS.h"
#include <ESP8266WiFi.h>
#include <Adafruit_ADXL345_U.h>
#include "EEPROM.h"
int rangeaddr = 0;
#define EEPROM_SIZE 64

long prev = 0;
long buttonTimer = 0;
long longPressTime = 2000;

boolean buttonActive = false;
boolean longPressActive = false;


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int gpsHour;
int timezone = 3;
int iz;
int initaccel;

// Инициализируем дисплей 128х64 подключенный к пинам D2 и D1
SSD1306  display(0x3c, 5, 4);
const char* filename = "/log.txt";
bool deBug = false;          // Отладка на порту
int ScreenUpdateTime = 10; // Время обновления экрана, мс (чем чаще - тем больше погрешность измерений)

int  loops = 0;         // для отладки
char gpsSpeed[3];       // Буфер для строки с скоростью
int  gpsSpeedKm = 0;    // Скорость в км/ч
int  gpsSpeedKmMAX = 0; // Максимальная скорость
bool start = false;     // Старт замера
long startMillis = 0;   // Начало отсчета
long startMillis200 = 0;   // Начало отсчета
long currentMillis = 0; // Текущее время
float meteringTime = 0; // Время замера
float meteringTime200 = 0; // Время замера 200
long NumSatellites = 0; // Количество спутников
char gpsTime[9];        // Время
unsigned long lastScreenUpdate = 0; // Последнее обновление экрана
int ScreenUpdateTimeOSV = 10; // Время обновления экрана когда GPS в поиске
char symbols[5] = "|/-\\";
int symbolnow = 0;
bool save = false;
float speed1;
float speed2;
float speed3;
int rollspeed = 200;
int startrange;
bool startacc = false;
int screen = 0;
String logbuff[40]; String str;
// Cтруктура результатов
struct Metering
{
  float accel30;
  float accel60;
  float accel100;
  float accel200;
  bool met30;
  bool met60;
  bool met100;
  bool met200;
  bool met200start;
};
Metering metering;
Metering meterings[10];

void setup() {
  delay(1500);                    // Без этой задержки плата GPS подвешивает Wemos
  serial.begin(115200);            // Скорость обмена с GPS, при 115200 мой чип работает не стабильно
  Serial.begin(115200);           // Вывод в порт для дебага, 115200 в оригинале

  EEPROM.begin(512);
  if ( EEPROM.read(rangeaddr) != 0 &&  EEPROM.read(rangeaddr) != 1 )
  {
    EEPROM.write(rangeaddr, 0);
    EEPROM.commit();
  }

  startrange = EEPROM.read(rangeaddr);
  if (startrange == 0)
  {
    speed1 = 100;
    speed2 = 150;
    speed3 = 200;
    rollspeed = 200;
  }
  if (startrange == 1)
  {
    speed1 = 30;
    speed2 = 60;
    speed3 = 100;
    rollspeed = 150;
  }
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  pinMode(0, INPUT);
  digitalWrite(0, HIGH);
  pinMode(2, OUTPUT);

  display.init();
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");

  }
  sensor_t sensor;
  accel.getSensor(&sensor);
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  sensors_event_t event;
  accel.getEvent(&event);
  initaccel = event.acceleration.z;
  // Serial.println(initaccel);

  if (SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }
  FSInfo fs_info;
  SPIFFS.info(fs_info);
  printf("SPIFFS: %lu of %lu bytes used.\n",
         fs_info.usedBytes, fs_info.totalBytes);
  File f = SPIFFS.open(filename, "a+");
  Serial.println(f.size());
  f.close();  //Close file
  if (fs_info.usedBytes > 2024)
  {
    display.clear();
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 1, "FORMATTING SD");
    Serial.println("Spiffs formatting");
    display.display();
    SPIFFS.format();
    display.clear();
    display.drawString(64, 1, "FORMATTING DONE");
    display.display();
    delay(1000);

  }

  gpsSetup();                     // Настройка модуля Ublox
  metering = {speed1, speed2, speed3, rollspeed, true, true, true, true }; // Тут будем хранить результаты

  // Вывод приветствия

  display.clear();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  if (startrange == 0)
    display.drawString(64, 5, "100-150-200");
  if (startrange == 1)
    display.drawString(64, 5, "30-60-100");

  display.drawString(64, 15, "SpeedGauge");
  display.drawString(64, 26, "Roll 100-" + String(rollspeed));
  display.drawString(64, 40, "Used " + String(fs_info.usedBytes / 1024) + " of " + String(fs_info.totalBytes / 1024) + " Kb");
  display.drawString(64, 50, "GPS 10Hz + GYRO");
  display.display();
  delay(1500);

}
void loop() {
  if (!start) {
    if (digitalRead(0) == LOW) {
      if (buttonActive == false) {
        buttonActive = true;
        buttonTimer = millis();
      }
      if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
        longPressActive = true;
        // TODO button is pressed long
        if ( EEPROM.read(rangeaddr) == 0)  EEPROM.write(rangeaddr, 1);
        else if (EEPROM.read(rangeaddr) == 1)  EEPROM.write(rangeaddr, 0);
        EEPROM.commit();
        // Serial.println(EEPROM.read(rangeaddr));
        delay(200);
        updateRestart();
      }

    } else {
      if (buttonActive == true) {
        if (longPressActive == true) {
          longPressActive = false;
        } else {
          screen++;
          if (screen > 3) screen = 0;
          if (screen == 1  ) deBug = true; else deBug = false;
          delay(200);
        }
        buttonActive = false;
      }
    }
     Serial.println("LongPress " + String(longPressActive) + " screen " + String(screen));
  }


  // 

  currentMillis = millis(); // текущее время в миллисекундах
  int msgType = processGPS();
  if (msgType == MT_NAV_PVT) {

    NumSatellites = ubxMessage.navPvt.numSV;
    gpsSpeedKm = ubxMessage.navPvt.gSpeed * 0.0036; // Переводим в км/ч
  }
  sensors_event_t event;
  accel.getEvent(&event);
  iz = event.acceleration.z - initaccel;
  //  Serial.println(iz);
  if (iz < -1) startacc = true;

  if (deBug && startacc) {
    {
      loops++;
      delay(12);
    }
    if ( loops > 210 ) {
      loops = 0;

    }
    gpsSpeedKm = loops;
  }

  if (gpsSpeedKm > gpsSpeedKmMAX) {
    gpsSpeedKmMAX = gpsSpeedKm;
  }
  // Если движемся
  if (gpsSpeedKm > 0 && startacc ) {
    digitalWrite(2, HIGH);

    // Если это был старт
    if (!start) {
      start = true;
      startMillis = millis();
      metering.met30 = false;
      metering.met60 = false;
      metering.met100 = false;
      metering.met200 = false;
    }
    if (!metering.met30 && !metering.met60 && !metering.met100 && !metering.met200 && gpsSpeedKm > 20) {
      clearResult();  // обнуление результатов на экране при достижении скорости
    }

    meteringTime = (float)(currentMillis - startMillis) / 1000; // Время замера
    meteringTime200 = (float)(currentMillis - startMillis) / 1000; // Время замера
    // Заношу результаты замера
    if (!metering.met200start && gpsSpeedKm >= 100) {
      startMillis200 = millis();
      metering.met200start = true;
    }
    if (!metering.met30 && gpsSpeedKm >= speed1) {
      metering.accel30 = meteringTime; // Разгон до 30км/ч
      metering.met30 = true;

    }
    if (!metering.met60 && gpsSpeedKm >= speed2) {
      metering.accel60 = meteringTime - metering.accel30; // Разгон до 60км/ч
      metering.met60 = true;
    }
    if (!metering.met100 && gpsSpeedKm >= speed3) {
      metering.accel100 = meteringTime - metering.accel30; // Разгон до 100км/ч
      metering.met100 = true;

      save = true;
    }
    //
    if (!metering.met200 && gpsSpeedKm >= rollspeed) {
      metering.accel200 = (float)(currentMillis - startMillis200) / 1000;
      metering.met200 = true;
    }
    if (gpsSpeedKm < 100) {
      metering.met200 = false;
      metering.met200start = false;
    }

  } else if (start && 0 == gpsSpeedKm && abs(iz) < 3) { // Если остановились
    start = false;
    startacc = false;
    digitalWrite(2, LOW);
    save = false;
  }

  //   if (deBug) {
  //    Serial.print("#SV: ");      Serial.print(ubxMessage.navPvt.numSV);
  //    Serial.print(" fixType: "); Serial.print(ubxMessage.navPvt.fixType);
  //    Serial.print(" Date:");     Serial.print(ubxMessage.navPvt.year); Serial.print("/"); Serial.print(ubxMessage.navPvt.month); Serial.print("/"); Serial.print(ubxMessage.navPvt.day); Serial.print(" "); Serial.print(ubxMessage.navPvt.hour); Serial.print(":"); Serial.print(ubxMessage.navPvt.minute); Serial.print(":"); Serial.print(ubxMessage.navPvt.second);
  //    Serial.print(" lat/lon: "); Serial.print(ubxMessage.navPvt.lat / 10000000.0f); Serial.print(","); Serial.print(ubxMessage.navPvt.lon / 10000000.0f);
  //    Serial.print(" gSpeed: ");  Serial.print(ubxMessage.navPvt.gSpeed / 1000.0f);
  //    Serial.print(" heading: "); Serial.print(ubxMessage.navPvt.heading / 100000.0f);
  //    Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPvt.hAcc / 1000.0f);
  //    Serial.println();
  //   } // debug
  // if (msgType == MT_NAV_PVT)

  if (metering.met30 && metering.met60 && metering.met100 && save)
  {

    Metering *meteringsNew = new Metering[10];
    memcpy(meteringsNew, meterings, sizeof(meterings[0]) * 10);
    meterings[0] = metering;
    for (int i = 0; i < 9; i++)
    {
      meterings[i + 1] = meteringsNew[i];
    }
    Serial.println(String((String(meterings[0].accel30)) + " " + String(meterings[0].accel60) + " " + String(meterings[0].accel100) + " " + String(meterings[0].accel200) + " | " + String(gpsHour) + ":" + String(ubxMessage.navPvt.minute)));
    delete[] meteringsNew;
    //-------------
    File f = SPIFFS.open(filename, "a");
    logbuff[1] = String(String(metering.accel30) + " " + String(metering.accel60) + " " + String(metering.accel100) + " | " + String(gpsHour) + ":" + String(ubxMessage.navPvt.minute) + "$");
    // Serial.println(logbuff[1]);
    f.print(logbuff[1]);
    // Serial.println(f.size());
    f.close();  //Close file

    f = SPIFFS.open(filename, "a+");
    if (!f) {
      Serial.println("file open failed");

    }
    else
    {
      Serial.println("Reading Data from File:");
      while (f.available()) {
        String line = f.readStringUntil('$');
        Serial.println(line);
      }
    }
    f.close();  //Close file
    save = false;
  }
  int ScreenUpdateTimeNow = ScreenUpdateTime;
  if ( NumSatellites < 2 ) {
    ScreenUpdateTimeNow = ScreenUpdateTimeOSV;
  }
  if (currentMillis - lastScreenUpdate > ScreenUpdateTimeNow) {
    if (screen == 0) updateDisplay();
    if (screen == 1) updateDisplay();
    if (screen == 2) updateDisplaySpeed();
    if (screen == 3) updateDisplayRoll();
    lastScreenUpdate = currentMillis;
  }

}
void clearResult() {
  metering.accel30 = 0.0;
  metering.accel60 = 0.0;
  metering.accel100 = 0.0;
  metering.accel200 = 0.0;
  gpsSpeedKmMAX = 0;
}
void updateDisplaySpeed()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(Arimo_Bold_52);
  sprintf(gpsSpeed, "%03d", gpsSpeedKm);
  display.drawString(64, 0, gpsSpeed);
  display.display();
}

void updateDisplayRoll()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(Arimo_Bold_36);
  sprintf(gpsSpeed, "%03d", gpsSpeedKm);
  display.drawString(64, 0, gpsSpeed);
  display.setFont(Arimo_Bold_16);
  display.drawString(64, 45, String(metering.accel200));
  display.display();
}
void updateDisplay() {
  display.clear();
  // Рисуем индикатор приёма
  display.drawVerticalLine(1, 12, 2);
  display.drawVerticalLine(0, 12, 2);
  // Херовый приём
  if (NumSatellites > 3 ) {
    display.drawVerticalLine(4, 10, 4);
    display.drawVerticalLine(3, 10, 4);
  }
  // Нормальный
  if (NumSatellites > 5 ) {
    display.drawVerticalLine(7, 7, 7);
    display.drawVerticalLine(6, 7, 7);
  }
  // Отличный приём спутников
  if (NumSatellites > 8 ) {
    display.drawVerticalLine(10, 4, 10);
    display.drawVerticalLine(9, 4, 10);
  }
  // Разделительная линия
  display.drawHorizontalLine(0, 14, 128);

  // Выводим количество спутников
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(14, 2, (String)NumSatellites + " fix: " + ubxMessage.navPvt.fixType ); // вывожу количество пойманных спутников
  // Вывод времени
  gpsHour = ubxMessage.navPvt.hour + timezone;
  if (gpsHour >= 24) gpsHour -= 24;
  sprintf(gpsTime, "%02d:%02d:%02d", gpsHour, ubxMessage.navPvt.minute, ubxMessage.navPvt.second);


  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  if (deBug) display.drawString(128, 2, (String)"DEBUG " + gpsTime);

  else display.drawString(128, 2, gpsTime);

  // Погрешность измерений
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  if ( NumSatellites < 2 ) {
    symbolnow++;
    if (symbolnow > 3 ) {
      symbolnow = 0;
    }
    if (deBug)
    {
      display.drawString(1, 54, "G:" + String(iz) + " GS: " + String(startacc) + " M: " + String(metering.accel200));
    }
    else
      display.drawString(1, 54, "SEARCH GPS");

  }
  else {
    float sAcc = ubxMessage.navPvt.sAcc / 1000.0f;
    if (sAcc > 9) {
      sAcc = 9;
    };
    display.drawString(1, 54, String(sAcc)  + " G:" + String(iz) + " T:" + String(metering.accel200));
  }
  // Вывод скорости
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(Arimo_Bold_36);
  sprintf(gpsSpeed, "%03d", gpsSpeedKm);
  display.drawString(40, 10, gpsSpeed);
  // Максимальной скорости за заезд
  if ( gpsSpeedKmMAX > 30 ) {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(40, 44, "max " + (String)gpsSpeedKmMAX + " km/h");
  }
  // Разгон до 30 км/ч
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(Arimo_Bold_16);
  display.drawString(128, 16, (String)metering.accel30);
  // Разгон до 60 км/ч
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(Arimo_Bold_16);
  display.drawString(128, 32, (String)metering.accel60);
  // Разгон до 100 км/ч
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(Arimo_Bold_16);
  display.drawString(128, 48, (String)metering.accel100);
  display.display();
}

void updateRestart()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 20, (String)"Range changed");
  display.drawString(64, 30, (String)"Restarting");
  display.display();
  Serial.println("RESTARTING");
  delay(500);
  ESP.restart();

}
