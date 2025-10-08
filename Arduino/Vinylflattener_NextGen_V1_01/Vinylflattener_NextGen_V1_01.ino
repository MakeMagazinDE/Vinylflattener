/*
#diy #Vinylflattener, NextGen 
Version 1.01, 18.07.2025
@vincent05.bsky.social

Cleanup & Debug Ooption added

V1.01: MCP added

*/

//Libraries

#include <Arduino.h>
#include <Wire.h>
#include <FT6X36.h>
#include <TFT_eSPI.h>
#include <TaskScheduler.h>  // TaskScheduler Library

// uncomment for debug output
//#define DEBUG
//#define DEBUGSENSOR
//#define DEBUGMCP

// Heizungsregelung

#include <AutoPID.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define GFXFF 1
#define CF_RT24 &Roboto_Thin_24
#define CF_FSB12 &FreeSansBold12pt7b


//Temperaturregelung
#define TEMP_READ_DELAY 1000  //can only read digital temp sensor every ~750ms

//pid settings fuer Relaisbetieb
#define PULSEWIDTH 200   
//#define KP .12
//#define KI .0003
//#define KD 3
#define KP 0.09
#define KI 0.0002
#define KD 1.8


//GPIO Pins, die Pins fuer das TFT Display sind in der Datei User_Setup.h definiert

#define TEMP_SENSOR1_PIN 16
#define TEMP_SENSOR2_PIN 17
#define RELAIS1 5
#define RELAIS2 19
#define T_RST 27  // Touch Controller Reset Pin
#define T_INT 26  // Touch Controller Interrupt Pin

FT6X36 ts(&Wire, T_INT);  //Touch Controller Interruptpin (CTP_INT)
TFT_eSPI tft = TFT_eSPI();
// Button-Positionen und Größen
#define BUTTON_START_X 140
#define BUTTON_START_Y 200
#define BUTTON_START_W 100
#define BUTTON_START_H 50

#define BUTTON_STOP_X 20
#define BUTTON_STOP_Y 200
#define BUTTON_STOP_W 100
#define BUTTON_STOP_H 50

#define NAV_UP_X 300
#define NAV_UP_Y 40
#define NAV_DOWN_X 300
#define NAV_DOWN_Y 200

// Navigation button touch areas
#define NAV_UP_TOUCH_X 280
#define NAV_UP_TOUCH_Y 20
#define NAV_UP_TOUCH_W 40
#define NAV_UP_TOUCH_H 40

#define NAV_DOWN_TOUCH_X 280
#define NAV_DOWN_TOUCH_Y 180
#define NAV_DOWN_TOUCH_W 40
#define NAV_DOWN_TOUCH_H 40

// Slider-Bereiche
#define SLIDER_TEMP_X 20
#define SLIDER_TEMP_Y 80
#define SLIDER_TEMP_W 280
#define SLIDER_TEMP_H 20
#define SLIDER_TIME_X 20
#define SLIDER_TIME_Y 150
#define SLIDER_TIME_W 280
#define SLIDER_TIME_H 20

// Variablen Temperaturregelung
double targetTemperature = 55;    // Solltemperatur Startwert
double currentTemperature1 = 21;  // Isttemperatur 1
double currentTemperature2 = 22;  // Isttemperatur 2

double temperature1;
double temperature2;
double setPoint;
double tempThreshold = 0.8;  //Schwellwert für Abbruch bei Überhitzung (MCP Watchdog)

bool RelayControl1, RelayControl2;
double changeTemp1;
double changeTemp2;
double RampTime = 300;  // Zeitintervall für einen Durchlauf der Funktion TemperatureRamp (kleiner => genauer)
double RampUpdate = 0;  //Variable in der Funktion TemperatureRamp
double currentSetpoint = 10; //Variable in der Funktion TemperatureRamp
double powerupTemperature = 0;  //Variable in der Funktion TemperatureRamp
double powerUpFactor = 9;  // Zieltemperatur - powerUpFactor ergibt den Wert, bis zu dem das System ungeregelt aufheizt
double rampRate = 0.8;  //Variable in der Funktion TemperatureRamp, größer bedeutet schnelleres Aufheizen (default 0.5)
int pidTimeStep = 50;  // die Zeit, wie oft eine Neuberechnung von autoPID durchgeführt wird, sollte <= PULSEWIDTH sein

unsigned long lastTempUpdate1;  //tracks clock time of last temp1 update
unsigned long lastTempUpdate2;  //tracks clock time of last temp2 update
unsigned long Time;

// alle anderen Variablen
int timeSetting = 240;     // Zeiteinstellung (in Minuten), (Startwert)
int lastTargetTemp = -1;   // Zum Überprüfen, ob die Temperatur geändert wurde
int lastTimeSetting = -1;  // Zum Überprüfen, ob die Zeit geändert wurde
int countdownSeconds = 0;  // Anzeige basiert auf dieser Sekunde
int errorcount = 0;        // MCP Zähler
int stopcount = 0;         // MCP Zähler

// Slider-Bereiche definieren
const int tempMin = 48;   //Minimalwert Temperaturslider
const int tempMax = 70;   //Maximalwert Temperaturslider
const int timeMin = 1;    //Minimalwert Zeitslider
const int timeMax = 360;  //Maximalwert Zeitslider

// Letzte Slider-Positionen speichern (um alte Marker zu löschen)
int lastTempSliderPos = -1;
int lastTimeSliderPos = -1;

int old_x = 0;
int old_y = 0;


// Seitenwechsel
int currentPage = 1;
bool screenChanged = true;

// Timer Variablen
unsigned long timerStartMillis = 0;  // Speichert den Startzeitpunkt des Timers
unsigned long remainingMillis = 0;   // Speichert die verbleibende Zeit in Millisekunden
unsigned long lastUpdate = 0;        // Letzter Zeitpunkt, wo der Countdown aktualisiert wurde

// Timer läuft?
bool timerRunning = false;  // Flag, das angibt, ob der Timer läuft
bool timerExpired = false;  // Flag, das anzeigt, dass der Timer abgelaufen ist

//Laufzeitprüfung

unsigned long startTime = 0;
unsigned long endTime = 0;
unsigned long duration = 0;

//Temperatursteuerung initialisieren

OneWire oneWire1(TEMP_SENSOR1_PIN);
OneWire oneWire2(TEMP_SENSOR2_PIN);
DallasTemperature temperatureSensor1(&oneWire1);
DallasTemperature temperatureSensor2(&oneWire2);



//input/output variables passed by reference, so they are updated automatically
AutoPIDRelay myPID1(&temperature1, &setPoint, &RelayControl1, PULSEWIDTH, KP, KI, KD);
AutoPIDRelay myPID2(&temperature2, &setPoint, &RelayControl2, PULSEWIDTH, KP, KI, KD);

Scheduler runner;

//Funktionen zu den Tasks deklarieren
void updateRemainingMillis();   //Zeitbasis
void touchTaskCallback();       // Task für die Touch-Abfrage erstellen
void temperatureUpdateTask1();  //Abfrage Sensor 1
void temperatureUpdateTask2();  //Abfrage Sensor 2
void screenUpdateTask();        //Bildschirmaktualisierung während der Timer läuft
void countdownTickTask();       //Zeitanzeige mit eigener Zeitberechnung für den Timer
void masterControlProgram();    //Sicherheitstask zur Überwachung von max. Temp und Sensor Output

//Tasks
Task timerCalcTask(250, TASK_FOREVER, &updateRemainingMillis);
Task touchTask(5, TASK_FOREVER, &touchTaskCallback);
Task taskTempUpdate1(TEMP_READ_DELAY, TASK_FOREVER, &temperatureUpdateTask1);
Task taskTempUpdate2(TEMP_READ_DELAY, TASK_FOREVER, &temperatureUpdateTask2);
Task screenTask(20, TASK_FOREVER, &screenUpdateTask);
Task countdownTick(20, TASK_FOREVER, &countdownTickTask);
Task mcpTask(100, TASK_FOREVER, &masterControlProgram);

void countdownTickTask() {
  static int lastDisplayedSeconds = -1;

  if (timerRunning) {
    unsigned long elapsed = millis() - timerStartMillis;
    int newCountdownSeconds = (timeSetting * 60) - (elapsed / 1000);

    // Timerende prüfen
    if (newCountdownSeconds <= 0) {
      countdownSeconds = 0;
      timerRunning = false;
      timerExpired = true;
      stopAll();
    } else {
      countdownSeconds = newCountdownSeconds;
    }

    // Nur anzeigen, wenn sich die Sekunde geändert hat
    if (countdownSeconds != lastDisplayedSeconds) {
      lastDisplayedSeconds = countdownSeconds;
      updateCountdown();  // Anzeige aktualisieren
    }
  }
}


void updateRemainingMillis() {
  if (timerRunning && !timerExpired) {
    unsigned long timerMillis = millis();
    remainingMillis = (timeSetting * 60 * 1000) - (timerMillis - timerStartMillis);

    if ((remainingMillis <= 0) || (remainingMillis > (timeSetting * 60 * 1000))) {
      remainingMillis = 0;
      timerRunning = false;
      timerExpired = true;
#ifdef DEBUG
      Serial.println("Timer abgelaufen!");
#endif
      stopAll();
    }
  }
}

void handleNavigation(TPoint p) {
  int oldPage = currentPage;

  if (p.x > NAV_UP_TOUCH_X && p.x < NAV_UP_TOUCH_X + NAV_UP_TOUCH_W && p.y > NAV_UP_TOUCH_Y && p.y < NAV_UP_TOUCH_Y + NAV_UP_TOUCH_H) {
    currentPage++;
    if (currentPage > 3) currentPage = 1;
  } else if (p.x > NAV_DOWN_TOUCH_X && p.x < NAV_DOWN_TOUCH_X + NAV_DOWN_TOUCH_W && p.y > NAV_DOWN_TOUCH_Y && p.y < NAV_DOWN_TOUCH_Y + NAV_DOWN_TOUCH_H) {
    currentPage--;
    if (currentPage < 1) currentPage = 3;
  }

  if (oldPage != currentPage) {
    screenChanged = true;
#ifdef DEBUG
    Serial.print("Switching to page: ");
    Serial.println(currentPage);
#endif
  }
}

void processTouch(TPoint p, TEvent e) {

  // if ((e == TEvent::Tap) || (e == TEvent::TouchMove)) {
  if (e != TEvent::Tap && e != TEvent::DragStart && e != TEvent::DragMove && e != TEvent::DragEnd)
    return;

  // Umrechnen der X- und Y-Koordinaten
  int new_x = 320 - p.y;  // 320 - Y_Touch
  int new_y = p.x;        // X_Touch

  p.x = new_x;
  p.y = new_y;
  if ((old_x == new_x) && (old_y == new_y))
    return;  //deBouncing durch Abbruch bei identischen Werten für x und Y

  old_x = new_x;
  old_y = new_y;
#ifdef DEBUG
  Serial.print("Tap detected at X: ");
  Serial.print(p.x);
  Serial.print(" Y: ");
  Serial.println(p.y);
#endif
  switch (currentPage) {
    case 1:
      if (p.x > BUTTON_START_X && p.x < BUTTON_START_X + BUTTON_START_W && p.y > BUTTON_START_Y && p.y < BUTTON_START_Y + BUTTON_START_H) {
#ifdef DEBUG
        Serial.println(F("Start Button pressed"));
#endif
        if (!timerRunning) {
          timerRunning = true;
          timerStartMillis = millis();
          remainingMillis = timeSetting * 60 * 1000;
          countdownSeconds = timeSetting * 60;  // ← neue Anzeige-Zeit in Sekunden
          timerExpired = false;
        }
      }

      if (p.x > BUTTON_STOP_X && p.x < BUTTON_STOP_X + BUTTON_STOP_W && p.y > BUTTON_STOP_Y && p.y < BUTTON_STOP_Y + BUTTON_STOP_H) {
#ifdef DEBUG
        Serial.println(F("Stop Button pressed"));
#endif
        stopAll();
      }
      break;
    case 2:
#ifdef DEBUG
      Serial.println(F("Touchscreen 2"));
      Serial.print(p.x);
      Serial.print(" Y: ");
      Serial.println(p.y);
#endif
      // Slider-Berührung
      if (p.x > SLIDER_TEMP_X && p.x < SLIDER_TEMP_X + SLIDER_TEMP_W && p.y > SLIDER_TEMP_Y && p.y < SLIDER_TEMP_Y + SLIDER_TEMP_H) {
        targetTemperature = map(p.x, SLIDER_TEMP_X, SLIDER_TEMP_X + SLIDER_TEMP_W, tempMin, tempMax);
        drawSlider(SLIDER_TEMP_X, SLIDER_TEMP_Y, SLIDER_TEMP_W, targetTemperature, tempMin, tempMax, TFT_RED, lastTempSliderPos);
#ifdef DEBUG
        Serial.print(F("Temperature set to: "));
        Serial.println(targetTemperature);
#endif
      }
      if (p.x > SLIDER_TIME_X && p.x < SLIDER_TIME_X + SLIDER_TIME_W && p.y > SLIDER_TIME_Y && p.y < SLIDER_TIME_Y + SLIDER_TIME_H) {
        timeSetting = map(p.x, SLIDER_TIME_X, SLIDER_TIME_X + SLIDER_TIME_W, timeMin, timeMax);
        drawSlider(SLIDER_TIME_X, SLIDER_TIME_Y, SLIDER_TIME_W, timeSetting, timeMin, timeMax, TFT_BLUE, lastTimeSliderPos);
#ifdef DEBUG
        Serial.print(F("Time set to: "));
        Serial.println(timeSetting);
#endif
      }
      break;
    case 3:
      //Touchscreen 3;
#ifdef DEBUG
      Serial.println(F("Touchscreen 3"));
      Serial.print(p.x);
      Serial.print(" Y: ");
      Serial.println(p.y);
#endif
      break;
  }
  handleNavigation(p);

  //}
}


void updateScreen() {
  if (!screenChanged) return;

  tft.fillScreen(TFT_BLACK);  // Clear screen
  //tft.setCursor(50, 50);
  //tft.setTextColor(TFT_WHITE);

  switch (currentPage) {
    case 1:
      //tft.print("Screen 1");
      setupScreen1();
      break;
    case 2:
      //tft.print("Screen 2");
      setupScreen2();
      break;
    case 3:
      //tft.print("Screen 3");
      setupScreen3();
      break;
  }

  drawNavigationArrows();
  screenChanged = false;
}

void drawNavigationArrows() {
  // Oben (nächste Seite)
  tft.fillTriangle(NAV_UP_X, NAV_UP_Y, NAV_UP_X - 10, NAV_UP_Y - 20, NAV_UP_X - 20, NAV_UP_Y, TFT_WHITE);

  // Unten (vorherige Seite)
  tft.fillTriangle(NAV_DOWN_X, NAV_DOWN_Y, NAV_DOWN_X - 10, NAV_DOWN_Y + 20, NAV_DOWN_X - 20, NAV_DOWN_Y, TFT_WHITE);
}

void drawStartStopButtons() {
  // Start-Button (rechts unten)
  tft.fillRect(BUTTON_START_X, BUTTON_START_Y, BUTTON_START_W, BUTTON_START_H, TFT_GREEN);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(BUTTON_START_X + 14, BUTTON_START_Y + 28);
  tft.print("START");

  // Stop-Button (links unten)
  tft.fillRect(BUTTON_STOP_X, BUTTON_STOP_Y, BUTTON_STOP_W, BUTTON_STOP_H, TFT_RED);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(BUTTON_STOP_X + 16, BUTTON_STOP_Y + 28);
  tft.print("STOP");
}

void setupScreen1() {

  // Fülle den Bildschirm mit Hintergrundfarbe
  tft.fillScreen(TFT_BLACK);

  // Zeichne Textzeilen
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(CF_RT24);  // Setzt den Font auf Roboto Thin 24

  tft.setCursor(20, 20);
  tft.print("Ist-Temp 1: ");
  //tft.print(currentTemperature1);
  tft.print(temperature1);
  tft.print(" C");

  tft.setCursor(20, 50);
  tft.print("Ist-Temp 2: ");
  //tft.print(currentTemperature2);
  tft.print(temperature2);
  tft.print(" C");

  tft.setCursor(20, 80);
  tft.print("Soll-Temp:  ");
  tft.print(targetTemperature);
  tft.print(" C");

  if (!timerRunning) {
    // } else {
    tft.setCursor(20, 110);
    tft.print("Restzeit:   ");
    tft.print(timeSetting);
    tft.print(":00");
    tft.print(" Min");
  }

  // Start- und Stop-Buttons zeichnen
  drawStartStopButtons();
  drawNavigationArrows();
}

void setupScreen2() {
  // Seite 2: Solltemperatur und Zeit einstellen
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(CF_RT24);  // Setzt den Font auf Roboto Thin 24

  tft.setCursor(50, 50);
  tft.setTextColor(TFT_WHITE);
  tft.print("Soll-Temp: ");
  tft.print(targetTemperature);
  tft.print(" C");

  tft.setCursor(50, 120);
  tft.setTextColor(TFT_WHITE);
  tft.print("Zeit: ");
  tft.print(timeSetting);
  tft.print(" Min");

  // Slider für Temperatur zeichnen
  drawSlider(20, 80, 280, targetTemperature, tempMin, tempMax, TFT_RED, lastTempSliderPos);

  // Slider für Zeit zeichnen
  drawSlider(20, 150, 280, timeSetting, timeMin, timeMax, TFT_BLUE, lastTimeSliderPos);



  drawNavigationArrows();
}
// Slider zeichnen
void drawSlider(int x, int y, int w, int value, int minVal, int maxVal, uint16_t color, int &lastPos) {
  tft.drawRect(x, y, w, 10, TFT_WHITE);

  int sliderPos = map(value, minVal, maxVal, x, x + w);

  if (lastPos != -1 && lastPos != sliderPos) {
    tft.fillRect(lastPos - 5, y - 5, 10, 20, TFT_BLACK);
  }

  tft.fillRect(sliderPos - 5, y - 5, 10, 20, color);

  lastPos = sliderPos;
}

void setupScreen3() {

  // Fülle den Bildschirm mit Hintergrundfarbe
  tft.fillScreen(TFT_DARKGREEN);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFreeFont(CF_RT24);  // Setzt den Font auf Roboto Thin 24

  tft.setCursor(30, 50);
  //tft.setTextColor(TFT_WHITE);
  tft.print("#diy #Vinylflattener");
  tft.setCursor(30, 90);
  tft.print("NextGen V1.0, 2025");
  tft.setCursor(30, 130);
  tft.print("contact:");
  tft.setCursor(30, 170);
  tft.print("@vincent05.bsky.social");

  drawNavigationArrows();  // Zeichne Navigationspfeile
}

void screenUpdateTask() {

  if (currentPage == 1) {
    unsigned long updateMillis = millis();
    if (updateMillis - lastUpdate >= 500) {
      if ((abs(changeTemp1) >= 0.06) || (abs(changeTemp2) >= 0.06)) {
        tft.fillRect(150, 0, 120, 24, TFT_BLACK);
        tft.setCursor(20, 20);
        tft.setTextColor(TFT_WHITE);
        tft.print("Ist-Temp 1: ");
        //tft.print(currentTemperature1); //Darstellung der Sensordaten
        tft.print(temperature1);  //fehlerbereinigte Darstellung
        tft.print(" C");

        tft.fillRect(150, 30, 120, 24, TFT_BLACK);
        tft.setCursor(20, 50);
        tft.setTextColor(TFT_WHITE);
        tft.print("Ist-Temp 2: ");
        //tft.print(currentTemperature2); //fehlerbereinigte Darstellung
        tft.print(temperature2);  //fehlerbereinigte Darstellung
        tft.print(" C");
      }

      lastUpdate = updateMillis;
    }
  } else if (currentPage == 2) {
    updateControlPage();
  }
}

// Countdown-Timer aktualisieren
void updateCountdown() {
#ifdef DEBUG
  Serial.println("update Countdown");

  Serial.print("UpdateCountdown @ ");
  Serial.println(millis());
#endif

  static int lastShown = -1;
  if ((remainingMillis > 0) && !(timerExpired)) {
    if (countdownSeconds != lastShown) {
      lastShown = countdownSeconds;
      int minutes = countdownSeconds / 60;
      int seconds = countdownSeconds % 60;
      if (currentPage == 1) {                   //Anzeige nur auf der ersten Seite aktualisieren
          tft.fillRect(130, 90, 180, 24, TFT_BLACK);
        tft.setCursor(20, 110);
        tft.setTextColor(TFT_WHITE);
        tft.print("Restzeit:   ");
        tft.printf("%02d:%02d", minutes, seconds);
        tft.print(" Min");
      }
    }
  }
}

// Dynamische Inhalte auf Seite 2 aktualisieren (Slider und Werte)
void updateControlPage() {
  if (targetTemperature != lastTargetTemp) {
    tft.fillRect(50, 30, 220, 24, TFT_BLACK);
    tft.setCursor(50, 50);
    tft.setTextColor(TFT_WHITE);
    tft.print("Soll-Temp: ");
    tft.print(targetTemperature);
    tft.print(" C");

    drawSlider(20, 80, 280, targetTemperature, tempMin, tempMax, TFT_RED, lastTempSliderPos);

    lastTargetTemp = targetTemperature;
  }

  if (timeSetting != lastTimeSetting) {
    tft.fillRect(50, 100, 220, 24, TFT_BLACK);
    tft.setCursor(50, 120);
    tft.setTextColor(TFT_WHITE);
    tft.print("Zeit: ");
    tft.print(timeSetting);
    tft.print(" Min");

    drawSlider(20, 150, 280, timeSetting, timeMin, timeMax, TFT_BLUE, lastTimeSliderPos);

    lastTimeSetting = timeSetting;
  }
}


void stopAll() {
  Serial.println("Stop All");
  timerRunning = false;
  timerExpired = true;

  RelayControl1 = 0;  //Relais aus
  RelayControl2 = 0;

  int minutes = timeSetting;  //Sliderwert abfragen
  int seconds = 0;
  if (currentPage == 1) {  // nur aktualisieren, wenn Startseite aktiv ist
    tft.setFreeFont(CF_RT24);
    tft.fillRect(20, 90, 300, 24, TFT_BLACK);
    tft.setCursor(20, 110);           //wenn Bildschirmseite 1 aktiv ist
    tft.setTextColor(TFT_DARKGREEN);  //Schrift für Restzeit auf dunkelgrün
    tft.print("Restzeit:   ");
    tft.printf("%02d:%02d", minutes, seconds);  //Zurücksetzen auf Sliderwert
    tft.print(" Min");
  }
}

void stopError() {
  Serial.println("Stop due to system error");
  timerRunning = false;
  timerExpired = true;

  RelayControl1 = 0;  //Relais aus
  RelayControl2 = 0;

  touchTask.disable();
  taskTempUpdate1.disable();
  taskTempUpdate2.disable();
  screenTask.disable();
  timerCalcTask.disable();
  countdownTick.disable();
}

void setup() {
  Serial.begin(115200);
#ifdef DEBUG
  Serial.println(F("Starting..."));  //Laufzeit Debugging
#endif
  startTime = millis();
  pinMode(T_RST, OUTPUT);
  pinMode(T_INT, INPUT);
  Wire.begin();
  Wire.setClock(400000);
  digitalWrite(T_RST, LOW);  //Initialisierung Touch Controller
  delay(300);
  digitalWrite(T_RST, HIGH);
  delay(300);

  ts.begin();
  ts.registerTouchHandler(processTouch);

  tft.init();
  tft.setRotation(3);

  updateScreen();  // Initialisierung Bildschirm

  //Initialisierung Temperaturregelung und GPIO Pins

  pinMode(TEMP_SENSOR1_PIN, INPUT);
  pinMode(TEMP_SENSOR2_PIN, INPUT);
  pinMode(RELAIS1, OUTPUT);
  pinMode(RELAIS2, OUTPUT);

  // Relais per default ausschalten
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);


  temperatureSensor1.begin();
  temperatureSensor2.begin();
  temperatureSensor1.setWaitForConversion(false);  // wichtig!!! Blockiert im ESP keine weiteren Tasks, während der Abfrage
  temperatureSensor2.setWaitForConversion(false);
  temperatureSensor1.requestTemperatures();
  temperatureSensor2.requestTemperatures();

  currentTemperature1 = temperature1;
  currentTemperature2 = temperature2;

  //BangBang Temperatur festlegen
  myPID1.setBangBang(10, 0.3);
  myPID2.setBangBang(10, 0.3);
  //setze PID update interval auf variablen Wert 'pidTimeStep'
  myPID1.setTimeStep(pidTimeStep);
  myPID2.setTimeStep(pidTimeStep);

  lastTempUpdate1 = millis();  //initial value
  lastTempUpdate2 = millis();  //initial value
  setPoint = targetTemperature;

  // Tasks starten

  runner.init();
  runner.addTask(timerCalcTask);
  runner.addTask(touchTask);
  runner.addTask(taskTempUpdate1);
  runner.addTask(taskTempUpdate2);
  runner.addTask(screenTask);
  runner.addTask(countdownTick);
  runner.addTask(mcpTask);

  timerCalcTask.enable();
  countdownTick.enable();
  touchTask.enable();
  taskTempUpdate1.enable();
  taskTempUpdate2.enable();
  screenTask.enable();
  mcpTask.enable();
}

void loop() {

  endTime = millis();

  runner.execute();  // Scheduler steuert die Tasks

  //Temperaturregelung
  Time = millis();
  if (timerRunning && !timerExpired) {
    TemperatureRamp();

    myPID1.run();  //call every loop, updates automatically at certain time interval
    myPID2.run();  //call every loop, updates automatically at certain time interval
  }
  if (RelayControl1 != 0) {
    digitalWrite(RELAIS1, HIGH);
  } else {
    digitalWrite(RELAIS1, LOW);
  }
  if (RelayControl2 != 0) {
    digitalWrite(RELAIS2, HIGH);
  } else {
    digitalWrite(RELAIS2, LOW);
  }

  updateScreen();  // Bildschirm Update falls notwendig

  duration = endTime - startTime;
  if (duration > 1) {
#ifdef DEBUG
    Serial.print("Laufzeit: ");
    Serial.println(duration);
#endif
  }
  startTime = millis();
}

void touchTaskCallback() {
  ts.loop();  // Touch-Abfrage ausführen // Hier kannst du weitere Aktionen ausführen, wenn ein Touch-Event erkannt wird
  if (ts.touched()) {
    //  unsigned char result = ts.touched();
    unsigned char result = ts.getCachedTouches();

#ifdef DEBUG
    Serial.println("Touch detected");
    Serial.println(result);
#endif
  }
}

void temperatureUpdateTask1() {

  if ((millis() - lastTempUpdate1) > TEMP_READ_DELAY) {
    temperature1 = temperatureSensor1.getTempCByIndex(0);  //get temp reading
    lastTempUpdate1 = millis();
    temperatureSensor1.requestTemperatures();  //request reading for next time
    changeTemp1 = currentTemperature1 - temperature1;
    currentTemperature1 = temperature1;
    if (currentTemperature1 < 0) {  //errorhandling, wenn der Sensor -127 zurück gibt
      temperature1 = targetTemperature;
#ifdef DEBUGSENSOR
      Serial.println("error reading Temperature Sensor 1");
#endif
    }
  }
}

void temperatureUpdateTask2() {
  if ((millis() - lastTempUpdate2) > TEMP_READ_DELAY) {
    temperature2 = temperatureSensor2.getTempCByIndex(0);  //get temp reading
    lastTempUpdate2 = millis();
    temperatureSensor2.requestTemperatures();  //request reading for next time
    changeTemp2 = currentTemperature2 - temperature2;
    currentTemperature2 = temperature2;
    if (currentTemperature2 < 0) {  //errorhandling, wenn der Sensor -127 zurück gibt
      temperature2 = targetTemperature;
#ifdef DEBUGSENSOR
      Serial.println("error reading Temperature Sensor 2");
#endif
    }
  }
}

// Sicherstellen, dass die Temperatur in Richtung Zielwert nicht zu stark ansteigt
void TemperatureRamp() {
  if ((millis() - RampUpdate) > RampTime) {
    powerupTemperature = targetTemperature - powerUpFactor;
    RampUpdate = millis();
    if (!((temperature1 < powerupTemperature) && (temperature2 < powerupTemperature))) {
      if (temperature1 < temperature2) {
        currentSetpoint = temperature1;
      } else {
        currentSetpoint = temperature2;
      }

      if (currentSetpoint < targetTemperature) {
        currentSetpoint += rampRate;
        setPoint = currentSetpoint;
        if (currentSetpoint > targetTemperature) {
          currentSetpoint = targetTemperature;
          setPoint = currentSetpoint;
        }
      }
    } else {
      setPoint = targetTemperature;
#ifdef DEBUGSENSOR
      Serial.println("Heating!!!");
#endif
    }
#ifdef DEBUGSENSOR
    Serial.print("Temperatur 1: ");
    Serial.println(temperature1);
    Serial.print("Temperatur 2: ");
    Serial.println(temperature2);
    Serial.print("currentSetPoint: ");
    Serial.println(currentSetpoint);
    Serial.print("SetPoint: ");
    Serial.println(setPoint);
    Serial.print("powerUpTemperature: ");
    Serial.println(powerupTemperature);
    Serial.print("powerUpFactor: ");
    Serial.println(powerUpFactor);
#endif
  }
}

void masterControlProgram() {

  double stopTemperature = targetTemperature + tempThreshold;
  if ((temperature1 > stopTemperature) || (temperature2 > stopTemperature)) {
#ifdef DEBUGMCP
    Serial.print("Temperatur zu hoch: ");
    Serial.println(stopTemperature);
    Serial.print("Zieltemperatur: ");
    Serial.print(targetTemperature);
    Serial.print(" Schwellwert: ");
    Serial.println(tempThreshold);
#endif
    if (timerRunning) {  //nur auf Temperaturfehler reagieren, wenn der Timer läuft
      stopcount++;
      if ((stopcount > 2) && (stopcount < 5)) {
        String alarmtext = "      Ueberhitzung";
        errorScreen(alarmtext);
        stopError();
      }
    }
  } else if (currentTemperature1 < 0) {  //Temperatursensor 1 prüfen
    errorcount++;                        //Fehlerzähler um 1 erhöhen
#ifdef DEBUGMCP
    Serial.print("Fehler Sensor 1 entdeckt: ");
    Serial.println(errorcount);
#endif
    if (errorcount > 100) {  //erst verzögert reagieren, um Einzelfehler zu ignorieren
#ifdef DEBUGMCP
      Serial.print("Sensor 1 ausgefallen ");
      Serial.println(currentTemperature1);
#endif
      stopcount++;
      if ((stopcount > 2) && (stopcount < 5)) {
        String alarmtext = "Sensor 1 ausgefallen ";
        errorScreen(alarmtext);
      }
    }
  } else if (currentTemperature2 < 0) {  //Temperatursensor 2 prüfen
    errorcount++;                        //Fehlerzähler um 1 erhöhen
#ifdef DEBUGMCP
    Serial.print("Fehler Sensor 2 entdeckt: ");
    Serial.println(errorcount);
#endif
    if (errorcount > 100) {  //erst verzögert reagieren, um Einzelfehler zu ignorieren
#ifdef DEBUGMCP
      Serial.print("Sensor 2 ausgefallen ");
      Serial.println(currentTemperature2);
#endif
      stopcount++;
      if ((stopcount > 2) && (stopcount < 5)) {
        String alarmtext = "Sensor 2 ausgefallen ";
        errorScreen(alarmtext);
      }
    }
  } else {
    errorcount = 0;
    stopcount = 0;
  }
}
void errorScreen(String outText) {
  tft.fillScreen(TFT_ORANGE);
  tft.setFreeFont(CF_FSB12);  // Setzt den Font auf Roboto Thin 24

  tft.setCursor(28, 50);
  tft.setTextColor(TFT_BLUE);
  tft.print("!!! SYSTEM HALTED !!!");
  tft.setCursor(35, 120);
  tft.print(outText);
  stopError();  // alle Funktionstasks anhalten
}