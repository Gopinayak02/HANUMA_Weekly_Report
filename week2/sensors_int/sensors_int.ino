/********************
HANUMA Rover - Sensor Integration
Arduino Uno
HW-506 Temperature, HW-499 Light Cup,
HW-505/HW-513 Shock & Vibration,
HW-040 Rotary Encoder, HW-488 IR Obstacle,
DS3231 RTC
********************/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "RTClib.h"
#include <Encoder.h>

// --- Pin Definitions ---
#define TEMP_PIN 2             // HW-506 DS18B20
#define LIGHT_CUP_PIN 3        // HW-499
#define SHOCK_PIN 4            // HW-505
#define VIB_PIN 5              // HW-513
#define ROTARY_CLK 6           // HW-040
#define ROTARY_DT 7
#define ROTARY_SW 8
#define IR_PIN 9               // HW-488

// --- Objects ---
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
RTC_DS3231 rtc;
Encoder rotary(ROTARY_CLK, ROTARY_DT);

// --- Variables ---
long encoderPos = 0;

void setup() {
  Serial.begin(9600);

  // Setup digital inputs
  pinMode(LIGHT_CUP_PIN, INPUT);
  pinMode(SHOCK_PIN, INPUT);
  pinMode(VIB_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP); // Rotary button

  // Initialize sensors
  tempSensor.begin();
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1); // Stop if RTC is missing
  }

  Serial.println("HANUMA Rover Sensors Initialized.");
}

void loop() {
  // --- Temperature (HW-506) ---
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  // --- Light Cup Sensor (HW-499) ---
  int lightCupState = digitalRead(LIGHT_CUP_PIN);

  // --- Shock & Vibration (HW-505 / HW-513) ---
  int shockState = digitalRead(SHOCK_PIN);
  int vibState = digitalRead(VIB_PIN);

  // --- Rotary Encoder (HW-040) ---
  encoderPos = rotary.read();
  int rotaryButton = digitalRead(ROTARY_SW);

  // --- IR Obstacle Sensor (HW-488) ---
  int irState = digitalRead(IR_PIN);

  // --- RTC Time ---
  DateTime now = rtc.now();

  // --- Print All Sensor Values ---
  Serial.println("------ Sensor Values ------");
  Serial.print("HW-506 Temperature: "); Serial.print(tempC); Serial.println(" C");
  Serial.print("HW-499 Light Cup: "); Serial.println(lightCupState);
  Serial.print("HW-505 Shock: "); Serial.println(shockState);
  Serial.print("HW-513 Vibration: "); Serial.println(vibState);
  Serial.print("HW-040 Rotary Encoder Position: "); Serial.println(encoderPos);
  Serial.print("Rotary Button: "); Serial.println(rotaryButton);
  Serial.print("HW-488 IR Obstacle: "); Serial.println(irState);
  Serial.print("RTC Time: "); Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":"); Serial.println(now.second());
  Serial.println("---------------------------\n");

  delay(500); // Update every 0.5 seconds
}
