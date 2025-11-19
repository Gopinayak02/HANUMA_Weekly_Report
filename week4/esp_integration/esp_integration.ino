/********************
HANUMA Rover -> ESP8266 + ADS1115 -> ThingSpeak
ThingSpeak write API key: 74E5YNJBWRUM3A8R
Channel ID: 3144805

Field mapping (ThingSpeak):
field1 = temperature_C         (float)
field2 = mq4                   (integer 0/1)  <-- boolean-like thresholded MQ-4
field3 = shock                 (0/1)
field4 = vibration             (0/1)
field5 = rotary_pos            (integer)
field6 = rotary_button         (0 = pressed, 1 = released)
field7 = ir_obstacle           (0/1)
field8 = mq7                   (integer raw ADS value)

NOTE: MQ-7 heater cycle not implemented — only analog read is transmitted.
Update interval set to 20 seconds (ThingSpeak min interval ≈ 15s).
********************/

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Encoder.h>
#include <Adafruit_ADS1X15.h>

// ---------- WiFi & ThingSpeak ----------
const char* ssid = "Hemanth";
const char* password = "12345679";

const char* THINGSPEAK_SERVER = "http://api.thingspeak.com/update";
const char* THINGSPEAK_API_KEY = "74E5YNJBWRUM3A8R";
const unsigned long THINGSPEAK_INTERVAL_MS = 20000UL; // 20 seconds between updates

// --- Pin Definitions (ESP8266 GPIO numbers) ---
#define TEMP_PIN       2   // GPIO2  (DS18B20 Data)
#define LIGHT_CUP_PIN  14  // GPIO14 (Light Cup)    <-- not used as a ThingSpeak field by name but earlier mapping confusion; we send MQ4 threshold instead
#define SHOCK_PIN      12  // GPIO12 (Shock)
#define VIB_PIN        13  // GPIO13 (Vibration)
#define ROTARY_CLK     5   // GPIO5  (Rotary CLK)  <-- note: same pins as I2C SCL/SDA below; see caution
#define ROTARY_DT      4   // GPIO4  (Rotary DT)
#define ROTARY_SW      16  // GPIO16 (Rotary Switch Button)
#define IR_PIN         15  // GPIO15 (IR Obstacle) - safe if kept LOW at boot

// ADS1115 channels for MQ sensors
#define MQ4_CHANNEL    0   // ADS A0
#define MQ7_CHANNEL    1   // ADS A1

// ---------- ADS1115 & sensors ----------
Adafruit_ADS1115 ads;  // default address 0x48
const adsGain_t ADS_GAIN = GAIN_ONE; // ±4.096V range (best resolution)
const float ADS_FSR = 4.096F;        // full-scale (V) for GAIN_ONE
const float VCC = 3.3F;              // circuit Vcc used to compute Rs (change to 5.0 if MQ board powered at 5V)
const float RL_VALUE = 10000.0F;     // load resistor on MQ sensors (10kΩ assumed)

// threshold to convert MQ-4 analog reading into boolean-like 0/1 for field2
// adjust depending on sensor behavior; example: >1.5V => 1 (gas present)
const float MQ4_THRESHOLD_V = 1.5F;

// ---------- OneWire / Temp / Encoder ----------
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
Encoder rotary(ROTARY_CLK, ROTARY_DT);

// ---------- Globals ----------
unsigned long lastThingSpeakPush = 0;

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("HANUMA Rover -> ThingSpeak sketch starting...");

  // WiFi connect
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  Serial.print(" ...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if (millis() - start > 15000UL) { // 15s timeout
      Serial.println();
      Serial.println("WiFi connect timed out. Continuing without network (check credentials).");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
  }

  // I2C (Wire) initialization: default pins SDA=GPIO4, SCL=GPIO5 on ESP8266
  Wire.begin(); // uses GPIO4 (SDA) and GPIO5 (SCL) by default

  // ADS1115 init
  if (!ads.begin()) {
    Serial.println("ERROR: Failed to initialize ADS1115. Check wiring.");
    // do not hang; continue so you can debug
  } else {
    ads.setGain(ADS_GAIN);
    Serial.println("ADS1115 initialized.");
  }

  // Setup digital inputs
  pinMode(LIGHT_CUP_PIN, INPUT);
  pinMode(SHOCK_PIN, INPUT);
  pinMode(VIB_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP);  // button -> GND

  // DS18B20
  tempSensor.begin();

  Serial.println("Setup complete.");
  lastThingSpeakPush = millis() - THINGSPEAK_INTERVAL_MS; // allow immediate push
}

void loop() {
  // Read sensors
  // Temperature
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  // Digital sensors
  int shockState = digitalRead(SHOCK_PIN);     // 0 or 1
  int vibState   = digitalRead(VIB_PIN);
  int rotaryButton = digitalRead(ROTARY_SW);   // 0 = pressed, 1 = released
  long rotaryPos = rotary.read();
  int irState = digitalRead(IR_PIN);

  // MQ sensors via ADS1115
  int16_t rawMQ4 = 0, rawMQ7 = 0;
  float vMQ4 = 0.0F, vMQ7 = 0.0F;
  bool adsOk = true;
  if (ads.begin()) { // if ADS initialized
    rawMQ4 = ads.readADC_SingleEnded(MQ4_CHANNEL);
    rawMQ7 = ads.readADC_SingleEnded(MQ7_CHANNEL);
    // convert raw to voltage based on FS range for selected gain
    vMQ4 = (float)rawMQ4 * (ADS_FSR / 32767.0F);
    vMQ7 = (float)rawMQ7 * (ADS_FSR / 32767.0F);
  } else {
    adsOk = false;
  }

  // Convert MQ-4 to boolean-like integer 0/1 based on threshold voltage
  int mq4_bool = 0;
  if (adsOk) {
    mq4_bool = (vMQ4 > MQ4_THRESHOLD_V) ? 1 : 0;
  } else {
    mq4_bool = 0; // fallback
  }

  // For MQ7 we will send raw ADC integer (rawMQ7) as Field8
  int mq7_field_value = rawMQ7; // can be negative if differential reading; SingleEnded should give 0..32767

  // Print diagnostics locally
  Serial.println("---- Local sensor readout ----");
  if (tempC == DEVICE_DISCONNECTED_C) Serial.println("Temperature: DISCONNECTED");
  else {
    Serial.print("Temperature (C): "); Serial.println(tempC, 2);
  }
  Serial.print("MQ4 Vout: "); Serial.print(vMQ4, 4); Serial.print(" V   -> mq4_bool: "); Serial.println(mq4_bool);
  Serial.print("MQ7 Vout: "); Serial.print(vMQ7, 4); Serial.print(" V   raw: "); Serial.println(rawMQ7);
  Serial.print("Shock: "); Serial.print(shockState);
  Serial.print("  Vibration: "); Serial.print(vibState);
  Serial.print("  IR: "); Serial.print(irState);
  Serial.print("  Rotary Pos: "); Serial.print(rotaryPos);
  Serial.print("  Rotary Btn: "); Serial.println(rotaryButton);
  Serial.println("-------------------------------");

  // Push to ThingSpeak at interval
  if (millis() - lastThingSpeakPush >= THINGSPEAK_INTERVAL_MS) {
    lastThingSpeakPush = millis();
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      WiFiClient client;

      // Prepare POST body as x-www-form-urlencoded
      // fields: 1..8 following user mapping
      String postData = "";
      // field1 = temperature_C (float)
      postData += "api_key=";
      postData += THINGSPEAK_API_KEY;
      postData += "&field1=";
      if (tempC == DEVICE_DISCONNECTED_C) postData += "NaN";
      else postData += String(tempC, 2);

      postData += "&field2=";
      postData += String(mq4_bool);          // MQ-4 boolean-like

      postData += "&field3=";
      postData += String(shockState);

      postData += "&field4=";
      postData += String(vibState);

      postData += "&field5=";
      postData += String(rotaryPos);

      postData += "&field6=";
      postData += String(rotaryButton);

      postData += "&field7=";
      postData += String(irState);

      postData += "&field8=";
      postData += String(mq7_field_value);

      Serial.print("ThingSpeak POST: ");
      Serial.println(postData);

      http.begin(client, THINGSPEAK_SERVER); // HTTP
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      int httpCode = http.POST(postData);
      if (httpCode > 0) {
        String payload = http.getString();
        Serial.print("ThingSpeak HTTP code: ");
        Serial.println(httpCode);
        Serial.print("ThingSpeak response body: ");
        Serial.println(payload);
      } else {
        Serial.print("ThingSpeak POST failed, error: ");
        Serial.println(http.errorToString(httpCode).c_str());
      }
      http.end();
    } else {
      Serial.println("Not connected to WiFi; skipping ThingSpeak push.");
    }
  }

  delay(200); // small delay for loop smoothing
}