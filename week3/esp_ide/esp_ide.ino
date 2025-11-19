#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- PIN CONFIG ----------
#define MPU_SDA_PIN 21
#define MPU_SCL_PIN 22

#define TRIG_PIN 4
#define ECHO_PIN 2

#define ENC_LEFT_PIN 34
#define ENC_RIGHT_PIN 35

#define IN1_LEFT 25
#define IN2_LEFT 26
#define PWM_LEFT_PIN 14
#define PWM_LEFT_CH 0

#define IN1_RIGHT 27
#define IN2_RIGHT 12
#define PWM_RIGHT_PIN 13
#define PWM_RIGHT_CH 1

#define PWM_FREQ 20000
#define PWM_RES 8

#define PULSES_PER_REV 20.0
#define SENSOR_PERIOD 100   // ms

// ---------- OBJECTS ----------
Adafruit_MPU6050 mpu;

// ---------- GLOBALS ----------
volatile long encLeft = 0, encRight = 0;

float leftRPM = 0, rightRPM = 0;
unsigned long lastRPMCalc = 0;

// IMU complementary filter
float pitch = 0, roll = 0;
unsigned long lastIMU = 0;

// PI control
float leftSet = 0, rightSet = 0;
float leftPWM = 0, rightPWM = 0;
float Kp = 1.8, Ki = 0.12;
float leftErrSum = 0, rightErrSum = 0;
unsigned long lastPID = 0;

unsigned long lastSensorSend = 0;

// ---------- ISR ----------
void IRAM_ATTR leftISR() { encLeft++; }
void IRAM_ATTR rightISR() { encRight++; }

// ---------- ULTRASONIC ----------
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long t = pulseIn(ECHO_PIN, HIGH, 30000);
  if (t == 0) return 999.0;

  return (t / 2.0) / 29.1;   // distance in cm
}

// ---------- MOTOR CONTROL ----------
void motorLeft(float pwm) {
  pwm = constrain(pwm, -255, 255);
  leftPWM = pwm;

  if (pwm > 0) {
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    ledcWrite(PWM_LEFT_CH, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    ledcWrite(PWM_LEFT_CH, -pwm);
  } else {
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, LOW);
    ledcWrite(PWM_LEFT_CH, 0);
  }
}

void motorRight(float pwm) {
  pwm = constrain(pwm, -255, 255);
  rightPWM = pwm;

  if (pwm > 0) {
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
    ledcWrite(PWM_RIGHT_CH, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, HIGH);
    ledcWrite(PWM_RIGHT_CH, -pwm);
  } else {
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, LOW);
    ledcWrite(PWM_RIGHT_CH, 0);
  }
}

// ---------- RPM CALC ----------
void computeRPM() {
  unsigned long now = millis();
  if (now - lastRPMCalc < 200) return;

  float dt = (now - lastRPMCalc) / 1000.0;
  lastRPMCalc = now;

  noInterrupts();
  long l = encLeft; encLeft = 0;
  long r = encRight; encRight = 0;
  interrupts();

  leftRPM = (l / PULSES_PER_REV) / dt * 60.0;
  rightRPM = (r / PULSES_PER_REV) / dt * 60.0;
}

// ---------- IMU FILTER ----------
void readIMU() {
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) return;

  unsigned long now = millis();
  float dt = (now - lastIMU) / 1000.0;
  lastIMU = now;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.29578;
  float rollAcc = atan2(ay, az) * 57.29578;

  float gx = g.gyro.x * 57.29578;
  float gy = g.gyro.y * 57.29578;

  float alpha = 0.98;
  pitch = alpha * (pitch + gx * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll + gy * dt) + (1 - alpha) * rollAcc;
}

// ---------- PI SPEED CONTROL ----------
void speedControl() {
  unsigned long now = millis();
  if (now - lastPID < 50) return;
  float dt = (now - lastPID) / 1000.0;
  lastPID = now;

  float eL = leftSet - leftRPM;
  float eR = rightSet - rightRPM;

  leftErrSum += eL * dt;
  rightErrSum += eR * dt;

  float outL = Kp * eL + Ki * leftErrSum;
  float outR = Kp * eR + Ki * rightErrSum;

  motorLeft(outL);
  motorRight(outR);
}

// ---------- SERIAL COMMANDS ----------
void handleCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if (c == 'f') { leftSet = rightSet = 40; Serial.println("FORWARD"); }
  if (c == 'b') { leftSet = rightSet = -30; Serial.println("BACK"); }
  if (c == 'l') { leftSet = 20; rightSet = 40; Serial.println("LEFT"); }
  if (c == 'r') { leftSet = 40; rightSet = 20; Serial.println("RIGHT"); }
  if (c == 's') { leftSet = rightSet = 0; leftErrSum = rightErrSum = 0; Serial.println("STOP"); }
}

// ---------- SEND JSON ----------
void sendJSON() {
  unsigned long now = millis();
  if (now - lastSensorSend < SENSOR_PERIOD) return;
  lastSensorSend = now;

  float us = readUltrasonic();

  String j = "{";
  j += "\"time\":" + String(millis());
  j += ",\"pitch\":" + String(pitch, 2);
  j += ",\"roll\":" + String(roll, 2);
  j += ",\"ultra_cm\":" + String(us, 2);
  j += ",\"rpm_l\":" + String(leftRPM, 2);
  j += ",\"rpm_r\":" + String(rightRPM, 2);
  j += ",\"pwm_l\":" + String(leftPWM, 2);
  j += ",\"pwm_r\":" + String(rightPWM, 2);
  j += "}";

  Serial.println(j);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  // IMU
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  if (!mpu.begin()) Serial.println("MPU ERROR");

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Encoders
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(ENC_LEFT_PIN, leftISR, RISING);
  attachInterrupt(ENC_RIGHT_PIN, rightISR, RISING);

  // Motor init
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);

  ledcSetup(PWM_LEFT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_LEFT_PIN, PWM_LEFT_CH);

  ledcSetup(PWM_RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_RIGHT_PIN, PWM_RIGHT_CH);

  motorLeft(0);
  motorRight(0);
}

// ---------- LOOP ----------
void loop() {
  readIMU();
  computeRPM();
  speedControl();
  sendJSON();
  handleCommands();
}
