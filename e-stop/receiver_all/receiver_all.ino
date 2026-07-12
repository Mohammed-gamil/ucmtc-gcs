/*
  RECEIVER  -  ESP32-S3
  ----------------------
  Listens for state packets from the transmitter over ESP-NOW, and
  reads serial drive/estop/resume commands from the host (Jetson Nano)
  to drive 4 independent motors via Cytron drivers.

  Includes:
  - 4 independent Cytron channels (PWM+DIR) via LEDC at 20 kHz.
  - Formatted serial parser for <linear_x_int,angular_z_int,drift_active_int,checksum>\n.
  - Software E-stop (E\n or <ESTOP>\n) and Resume (R\n or <RESET>\n).
  - Physical E-stop on GPIO 34 (Normally Closed to GND, open/HIGH = STOP).
  - Local watchdog: drops to safe state if no serial command in 300 ms.
  - On-board 4-wheel independent skid-steer mixing with speed-sensitive turning and drift traction vectoring.
*/

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#define LINK_TIMEOUT  300   // ms - watchdog timeout

// ---- Cytron Pin Configurations ----
#define FL_PWM_PIN 32
#define FL_DIR_PIN 33
#define RL_PWM_PIN 25
#define RL_DIR_PIN 26
#define FR_PWM_PIN 27
#define FR_DIR_PIN 14
#define RR_PWM_PIN 12
#define RR_DIR_PIN 13

// ---- LEDC Setup ----
#define LEDC_FREQUENCY 20000 // 20 kHz
#define LEDC_RESOLUTION 8    // 8-bit duty cycle (0-255)

#define FL_CH 0
#define RL_CH 1
#define FR_CH 2
#define RR_CH 3

// ---- Physical E-Stop Pin ----
#define ESTOP_PIN 34

// ---- Kinematics & Mixing Parameters ----
const float TRACK_WIDTH = 0.5f;                    // meters
const float MAX_WHEEL_SPEED = 2.5f;                // m/s
const float BIAS_FL = 1.0f;
const float BIAS_FR = 1.0f;
const float BIAS_RL = 1.0f;
const float BIAS_RR = 1.0f;
const float DRIFT_TRACTION_INNER = 0.6f;
const float DRIFT_TRACTION_OUTER = 1.0f;

// ---- State Variables ----
typedef struct struct_message {
  bool stopped;
  int  commandValue;
} struct_message;

struct_message incomingData;
volatile unsigned long lastReceiveTime = 0;
volatile bool haveData = false;

volatile bool estopLatched = false;
bool serialStopActive = false;

// Last received inputs from Jetson serial bridge (scaled by 1000)
float targetLinearX = 0.0f;
float targetAngularZ = 0.0f;
bool targetDriftActive = false;

unsigned long lastCommandTime = 0;

// Force all driver pins LOW immediately.
void forceAllSafe() {
  ledcWrite(FL_CH, 0);
  ledcWrite(RL_CH, 0);
  ledcWrite(FR_CH, 0);
  ledcWrite(RR_CH, 0);
  digitalWrite(FL_DIR_PIN, LOW);
  digitalWrite(RL_DIR_PIN, LOW);
  digitalWrite(FR_DIR_PIN, LOW);
  digitalWrite(RR_DIR_PIN, LOW);
}

// Hardware E-Stop ISR
void IRAM_ATTR estopISR() {
  if (digitalRead(ESTOP_PIN) == HIGH) { // NC is open (pressed or cut)
    estopLatched = true;
    // Shut off all PWM channels instantly in hardware
    ledcWrite(FL_CH, 0);
    ledcWrite(RL_CH, 0);
    ledcWrite(FR_CH, 0);
    ledcWrite(RR_CH, 0);
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  lastReceiveTime = millis();
  haveData = true;
}

void setup() {
  Serial.begin(115200);

  // Set up DIR pins
  pinMode(FL_DIR_PIN, OUTPUT);
  pinMode(RL_DIR_PIN, OUTPUT);
  pinMode(FR_DIR_PIN, OUTPUT);
  pinMode(RR_DIR_PIN, OUTPUT);

  // Set up PWM pins via LEDC
  ledcSetup(FL_CH, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(FL_PWM_PIN, FL_CH);

  ledcSetup(RL_CH, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(RL_PWM_PIN, RL_CH);

  ledcSetup(FR_CH, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(FR_PWM_PIN, FR_CH);

  ledcSetup(RR_CH, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(RR_PWM_PIN, RR_CH);

  // Set up hardware E-Stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);
  
  // Initial check
  if (digitalRead(ESTOP_PIN) == HIGH) {
    estopLatched = true;
  }

  forceAllSafe();

  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("Receiver ready.");
}

void setMotor(int ch, int dirPin, int value) {
  // Value is in -1000 .. 1000
  if (value >= 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  int duty = (abs(value) * 255) / 1000;
  duty = constrain(duty, 0, 255);
  ledcWrite(ch, duty);
}

// 4-wheel independent skid-steer mixing on-board
void mixAndDrive(float linear_x, float angular_z, bool drift_active) {
  float fl_t = 1.0f, fr_t = 1.0f, rl_t = 1.0f, rr_t = 1.0f;
  if (drift_active) {
    if (angular_z > 0.05f) { // Left turn -> Left wheels (FL, RL) are inner
      fl_t = rl_t = DRIFT_TRACTION_INNER;
      fr_t = rr_t = DRIFT_TRACTION_OUTER;
    } else if (angular_z < -0.05f) { // Right turn -> Right wheels (FR, RR) are inner
      fr_t = rr_t = DRIFT_TRACTION_INNER;
      fl_t = rl_t = DRIFT_TRACTION_OUTER;
    }
  }

  // Calculate skid-steer wheel speeds in m/s
  // V_wheel = (V_linear + side * V_angular * track_width / 2.0) * bias * traction
  float raw_fl = (linear_x - angular_z * (TRACK_WIDTH / 2.0f)) * BIAS_FL * fl_t;
  float raw_rl = (linear_x - angular_z * (TRACK_WIDTH / 2.0f)) * BIAS_RL * rl_t;
  float raw_fr = (linear_x + angular_z * (TRACK_WIDTH / 2.0f)) * BIAS_FR * fr_t;
  float raw_rr = (linear_x + angular_z * (TRACK_WIDTH / 2.0f)) * BIAS_RR * rr_t;

  // Normalization and clipping prevention if any wheel exceeds max speed
  float largest = max(max(abs(raw_fl), abs(raw_fr)), max(abs(raw_rl), abs(raw_rr)));
  largest = max(largest, MAX_WHEEL_SPEED);

  // Convert to scaled range -1000 .. 1000 for the motor channels
  int fl = (int)((raw_fl / largest) * 1000.0f);
  int fr = (int)((raw_fr / largest) * 1000.0f);
  int rl = (int)((raw_rl / largest) * 1000.0f);
  int rr = (int)((raw_rr / largest) * 1000.0f);

  setMotor(FL_CH, FL_DIR_PIN, fl);
  setMotor(FR_CH, FR_DIR_PIN, fr);
  setMotor(RL_CH, RL_DIR_PIN, rl);
  setMotor(RR_CH, RR_DIR_PIN, rr);
}

void loop() {
  // Read Serial commands from the Jetson Nano
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      if (input == "E" || input == "<ESTOP>") {
        serialStopActive = true;
        targetLinearX = targetAngularZ = 0.0f;
      }
      else if (input == "R" || input == "<RESET>") {
        // Resume only if hardware E-Stop button is released (reads LOW)
        if (digitalRead(ESTOP_PIN) == LOW) {
          estopLatched = false;
          serialStopActive = false;
          lastCommandTime = millis();
        }
      }
      else if (input.startsWith("<") && input.endsWith(">")) {
        // Parse framed commands packet: <linear_x_int,angular_z_int,drift_active_int,checksum>
        String content = input.substring(1, input.length() - 1);
        int commas[3];
        int commaCount = 0;
        for (int i = 0; i < content.length(); i++) {
          if (content.charAt(i) == ',') {
            if (commaCount < 3) {
              commas[commaCount++] = i;
            }
          }
        }
        if (commaCount == 3) {
          int linear_x_int = content.substring(0, commas[0]).toInt();
          int angular_z_int = content.substring(commas[0] + 1, commas[1]).toInt();
          int drift_active_int = content.substring(commas[1] + 1, commas[2]).toInt();
          int checksum = content.substring(commas[2] + 1).toInt();

          int computed = (linear_x_int + angular_z_int + drift_active_int) & 0xFF;
          if (computed == checksum) {
            targetLinearX = linear_x_int / 1000.0f;
            targetAngularZ = angular_z_int / 1000.0f;
            targetDriftActive = (drift_active_int > 0);
            lastCommandTime = millis();
          }
        }
      }
    }
  }

  // Local Watchdog / Link lost failsafe
  bool linkLost = (millis() - lastCommandTime) > LINK_TIMEOUT;
  bool wirelessStopped = haveData && incomingData.stopped;
  bool shouldStop = estopLatched || serialStopActive || linkLost || wirelessStopped;

  if (shouldStop) {
    forceAllSafe();
    targetLinearX = targetAngularZ = 0.0f;
    return;
  }

  // Drive Cytron Motor Drivers with mixed speed commands
  mixAndDrive(targetLinearX, targetAngularZ, targetDriftActive);
}
