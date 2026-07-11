/*
  RECEIVER  -  ESP32-S3
  ----------------------
  Listens for state packets from the transmitter over ESP-NOW, and
  reads serial drive/estop/resume commands from the host (Jetson Nano)
  to drive a Cytron motor driver.

  - RUN state  -> Drive Cytron motor driver pins based on serial inputs.
  - STOP state -> Every pin in OUTPUT_PINS (including PWM/DIR) is immediately
                  forced to its SAFE_STATE value, and the motors halt.
  - Fail-safe  -> If no wireless packet arrives for LINK_TIMEOUT ms (link lost),
                  or if a software E-stop is triggered via Serial, the system
                  enters STOP mode and stays safe.

  Cytron Pins:
    - Left Motor:  PWM = GPIO11, DIR = GPIO12
    - Right Motor: PWM = GPIO13, DIR = GPIO14
*/

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#define LINK_TIMEOUT  500   // ms - no packet within this window = assume stopped

// ---- Cytron Pins ----
#define L_PWM_PIN 11
#define L_DIR_PIN 12
#define R_PWM_PIN 13
#define R_DIR_PIN 14

// ---- All outputs controlled by this receiver ----
const uint8_t OUTPUT_PINS[] = {
  44, 43, L_PWM_PIN, L_DIR_PIN, 1, 2, R_PWM_PIN, R_DIR_PIN, 4, 14, 5, 6, 7, 8, 15, 16, 21,
  38, 39, 18, 17, 40, 9, 45, 35
};
const uint8_t NUM_OUTPUTS = sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);

// ---- Safe/OFF level for each pin ----
const uint8_t SAFE_STATE[] = {
  LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW,
  LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW
};

typedef struct struct_message {
  bool stopped;
  int  commandValue;
} struct_message;

struct_message incomingData;
volatile unsigned long lastReceiveTime = 0;
volatile bool haveData = false;

// Software E-stop state triggered by Jetson Nano over Serial
bool serialStopActive = false;

// Current motor speeds (PWM values: -255 to 255)
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

unsigned long lastCommandTime = 0;

// Forces every controlled pin to its safe state in one shot.
void forceAllSafe() {
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    digitalWrite(OUTPUT_PINS[i], SAFE_STATE[i]);
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  lastReceiveTime = millis();
  haveData = true;
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(OUTPUT_PINS[i], OUTPUT);
  }
  forceAllSafe();   // safe until we hear from transmitter and Jetson

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

void loop() {
  // Read Serial commands from the Jetson Nano
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      if (input.startsWith("E")) {
        serialStopActive = true;
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
      }
      else if (input.startsWith("R")) {
        serialStopActive = false;
        lastCommandTime = millis();
      }
      else if (input.startsWith("D,")) {
        int firstComma = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
          leftMotorSpeed = input.substring(firstComma + 1, secondComma).toInt();
          rightMotorSpeed = input.substring(secondComma + 1).toInt();
          lastCommandTime = millis();
        }
      }
    }
  }

  // Serial link timeout failsafe
  if (millis() - lastCommandTime > LINK_TIMEOUT) {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
  }

  bool linkLost   = (millis() - lastReceiveTime) > LINK_TIMEOUT;
  bool shouldStop = (!haveData) || linkLost || incomingData.stopped || serialStopActive;

  if (shouldStop) {
    forceAllSafe();
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    return;
  }

  // --- Normal Operation: Drive Cytron Motor Driver ---

  if (leftMotorSpeed >= 0) {
    digitalWrite(L_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, leftMotorSpeed);
  } else {
    digitalWrite(L_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN, abs(leftMotorSpeed));
  }

  if (rightMotorSpeed >= 0) {
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(R_PWM_PIN, rightMotorSpeed);
  } else {
    digitalWrite(R_DIR_PIN, LOW);
    analogWrite(R_PWM_PIN, abs(rightMotorSpeed));
  }
}
