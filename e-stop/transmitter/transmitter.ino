/*
  TRANSMITTER  -  ESP32 Dev Module
  ---------------------------------
  Reads a KCD1 rocker (latching) switch and mirrors its position as a
  stop/run state. Continuously broadcasts that state (plus an example
  command value) to the receiver over ESP-NOW.

  WIRING:
    KCD1 rocker switch, terminal 1 -> GPIO4
    KCD1 rocker switch, terminal 2 -> GND
    (uses internal pull-up, no external resistor needed)

    Switch ON  (closed to GND) -> GPIO4 reads LOW  -> STOP
    Switch OFF (open)          -> GPIO4 reads HIGH -> RUN

  BEFORE UPLOADING:
    1. Upload receiver.ino to the ESP32-S3 first.
    2. Open its Serial Monitor - it prints its MAC address on boot.
    3. Paste that MAC address into receiverMac[] below (0xXX format).
*/

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// >>> Replace with your ESP32-S3 receiver's MAC address <<<
uint8_t receiverMac[] = {0x74, 0x4D, 0xBD, 0x46, 0x0A, 0x9C};

#define LIMIT_SWITCH_PIN 14
#define DEBOUNCE_MS      50
#define SEND_INTERVAL_MS 100   // how often we broadcast state

typedef struct struct_message {
  bool stopped;       // true = STOP, false = RUN
  int  commandValue;  // example payload - replace with your real command
} struct_message;

struct_message outgoingData;
esp_now_peer_info_t peerInfo;

bool stoppedState   = false;
bool lastRawReading = HIGH;
bool stableReading   = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastSendTime     = 0;

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAILED");
}

void setup() {
  Serial.begin(115200);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Read the switch's initial position so the first broadcast is correct
  stableReading = digitalRead(LIMIT_SWITCH_PIN);
  lastRawReading = stableReading;
  stoppedState = (stableReading == LOW);

  Serial.println("Transmitter ready.");
}

void loop() {
  bool reading = digitalRead(LIMIT_SWITCH_PIN);

  // --- debounce ---
  if (reading != lastRawReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableReading) {
      stableReading = reading;
      // Switch ON (closed to GND) = LOW = STOP
      // Switch OFF (open)        = HIGH = RUN
      stoppedState = (stableReading == LOW);
      Serial.println(stoppedState ? ">> Switch ON: STOP" : ">> Switch OFF: RESUME");
    }
  }
  lastRawReading = reading;

  // --- periodic broadcast ---
  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = millis();

    outgoingData.stopped = stoppedState;
    // Replace this with whatever value you actually want to send when running.
    outgoingData.commandValue = stoppedState ? 0 : 1;

    esp_now_send(receiverMac, (uint8_t *) &outgoingData, sizeof(outgoingData));
  }
}
