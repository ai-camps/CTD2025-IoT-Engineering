#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>

#define SETUP_DONE_PIN 12
#define WIFI_CONNECTED_PIN 13

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(SETUP_DONE_PIN, OUTPUT);
  pinMode(WIFI_CONNECTED_PIN, OUTPUT);
  digitalWrite(SETUP_DONE_PIN, LOW);
  digitalWrite(WIFI_CONNECTED_PIN, LOW);

  WiFiManager wm;

  wm.autoConnect("ESP32JUN");

  Serial.println("Connected!");

  // Indicate setup() is completed
  digitalWrite(SETUP_DONE_PIN, HIGH);

  // Indicate WiFi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(WIFI_CONNECTED_PIN, HIGH);
  }
}

void loop() {}