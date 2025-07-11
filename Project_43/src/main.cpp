#include <Arduino.h>
#include <WiFi.h>
#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#include <WiFiManager.h>      // Include WiFiManager library for WiFi configuration
#include <ThingsBoard.h>      // Include ThingsBoard library for data transmission
#include <ArduinoJson.h>      // Include ArduinoJson library for JSON parsing

// ====
//.....
// ====
void sendDataToCloud();

void loop()
{
  unsigned long now = millis();
  if (now - lastOledUpdate >= oledUpdateInterval)
  {
    lastOledUpdate = now;
    readDHT11(); // Only read sensor every 10 seconds
    sendDataToCloud();
  }
  updateOLEDInfo(); // Update OLED as often as possible for responsiveness
}

// ====
//.....
// ====

void sendDataToCloud()
{
  // Set the buffer size by bytes for the JSON document
  const size_t jsonBufferSize = 256;
  StaticJsonDocument<256> jsonDoc;

  // Set the buffer size for the JSON document (if supported by tb)
  tb.setBufferSize(jsonBufferSize);

  // Clear the JSON document
  jsonDoc.clear();

  // Fill in the requested fields
  jsonDoc["deviceID"] = deviceID;
  jsonDoc["Owner"] = "Jun";
  jsonDoc["IP"] = WiFi.localIP().toString();
  jsonDoc["tempF"] = lastTempF;
  jsonDoc["tempC"] = lastTempC;
  jsonDoc["humidity"] = lastHumidity;
  jsonDoc["tilt"] = tiltStatus ? "On" : "Off";
  jsonDoc["collision"] = collisionStatus ? "On" : "Off";
  jsonDoc["pir"] = pirStatus ? "On" : "Off";
  jsonDoc["fan"] = digitalRead(FAN_PIN) == HIGH ? "On" : "Off";

  // Serialize the JSON document into a string
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  Serial.println("Publishing message: ");
  Serial.println(jsonString);

  // Send the JSON document directly
  if (!tb.sendTelemetryJson(jsonString.c_str()))
  {
    Serial.println("Failed to send telemetry");
  }
  else
  {
    Serial.println("Telemetry sent successfully");
  }

  // Clear the JSON document
  jsonDoc.clear();
  // Clear the JSON string
  jsonString = "";
}
