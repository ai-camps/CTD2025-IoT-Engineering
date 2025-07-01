#include <Arduino.h>

constexpr auto USER_EMAIL = "your.email@example.com";

// Global device info
String deviceID;
String chipModel;

void setup()
{
    Serial.begin(115200);
    delay(1000); // Wait for serial to initialize

    // Get device ID (MAC address) and chipset model once
    deviceID = String(ESP.getEfuseMac(), HEX);
    chipModel = ESP.getChipModel();
}

void loop()
{
    Serial.println("=== Device Information ===");
    Serial.print("Device ID: ");
    Serial.println(deviceID);
    Serial.print("Chipset Model: ");
    Serial.println(chipModel);
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    Serial.println("=========================");
    delay(5000);
}
