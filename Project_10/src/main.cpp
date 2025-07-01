#include <Arduino.h>

constexpr auto PIR_PIN = 17;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(PIR_PIN, INPUT);
}

void loop()
{
    int pirStatus = digitalRead(PIR_PIN);
    Serial.println("=== Motion Detection Information ===");
    Serial.print("PIR Sensor Status: ");
    Serial.println(pirStatus == HIGH ? "Motion Detected !!!" : "No Motion Detected");
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    Serial.println("================================");
    delay(1000);
}
