#include <Arduino.h>

constexpr auto COLLISION_SENSOR_PIN = 21;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(COLLISION_SENSOR_PIN, INPUT);
}

void loop()
{
    int pirStatus = digitalRead(COLLISION_SENSOR_PIN);
    Serial.println("=== Collision Detection Information ===");
    Serial.print("Collision Sensor Status: ");
    Serial.println(pirStatus == LOW ? "Collision Detected !!!" : "No Collision Detected");
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    Serial.println("================================");
    delay(1000);
}
