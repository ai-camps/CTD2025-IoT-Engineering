#include <Arduino.h>

constexpr auto FAN_PIN = 11;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(FAN_PIN, OUTPUT);
    delay(1000);
}

void loop()
{
    // Sound buzzer for 1 second
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("Fan Status: ON");
    delay(5000);
    digitalWrite(FAN_PIN, LOW);
    Serial.println("Fan Status: OFF");
    delay(5000);
    Serial.println(USER_EMAIL);
}
