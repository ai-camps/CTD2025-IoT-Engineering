#include <Arduino.h>

constexpr auto BUZZER_PIN = 10;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);
    delay(1000);
}

void loop()
{
    // Sound buzzer for 1 second
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Buzzer Status: ON");
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Buzzer Status: OFF");
    delay(1000);
    Serial.println(USER_EMAIL);
}
