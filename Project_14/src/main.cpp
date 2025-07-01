#include <Arduino.h>

constexpr auto RED_LED_PIN = 4;
constexpr auto GREEN_LED_PIN = 5;
constexpr auto BLUE_LED_PIN = 6;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    delay(1000);

    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(1000);

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);

    delay(1000);
}

void loop()
{
    // Green LED on for 1 second
    digitalWrite(GREEN_LED_PIN, HIGH);
    Serial.print("Green LED Status: ");
    Serial.println(digitalRead(GREEN_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.print("Green LED Status: ");
    Serial.println(digitalRead(GREEN_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);

    // Red LED on for 1 second
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.print("Red LED Status: ");
    Serial.println(digitalRead(RED_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    digitalWrite(RED_LED_PIN, LOW);
    Serial.print("Red LED Status: ");
    Serial.println(digitalRead(RED_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);

    // Blue LED on for 1 second
    digitalWrite(BLUE_LED_PIN, HIGH);
    Serial.print("Blue LED Status: ");
    Serial.println(digitalRead(BLUE_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    digitalWrite(BLUE_LED_PIN, LOW);
    Serial.print("Blue LED Status: ");
    Serial.println(digitalRead(BLUE_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
}
