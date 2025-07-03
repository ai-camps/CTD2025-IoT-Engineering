#include <Arduino.h>

constexpr auto RED_LED_PIN = 14;
constexpr auto GREEN_LED_PIN = 15;
constexpr auto BLUE_LED_PIN = 16;
constexpr auto BUZZER_PIN = 17;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    pinMode(RED_LED_PIN, INPUT);
    pinMode(GREEN_LED_PIN, INPUT);
    pinMode(BLUE_LED_PIN, INPUT);
    delay(1000);

    analogWrite(RED_LED_PIN, 255);
    analogWrite(GREEN_LED_PIN, 255);
    analogWrite(BLUE_LED_PIN, 255);
    delay(1000);

    analogWrite(RED_LED_PIN, 0);
    analogWrite(GREEN_LED_PIN, 0);
    analogWrite(BLUE_LED_PIN, 0);

    delay(1000);
}

void loop()
{
    // Green LED on for 1 second
    analogWrite(GREEN_LED_PIN, 255);
    Serial.print("Green LED Status: ");
    Serial.println(analogRead(GREEN_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    analogWrite(GREEN_LED_PIN, 0);
    Serial.print("Green LED Status: ");
    Serial.println(analogRead(GREEN_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);

    // Red LED on for 1 second
    analogWrite(RED_LED_PIN, 255);
    Serial.print("Red LED Status: ");
    Serial.println(analogRead(RED_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    analogWrite(RED_LED_PIN, 0);
    Serial.print("Red LED Status: ");
    Serial.println(analogRead(RED_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);

    // Blue LED on for 1 second
    analogWrite(BLUE_LED_PIN, 255);
    Serial.print("Blue LED Status: ");
    Serial.println(analogRead(BLUE_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);
    analogWrite(BLUE_LED_PIN, 0);
    Serial.print("Blue LED Status: ");
    Serial.println(analogRead(BLUE_LED_PIN) == HIGH ? "ON" : "OFF");
    delay(1000);

}
