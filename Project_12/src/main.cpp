#include <Arduino.h>

constexpr auto BUTTON_PIN = 20;
constexpr auto USER_EMAIL = "your.email@example.com";

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, OUTPUT);
}

void loop()
{
    int buttonStatus = analogRead(BUTTON_PIN);
    Serial.println("=== Button Pressed Information ===");
    Serial.print("Button Status: ");
    Serial.println(buttonStatus == LOW ? "Button Pressed !!!" : "Button Not Pressed");
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    Serial.println("================================");
    delay(1000);
}
