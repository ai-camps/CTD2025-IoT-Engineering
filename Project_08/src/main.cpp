#include <Arduino.h>

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto LOOP_LED_PIN = 13;  // Loop LED pin
constexpr auto TILT_PIN = 6;       // Tilt sensor pin
constexpr auto SOUND_PIN = 1;     // Sound sensor pin

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT);  // Setup LED
    pinMode(LOOP_LED_PIN, OUTPUT);   // Loop LED
    pinMode(TILT_PIN, INPUT_PULLUP); // Tilt sensor pin
    pinMode(SOUND_PIN, INPUT_PULLUP); // Sound sensor pin

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED

    // Print confirmation message
    Serial.println("Setup completed successfully - LED on GPIO 12 is now ON");

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
    // Main loop - LED remains on
    digitalWrite(LOOP_LED_PIN, HIGH);  // Turn on loop LED
    digitalWrite(SETUP_LED_PIN, LOW);  // Turn off setup LED
    delay(5000);                       // Wait for 5 seconds
    digitalWrite(LOOP_LED_PIN, LOW);   // Turn off loop LED
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED
    delay(5000);                       // Wait for 5 seconds

    // Check if the tilt sensor is triggered
    if (digitalRead(TILT_PIN) == LOW)
    {
        Serial.println("Tilt sensor is triggered");
    }
    else
    {
        Serial.println("Tilt sensor is not triggered");
    }

    // Check if the sound sensor is triggered
    if (analogRead(SOUND_PIN) > 100)
    {
        Serial.println("Sound sensor is triggered");
    }
    else
    {
        Serial.println("Sound sensor is not triggered");
    }
}
