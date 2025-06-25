#include <Arduino.h>

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto LOOP_LED_PIN = 13;  // Loop LED pin

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT); // Setup LED
    pinMode(LOOP_LED_PIN, OUTPUT);  // Loop LED

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
}
