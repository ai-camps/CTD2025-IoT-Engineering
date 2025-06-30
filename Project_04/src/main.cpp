#include <Arduino.h>

// LED pin definition
constexpr auto SETUP_LED_PIN = 12;

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT);

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH);

    // Print confirmation message
    Serial.println("Setup completed successfully - LED on GPIO 12 is now ON");

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
    // Main loop - LED remains on
    // Add any additional functionality here as needed
    delay(5000); // Small delay to prevent watchdog issues
}