#include <Arduino.h>

void setup()
{
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait for serial port to connect (needed for some boards)
  while (!Serial)
  {
    ; // Wait for serial port to connect
  }

  // Print Hello World message
  Serial.println("Hello World!");
}

void loop()
{
  // Print Hello World every 2 seconds
  Serial.println("Hello World!");
  delay(3000);
}
