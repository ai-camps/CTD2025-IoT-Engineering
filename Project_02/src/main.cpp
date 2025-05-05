#include <Arduino.h>

// Constants
const unsigned long PAUSE_TIME = 1000; // Delay time in milliseconds

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("Hello World !");
  delay(PAUSE_TIME); // Wait for specified time before printing again
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}