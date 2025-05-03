#include <Arduino.h>

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
  delay(1000); // Wait for 1 second before printing again
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}