#include <Arduino.h>
#include <WiFi.h>
#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#include <WiFiManager.h>      // Include WiFiManager library for WiFi configuration
#include <ThingsBoard.h>      // Include ThingsBoard library for cloud communication

// ====
//.....
// ====

//======Cloud Access Settings======
// ThingsBoard settings
WiFiClient espClient;

// Set ThingsBoard server, port, access token, client ID, and retry delay
constexpr auto thingsboardServer = "dashboard.ai-camps.com";
constexpr auto thingsboardPort = 1884;
constexpr auto accessToken = "j5jbbcwi1fgffl1aqxc8"; // change to your own device access token when provided by the cloud providerin Thingsboard

// Define the desired MaxFieldsAmt (max number of fields in the JSON document)
constexpr unsigned int MAX_FIELDS_AMT = 64;

// Initialize ThingsBoard instance
ThingsBoardSized<MAX_FIELDS_AMT> tb(espClient);
//======Cloud Access Settings======

// Function declarations
// ====
//.....
// ====

void connectCloud();

void setup()
{
  // ====
  //.....
  // ====

  connectCloud(); // Maintain ThingsBoard connection
}

// ====
//.....
// ====

void connectCloud()
{
  if (!tb.connected())
  {
    Serial.print("Connecting to ThingsBoard at ");
    Serial.print(thingsboardServer);
    Serial.print(":");
    Serial.println(thingsboardPort);
    if (!tb.connect(thingsboardServer, accessToken, thingsboardPort))
    {
      Serial.println("Failed to connect to ThingsBoard");
      return;
    }
    else
    {
      Serial.println("Connected to ThingsBoard!");
    }
  }
}
