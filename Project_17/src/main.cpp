#include <Arduino.h>          // Include Arduino core library
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display

// Global device ID
String deviceID;

// Global user email
constexpr auto USER_EMAIL = "your.email@example.com";

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;         // OLED display width
constexpr auto SCREEN_HEIGHT = 64;         // OLED display height
constexpr auto OLED_RESET = -1;            // Reset pin (or -1 if sharing Arduino reset pin)
constexpr auto SDA_PIN = 5;                // I2C SDA pin
constexpr auto SCL_PIN = 4;                // I2C SCL pin
constexpr auto SSD1306_I2C_ADDRESS = 0x3C; // OLED display address

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Function prototypes
void initOled();

void setup()
{

    // Generate device ID
    deviceID = String(ESP.getEfuseMac(), HEX);

    Serial.print("Device ID: ");
    Serial.println(deviceID);
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    Serial.println("Hello World!");

    // Initialize OLED display
    initOled();
}

void loop()
{
    Serial.print("Device ID: ");
    Serial.println(deviceID);
    Serial.print("User Email: ");
    Serial.println(USER_EMAIL);
    delay(5000);

    // Display deviceID on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("DeviceID:" + deviceID);
    display.println("Hello World!");
    display.println(USER_EMAIL);
    display.display();
    delay(5000);
}

// Function definitions
void initOled()
{
    // Initialize I2C with custom pins
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize OLED display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    // Clear the display buffer
    display.clearDisplay();

    // Display startup message
    display.setTextSize(1);              // Set text size
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0, 0);             // Set cursor position
    display.println("Initializing...");  // Display startup message
    display.display();                   // Display the message
}