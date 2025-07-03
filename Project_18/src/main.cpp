#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display

// LED pin definition
constexpr auto DHT11_PIN = 18; // DHT11 pin

// Global device ID
String deviceID;

// Global user email
constexpr auto USER_EMAIL = "your.email@example.com";

// DHT sensor object
DHT dht11(DHT11_PIN, DHT11);

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;         // OLED display width
constexpr auto SCREEN_HEIGHT = 64;         // OLED display height
constexpr auto OLED_RESET = -1;            // Reset pin (or -1 if sharing Arduino reset pin)
constexpr auto SDA_PIN = 14;               // I2C SDA pin
constexpr auto SCL_PIN = 15;               // I2C SCL pin
constexpr auto SSD1306_I2C_ADDRESS = 0x3D; // OLED display address

// OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Function declarations
void initOled();

void setup()
{

    // Generate device ID
    deviceID = String(ESP.getEfuseMac(), HEX);

    // Configure LED pin as output
    pinMode(DHT11_PIN, OUTPUT); // Setup LED

    // Initialize DHT sensor
    dht11.begin();

    // Initialize OLED display
    initOled();

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
    // Read DHT11 sensor data
    float humidity = dht11.readHumidity();            // Read humidity
    float temperatureC = dht11.readTemperature();     // Read temperature in Celsius
    float temperatureF = dht11.readTemperature(true); // Read temperature in Fahrenheit

    // Check if readings are valid
    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF))
    {
        Serial.println("Failed to read from DHT11 sensor!");
        return;
    }

    // Print sensor readings
    Serial.println("========= DHT11 Sensor Readings =========");
    Serial.print("Temperature (Celsius): ");
    Serial.print(temperatureC);
    Serial.println(" °C");
    Serial.print("Temperature (Fahrenheit): ");
    Serial.print(temperatureF);
    Serial.println(" °F");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println(USER_EMAIL);
    Serial.println("=========================================");

    // --- OLED Display Update ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0); // Row 1: deviceID
    display.print("DeviceID:");
    display.println(deviceID);
    display.setCursor(0, 16); // Row 2: temperatureF
    display.print("Temp_F: ");
    display.print(temperatureF, 1);
    display.println(" F");
    display.setCursor(0, 32); // Row 3: temperatureC
    display.print("Temp_C: ");
    display.print(temperatureC, 1);
    display.println(" C");
    display.setCursor(0, 48); // Row 4: humidity
    display.print("Humidity: ");
    display.print(humidity, 1);
    display.println(" %");
    display.display();
    // --- End OLED Display Update ---

    // Add a delay before the next reading
    delay(3000);
}

// Function definitions
void initOled()
{

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