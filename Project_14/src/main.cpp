#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto LOOP_LED_PIN = 13;  // Loop LED pin
constexpr auto DHT11_PIN = 8;      // DHT11 pin
constexpr auto GREEN_LED_PIN = 2;  // Green LED pin
constexpr auto RED_LED_PIN = 3;    // Red LED pin
constexpr auto BLUE_LED_PIN = 0;   // Blue LED pin
constexpr auto BUZZER_PIN = 1;     // Buzzer pin

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;         // OLED display width
constexpr auto SCREEN_HEIGHT = 64;         // OLED display height
constexpr auto OLED_RESET = -1;            // Reset pin (or -1 if sharing Arduino reset pin)
constexpr auto SDA_PIN = 4;                // I2C SDA pin
constexpr auto SCL_PIN = 5;                // I2C SCL pin
constexpr auto SSD1306_I2C_ADDRESS = 0x3C; // OLED display address

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT sensor object
DHT dht11(DHT11_PIN, DHT11);

// WiFi settings
constexpr auto WIFI_SSID = "WiFi_TEST";    // replace with your WiFi SSID
constexpr auto WIFI_PASSWORD = "16032390"; // replace with your WiFi password
bool isWiFiConnected = false;

// NTP settings
constexpr auto NTP_SERVER = "pool.ntp.org";
constexpr auto NTP_UPDATE_INTERVAL = 3600000; // 1 hour
constexpr auto NTP_OFFSET = -8 * 3600;        // UTC-8
constexpr auto NTP_INTERVAL = 60000;          // 1 minute

// NTP client objects
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, NTP_UPDATE_INTERVAL);

// Global variables
String currentDate = ""; // Store the formatted date

// Function prototypes
void initOled();    // Initialize OLED display
void connectWiFi(); // Connect to WiFi
String syncTime();  // Synchronize time

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT); // Setup LED
    pinMode(LOOP_LED_PIN, OUTPUT);  // Loop LED
    pinMode(GREEN_LED_PIN, OUTPUT); // Green LED pin
    pinMode(RED_LED_PIN, OUTPUT);   // Red LED pin
    pinMode(BLUE_LED_PIN, OUTPUT);  // Blue LED pin
    pinMode(BUZZER_PIN, OUTPUT);    // Buzzer pin

    // Initialize DHT sensor
    dht11.begin();

    // Initialize OLED display
    initOled();

    // Connect to WiFi
    connectWiFi();

    // Sync time
    currentDate = syncTime();

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED

    // Print confirmation message
    Serial.println("Setup completed successfully - LED on GPIO 12 is now ON");
    Serial.println("DHT11 sensor initialized");

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
    // Main loop - LED remains on
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(1000);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(RED_LED_PIN, LOW);
    delay(1000);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(1000);

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
    Serial.println("=== DHT11 Sensor Readings ===");
    Serial.print("Temperature (Celsius): ");
    Serial.print(temperatureC);
    Serial.println(" °C");
    Serial.print("Temperature (Fahrenheit): ");
    Serial.print(temperatureF);
    Serial.println(" °F");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println("=============================");

    // Print to OLED display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.println("Date: " + currentDate);
    display.println("Time: " + String(timeClient.getFormattedTime()));
    display.println("WiFi: " + String(WiFi.isConnected() == 1 ? "Connected" : "Disconnected"));
    display.println("IP: " + String(WiFi.localIP().toString()));
    display.println("Temp_C: " + String(temperatureC) + " C");
    display.println("Temp_F: " + String(temperatureF) + " F");
    display.println("Humidity: " + String(humidity) + " %");
    display.display();

    // Play buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000);
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

void connectWiFi()
{
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    // Print connection status
    Serial.println("Connected to WiFi");
    isWiFiConnected = true;
}

String syncTime()
{
    // Check if WiFi is connected before attempting NTP sync
    if (!isWiFiConnected)
    {
        Serial.println("WiFi not connected. Cannot sync time.");
        return "WiFi Not Connected";
    }

    // Initialize NTP client
    timeClient.begin();

    // Set time zone
    timeClient.setTimeOffset(NTP_OFFSET);

    // Wait for NTP synchronization
    Serial.println("Waiting for NTP synchronization...");
    timeClient.update();

    // Wait up to 10 seconds for valid time
    int attempts = 0;
    while (timeClient.getEpochTime() < 24 * 3600 && attempts < 10)
    {
        delay(1000);
        timeClient.update();
        attempts++;
        Serial.print("NTP sync attempt: ");
        Serial.println(attempts);
    }

    if (timeClient.getEpochTime() > 24 * 3600)
    {
        Serial.println("NTP synchronization successful!");
        time_t epochTime = timeClient.getEpochTime();
        struct tm *ptm = gmtime(&epochTime);
        return String(ptm->tm_mon + 1) + "/" + String(ptm->tm_mday) + "/" + String(ptm->tm_year + 1900);
    }
    else
    {
        Serial.println("NTP synchronization failed!");
        return "Sync Failed";
    }
}
