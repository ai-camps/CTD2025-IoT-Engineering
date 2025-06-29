#include <Arduino.h>          // Include Arduino core library
#include <DHT.h>              // Include DHT library for temperature and humidity sensor
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#include <WiFi.h>             // Include WiFi library for ESP32
#include <NTPClient.h>        // Include NTPClient library for time synchronization
#include <WiFiUdp.h>          // Include WiFiUdp library for UDP communication
#include <time.h>             // Include time library for time-related functions
#include <ArduinoOTA.h>       // Library for OTA updates

// Global device ID
String deviceID;

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto WIFI_LED_PIN = 13;  // Loop LED pin
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
constexpr auto DHT11_UPDATE_INTERVAL = 5000; // 5 seconds
unsigned long lastDht11Read = 0;             // Timer for DHT11 readings
bool isTemperatureTooHigh = false;           // Flag to check if temperature is too high
bool isTemperatureTooLow = false;            // Flag to check if temperature is too low
bool isTemperatureNormal = false;            // Flag to check if temperature is normal
constexpr auto TEMP_C_HIGH_THRESHOLD = 30;   // Temperature too high threshold
constexpr auto TEMP_C_LOW_THRESHOLD = 20;    // Temperature too low threshold

// WiFi settings
constexpr auto WIFI_SSID = "sesplearningstudios"; // CTD WiFi
constexpr auto WIFI_PASSWORD = "@nn3nb3rg";       // CTD WiFi password
bool isWiFiConnected = false;

// NTP settings
constexpr auto NTP_SERVER = "pool.ntp.org";
constexpr auto NTP_UPDATE_INTERVAL = 3600000; // 1 hour
constexpr auto NTP_OFFSET = -8 * 3600;        // UTC-8
constexpr auto NTP_INTERVAL = 60000;          // 1 minute

// NTP client objects
WiFiUDP ntpUDP;                                                            // WiFiUDP object
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, NTP_UPDATE_INTERVAL); // NTPClient object

// Global variables
String currentDate = ""; // Store the formatted date

// OTA settings
constexpr auto OTA_PORT = 3232;               // OTA port
constexpr auto OTA_HOSTNAME = "a011a98481b0"; // OTA hostname by device ID
bool isOtaInProgress = false;                 // Flag to track OTA status

// LED flags
bool isGreenLEDOn = false; // Green LED flag
bool isRedLEDOn = false;   // Red LED flag
bool isBlueLEDOn = false;  // Blue LED flag

// Function prototypes
void initOled();    // Initialize OLED display
void connectWiFi(); // Connect to WiFi
String syncTime();  // Synchronize time
void initOTA();     // Initialize OTA updates
void readDht11();   // Read DHT11 sensor data
void setLED();      // Turn on/off Green, Red, Blue LED

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Generate device ID
    deviceID = String(ESP.getEfuseMac(), HEX);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT); // Setup LED
    pinMode(WIFI_LED_PIN, OUTPUT);  // Loop LED
    pinMode(GREEN_LED_PIN, OUTPUT); // Green LED pin
    pinMode(RED_LED_PIN, OUTPUT);   // Red LED pin
    pinMode(BLUE_LED_PIN, OUTPUT);  // Blue LED pin
    pinMode(BUZZER_PIN, OUTPUT);    // Buzzer pin

    // Initialize all LEDs to OFF state
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);

    // Initialize LED flags
    isGreenLEDOn = false;
    isRedLEDOn = false;
    isBlueLEDOn = false;

    // Initialize DHT sensor
    dht11.begin();

    // Initialize OLED display
    initOled();

    // Connect to WiFi
    connectWiFi();
    digitalWrite(WIFI_LED_PIN, HIGH ? isWiFiConnected : LOW); // Set WiFi LED to HIGH if connected, LOW if not connected

    // Sync time
    currentDate = syncTime();

    // Initialize OTA
    initOTA();

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED

    // Print confirmation message
    Serial.println("Setup completed successfully");
}

void loop()
{
    // Handle OTA updates
    ArduinoOTA.handle();

    // Check if it's time to read DHT11 sensor (non-blocking)
    if (millis() - lastDht11Read >= DHT11_UPDATE_INTERVAL)
    {
        // Read DHT11 sensor data
        readDht11();

        // Set LED status based on temperature
        setLED();

        // Update the timer
        lastDht11Read = millis();
    }

    // Update time client periodically
    timeClient.update();
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

// Function to initialize OTA
void initOTA()
{
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        isOtaInProgress = true; // Set OTA flag
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Start OTA " + type);
        display.display(); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        // Calculate progress percentage
        int progressPercent = (progress * 100) / total;

        // Clear the row where progress will be displayed
        display.fillRect(0, 10, SCREEN_WIDTH, 8, SSD1306_BLACK); // Clear the row at y=10

        // Set cursor and display progress
        display.setCursor(0, 10);
        display.println("Progress: " + String(progressPercent) + "%");
        display.display(); });

    ArduinoOTA.onEnd([]()
                     {
                         display.println("\nOTA End");
                         display.display();
                         delay(2000);             // Show completion message for 2 seconds
                         isOtaInProgress = false; // Clear OTA flag
                     });

    ArduinoOTA.onError([](ota_error_t error)
                       {
                           String errorMsg = "Error[" + String(error) + "]: ";
                           if (error == OTA_AUTH_ERROR)
                           {
                               errorMsg += "Auth Failed";
                           }
                           else if (error == OTA_BEGIN_ERROR)
                           {
                               errorMsg += "Begin Failed";
                           }
                           else if (error == OTA_CONNECT_ERROR)
                           {
                               errorMsg += "Connect Failed";
                           }
                           else if (error == OTA_RECEIVE_ERROR)
                           {
                               errorMsg += "Receive Failed";
                           }
                           else if (error == OTA_END_ERROR)
                           {
                               errorMsg += "End Failed";
                           }
                           display.println(errorMsg);
                           display.display();
                           delay(3000);             // Show error message for 3 seconds
                           isOtaInProgress = false; // Clear OTA flag
                       });

    // Start OTA
    ArduinoOTA.begin();
}

// Function to read DHT11 sensor data
void readDht11()
{
    // Read DHT11 sensor data
    float humidity = dht11.readHumidity();            // Read humidity
    float temperatureC = dht11.readTemperature();     // Read temperature in Celsius
    float temperatureF = dht11.readTemperature(true); // Read temperature in Fahrenheit

    // Check if readings are valid
    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF))
    {
        Serial.println("Failed to read from DHT11 sensor!");
    }
    else
    {
        // Check temperature thresholds and set flags
        if (temperatureC >= TEMP_C_LOW_THRESHOLD && temperatureC <= TEMP_C_HIGH_THRESHOLD)
        {
            isTemperatureNormal = true;
            isTemperatureTooLow = false;
            isTemperatureTooHigh = false;
        }
        else if (temperatureC < TEMP_C_LOW_THRESHOLD)
        {
            isTemperatureNormal = false;
            isTemperatureTooLow = true;
            isTemperatureTooHigh = false;
        }
        else if (temperatureC > TEMP_C_HIGH_THRESHOLD)
        {
            isTemperatureNormal = false;
            isTemperatureTooLow = false;
            isTemperatureTooHigh = true;
        }

        // Only update OLED display if OTA is not in progress
        if (!isOtaInProgress)
        {
            // Update OLED display with sensor data
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("DeviceID:" + deviceID);
            display.println("Date: " + currentDate);
            display.println("Time: " + String(timeClient.getFormattedTime()));
            display.println("WiFi: " + String(WiFi.isConnected() == 1 ? "Connected" : "Disconnected"));
            display.println("IP: " + String(WiFi.localIP().toString()));
            display.println("Temp_C: " + String(temperatureC) + " C");
            display.println("Temp_F: " + String(temperatureF) + " F");
            display.println("Humidity: " + String(humidity) + " %");
            display.display();
        }
    }
}

// Function to set LED status based on temperature
void setLED()
{
    // Set LED based on temperature status
    if (isTemperatureNormal)
    {
        digitalWrite(GREEN_LED_PIN, HIGH); // Green LED for normal temperature
        digitalWrite(RED_LED_PIN, LOW);    // Turn off other LEDs
        digitalWrite(BLUE_LED_PIN, LOW);
        isGreenLEDOn = true;
        isRedLEDOn = false;
        isBlueLEDOn = false;
        Serial.println("Temperature is normal - Green LED ON");
    }
    else if (isTemperatureTooLow)
    {
        digitalWrite(GREEN_LED_PIN, LOW); // Turn off other LEDs
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, HIGH); // Blue LED for low temperature
        isGreenLEDOn = false;
        isRedLEDOn = false;
        isBlueLEDOn = true;
        Serial.println("Temperature is too low - Blue LED ON");
    }
    else if (isTemperatureTooHigh)
    {
        digitalWrite(GREEN_LED_PIN, LOW); // Turn off other LEDs
        digitalWrite(RED_LED_PIN, HIGH);  // Red LED for high temperature
        digitalWrite(BLUE_LED_PIN, LOW);
        isGreenLEDOn = false;
        isRedLEDOn = true;
        isBlueLEDOn = false;
        Serial.println("Temperature is too high - Red LED ON");
    }
}