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
#include <esp_now.h>          // Include ESP-NOW library

// Global device ID
String deviceID;

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto WIFI_LED_PIN = 13;  // Loop LED pin
constexpr auto DHT11_PIN = 8;      // DHT11 pin
constexpr auto GREEN_LED_PIN = 2;  // Green LED pin
constexpr auto RED_LED_PIN = 3;    // Red LED pin
constexpr auto BLUE_LED_PIN = 0;   // Blue LED pin
constexpr auto FAN_PIN = 1;        // Fan pin
constexpr auto BUTTON_PIN = 10;    // Button pin
constexpr auto TILT_PIN = 6;       // Tilt sensor pin
constexpr auto PIR_PIN = 7;        // Buzzer pin

// PWM settings for red LED
constexpr auto RED_LED_PWM_FREQUENCY = 5000; // PWM frequency in Hz
constexpr auto RED_LED_PWM_RESOLUTION = 8;   // PWM resolution (8-bit = 0-255)
constexpr auto RED_LED_PWM_CHANNEL = 0;      // PWM channel for red LED
constexpr auto RED_LED_BLINK_INTERVAL = 500; // Blink interval in milliseconds
constexpr auto RED_LED_PWM_DUTY_CYCLE = 128; // PWM duty cycle for blinking (50% brightness)

// PWM settings for fan control
constexpr auto FAN_PWM_CHANNEL = 1;       // PWM channel for fan
constexpr auto FAN_PWM_FREQUENCY = 25000; // Fan PWM frequency (25kHz for quiet operation)
constexpr auto FAN_SPEED_NORMAL = 102;    // 40% of 255 for normal temperature

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
constexpr auto TEMP_C_HIGH_THRESHOLD = 25;   // Temperature too high threshold
constexpr auto TEMP_C_LOW_THRESHOLD = 18;    // Temperature too low threshold

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

// Time variables
String currentDate = ""; // Store the formatted date

// OTA settings
constexpr auto OTA_PORT = 3232;               // OTA port
constexpr auto OTA_HOSTNAME = "a011a98481b0"; // OTA hostname by device ID
bool isOtaInProgress = false;                 // Flag to track OTA status

// LED flags
bool isGreenLEDOn = false; // Green LED flag
bool isRedLEDOn = false;   // Red LED flag
bool isBlueLEDOn = false;  // Blue LED flag

// Fan control variable
int fanSpeed = 0; // Current fan speed (0-255)

// PWM blinking variables
unsigned long lastBlinkTime = 0; // Timer for red LED blinking
bool isRedLEDBlinking = false;   // Flag to track red LED blinking state

// ESP-NOW variables
uint8_t localMacAddress[] = {0xB0, 0x81, 0x84, 0xA9, 0x11, 0x11};  // the MAC address of the slave device (remote one) 34:B7:DA:86:F4:CC
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA8, 0xFB, 0x98}; // the MAC address of the master device (present one) B0:81:84:A8:FB:98

// Define a constant for the default WiFi channel for ESP-NOW communication
constexpr auto ESPNOW_COMMUNICATION_MODE = 0; // 0: Through Same WiFi Network, 1: Point-to-Point

// Define ESP-NOW command types
enum ESPNOW_COMMAND_TYPE : uint8_t
{
    COMMAND = 0,
    COMMAND_ACK = 1,
    STATUS_QUERY = 2,
    STATUS_QUERY_RESPONSE = 3,
    TILT_STATUS_ON = 4,
    TILT_STATUS_OFF = 5
};

// Define a struct to hold the message data
struct ESPNOW_COMMAND
{
    uint8_t commandType;
    bool fanState;
};

// ESP-NOW state variables
bool fanState = false; // Current fan state

// Function prototypes
void initOled();                                                                            // Initialize OLED display
void connectWiFi();                                                                         // Connect to WiFi
String syncTime();                                                                          // Synchronize time
void initOTA();                                                                             // Initialize OTA updates
void readDht11();                                                                           // Read DHT11 sensor data
void setLED();                                                                              // Turn on/off Green, Red, Blue LED
void updateRedLEDBlink();                                                                   // Update red LED blinking state
void IRAM_ATTR triggerTilt();                                                               // Tilt sensor interrupt service routine
void IRAM_ATTR triggerPIR();                                                                // PIR sensor interrupt service routine
void initESPNow();                                                                          // Initialize ESP-NOW
void handleEspNowRequestReceived(const uint8_t *mac, const uint8_t *incomingData, int len); // ESP-NOW data receive callback
void handleEspNowSendStatus(const uint8_t *mac_addr, esp_now_send_status_t status);         // ESP-NOW data send callback
void espNowControlFan(bool state);                                                          // Control fan via ESP-NOW command

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Generate device ID
    deviceID = String(ESP.getEfuseMac(), HEX);

    // Print MAC address for debugging
    Serial.print("Slave Device MAC Address: ");
    uint8_t mac[6];
    WiFi.macAddress(mac);
    for (int i = 0; i < 6; i++)
    {
        Serial.print(mac[i], HEX);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    Serial.print("Configured Local MAC: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(localMacAddress[i], HEX);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    Serial.print("Configured Remote MAC: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(remoteMacAddress[i], HEX);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT); // Setup LED
    pinMode(WIFI_LED_PIN, OUTPUT);  // Loop LED
    pinMode(GREEN_LED_PIN, OUTPUT); // Green LED pin
    pinMode(RED_LED_PIN, OUTPUT);   // Red LED pin
    pinMode(BLUE_LED_PIN, OUTPUT);  // Blue LED pin
    pinMode(FAN_PIN, OUTPUT);       // Fan pin

    // Configure tilt sensor pin and attach interrupt
    pinMode(TILT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TILT_PIN), triggerTilt, CHANGE);

    // Configure PIR sensor pin and attach interrupt
    pinMode(PIR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIR_PIN), triggerPIR, CHANGE);

    // Configure PWM for red LED
    ledcSetup(RED_LED_PWM_CHANNEL, RED_LED_PWM_FREQUENCY, RED_LED_PWM_RESOLUTION);
    ledcAttachPin(RED_LED_PIN, RED_LED_PWM_CHANNEL);

    // Configure PWM for fan control
    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQUENCY, RED_LED_PWM_RESOLUTION);
    ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);

    // Initialize all LEDs to OFF state
    digitalWrite(GREEN_LED_PIN, LOW);
    ledcWrite(RED_LED_PWM_CHANNEL, 0); // Turn off red LED via PWM
    digitalWrite(BLUE_LED_PIN, LOW);

    // Initialize fan to OFF state
    ledcWrite(FAN_PWM_CHANNEL, 0);
    fanSpeed = 0;

    // Initialize LED flags
    isGreenLEDOn = false;
    isRedLEDOn = false;
    isBlueLEDOn = false;
    isRedLEDBlinking = false;

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

    // Initialize ESP-NOW service for communication between ESP32 devices
    initESPNow();

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED

    // Print confirmation message
    Serial.println("Setup completed successfully");
}

void loop()
{
    // Handle OTA updates
    ArduinoOTA.handle();

    // Update red LED blinking if temperature is too high
    if (isTemperatureTooHigh)
    {
        updateRedLEDBlink();
    }

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

// Function to initialize ESP-NOW
void initESPNow()
{
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Failed to initialize ESP-NOW");
        return;
    }

    // Register send and receive callbacks
    esp_now_register_recv_cb(handleEspNowRequestReceived); // Register the receive callback
    esp_now_register_send_cb(handleEspNowSendStatus);      // Register the send callback

    // Add peer to ESP-NOW network
    esp_now_peer_info_t peerInfo = {};               // Initialize peer info structure
    memcpy(peerInfo.peer_addr, remoteMacAddress, 6); // Copy master MAC address
    peerInfo.channel = ESPNOW_COMMUNICATION_MODE;    // Use default channel
    peerInfo.encrypt = false;                        // No encryption

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    else
    {
        Serial.println("Peer added successfully.");
    }
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
            display.println("Fan: " + String(fanSpeed) + "/255 (" + String((fanSpeed * 100) / 255) + "%)");
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
        ledcWrite(RED_LED_PWM_CHANNEL, 0); // Turn off red LED via PWM
        digitalWrite(BLUE_LED_PIN, LOW);
        isGreenLEDOn = true;
        isRedLEDOn = false;
        isBlueLEDOn = false;
        isRedLEDBlinking = false; // Stop blinking
        Serial.println("Temperature is normal - Green LED ON");
    }
    else if (isTemperatureTooLow)
    {
        digitalWrite(GREEN_LED_PIN, LOW);  // Turn off other LEDs
        ledcWrite(RED_LED_PWM_CHANNEL, 0); // Turn off red LED via PWM
        digitalWrite(BLUE_LED_PIN, HIGH);  // Blue LED for low temperature
        isGreenLEDOn = false;
        isRedLEDOn = false;
        isBlueLEDOn = true;
        isRedLEDBlinking = false; // Stop blinking
        Serial.println("Temperature is too low - Blue LED ON");
    }
    else if (isTemperatureTooHigh)
    {
        digitalWrite(GREEN_LED_PIN, LOW); // Turn off other LEDs
        digitalWrite(BLUE_LED_PIN, LOW);
        isGreenLEDOn = false;
        isRedLEDOn = true;
        isBlueLEDOn = false;
        isRedLEDBlinking = true; // Start blinking
        Serial.println("Temperature is too high - Red LED blinking with PWM");
    }
}

// Function to update red LED blinking state
void updateRedLEDBlink()
{
    if (isRedLEDBlinking && millis() - lastBlinkTime >= RED_LED_BLINK_INTERVAL)
    {
        // Toggle red LED state
        if (isRedLEDOn)
        {
            ledcWrite(RED_LED_PWM_CHANNEL, RED_LED_PWM_DUTY_CYCLE); // Turn on red LED with PWM
        }
        else
        {
            ledcWrite(RED_LED_PWM_CHANNEL, 0); // Turn off red LED
        }

        isRedLEDOn = !isRedLEDOn; // Toggle the flag
        lastBlinkTime = millis(); // Update the timer
    }
}

// Tilt sensor interrupt service routine
void IRAM_ATTR triggerTilt()
{
    // Read the current state of the tilt sensor
    bool tiltState = digitalRead(TILT_PIN);

    // Control red LED based on tilt state (local indication)
    if (tiltState == LOW)
    {
        // Tilt sensor is triggered (tilted) - turn on red LED
        ledcWrite(RED_LED_PWM_CHANNEL, 255); // Full brightness
        // Send TILT_STATUS_ON message to master
        ESPNOW_COMMAND tiltMsg;
        tiltMsg.commandType = TILT_STATUS_ON;
        tiltMsg.fanState = false; // Not used for tilt
        esp_now_send(remoteMacAddress, (uint8_t *)&tiltMsg, sizeof(tiltMsg));
    }
    else
    {
        // Tilt sensor is not triggered (not tilted) - turn off red LED
        ledcWrite(RED_LED_PWM_CHANNEL, 0);
        // Send TILT_STATUS_OFF message to master
        ESPNOW_COMMAND tiltMsg;
        tiltMsg.commandType = TILT_STATUS_OFF;
        tiltMsg.fanState = false; // Not used for tilt
        esp_now_send(remoteMacAddress, (uint8_t *)&tiltMsg, sizeof(tiltMsg));
    }
}

// PIR sensor interrupt service routine
void IRAM_ATTR triggerPIR()
{
    // Read the current state of the PIR sensor
    bool pirState = digitalRead(PIR_PIN);

    // Control blue LED based on PIR state
    if (pirState == HIGH)
    {
        // PIR sensor detected motion - turn on blue LED
        digitalWrite(BLUE_LED_PIN, HIGH);
    }
    else
    {
        // PIR sensor no motion detected - turn off blue LED
        digitalWrite(BLUE_LED_PIN, LOW);
    }
}

// Callback function to handle incoming ESP-NOW data
void handleEspNowRequestReceived(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    // Print sender MAC address
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println("Received ESP-NOW data from: " + String(macStr));

    // Print the size of the incoming data and the expected size
    Serial.println("Received data length: " + String(len));
    Serial.println("Expected data length: " + String(sizeof(ESPNOW_COMMAND)));

    if (len == sizeof(ESPNOW_COMMAND))
    {
        ESPNOW_COMMAND receivedMessage;
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));

        // Display the contents of the received message
        Serial.println("Command Type: " + String(receivedMessage.commandType));
        Serial.println("Fan Status: " + String(receivedMessage.fanState ? "On" : "Off"));

        switch (receivedMessage.commandType)
        {
        case COMMAND:
            Serial.println("Received command from remote device.");

            // Execute the command using control functions to update state variables
            espNowControlFan(receivedMessage.fanState);

            // **Send acknowledgment back to master**
            ESPNOW_COMMAND ackMessage;
            ackMessage.commandType = COMMAND_ACK;
            ackMessage.fanState = fanState;
            esp_now_send(remoteMacAddress, (uint8_t *)&ackMessage, sizeof(ackMessage));
            break;

        case STATUS_QUERY:
            Serial.println("Received status query from remote device.");

            // **Send status response back to master**
            ESPNOW_COMMAND statusMessage;
            statusMessage.commandType = STATUS_QUERY_RESPONSE;
            statusMessage.fanState = fanState;
            esp_now_send(remoteMacAddress, (uint8_t *)&statusMessage, sizeof(statusMessage));
            break;

        default:
            Serial.println("Unknown command type received from remote device.");
            break;
        }
    }
    else
    {
        Serial.println("Received data size mismatch");
    }
}

// Callback function to handle data sent via ESP-NOW
void handleEspNowSendStatus(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    String sendStatus = (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail";
    Serial.println("Last Packet Sent to: " + String(macStr) + " | Status: " + sendStatus);

    if (status != ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Note: 'Send Fail' may occur when devices are connected to Wi-Fi due to acknowledgment limitations.");
    }
}

void espNowControlFan(bool state)
{
    // Control fan via ESP-NOW command
    fanState = state;

    if (state)
    {
        fanSpeed = 255; // Full speed
        ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
        Serial.println("Remote command: Fan ON");
    }
    else
    {
        fanSpeed = 0; // Off
        ledcWrite(FAN_PWM_CHANNEL, fanSpeed);
        Serial.println("Remote command: Fan OFF");
    }
}