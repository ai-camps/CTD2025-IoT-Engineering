#include <Arduino.h>          // Include Arduino core library
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
constexpr auto GREEN_LED_PIN = 2;  // Green LED pin
constexpr auto RED_LED_PIN = 3;    // Red LED pin
constexpr auto BLUE_LED_PIN = 0;   // Blue LED pin
constexpr auto BUTTON_PIN = 10;    // Button pin

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;         // OLED display width
constexpr auto SCREEN_HEIGHT = 64;         // OLED display height
constexpr auto OLED_RESET = -1;            // Reset pin (or -1 if sharing Arduino reset pin)
constexpr auto SDA_PIN = 4;                // I2C SDA pin
constexpr auto SCL_PIN = 5;                // I2C SCL pin
constexpr auto SSD1306_I2C_ADDRESS = 0x3C; // OLED display address

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
constexpr auto OTA_HOSTNAME = "98fba88481b0"; // OTA hostname by device ID
bool isOtaInProgress = false;                 // Flag to track OTA status

// LED flags
bool isGreenLEDOn = false; // Green LED flag
bool isRedLEDOn = false;   // Red LED flag
bool isBlueLEDOn = false;  // Blue LED flag

// ESP-NOW variables
uint8_t localMacAddress[] = {0xB0, 0x81, 0x84, 0xA8, 0xFB, 0x98};  // the MAC address of the slave device (present one) B0:81:84:A8:FB:98
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA9, 0x11, 0xA0}; // the MAC address of the master device (remote one) B0:81:84:A9:11:11

// Define a constant for the default WiFi channel for ESP-NOW communication
constexpr auto ESPNOW_COMMUNICATION_MODE = 0; // 0: Through Same WiFi Network, 1: Point-to-Point

// Define ESP-NOW command types
enum ESPNOW_COMMAND_TYPE : uint8_t
{
    COMMAND = 0,
    COMMAND_ACK = 1,
    STATUS_QUERY = 2,
    STATUS_QUERY_RESPONSE = 3
};

// Define a struct to hold the message data
struct ESPNOW_COMMAND
{
    uint8_t commandType;
    bool fanState;
};

// ESP-NOW state variables
bool slaveFanState = false; // Current slave fan state
bool buttonPressed = false; // Button press flag

// Function prototypes
void initOled();                                                                            // Initialize OLED display
void connectWiFi();                                                                         // Connect to WiFi
String syncTime();                                                                          // Synchronize time
void initOTA();                                                                             // Initialize OTA updates
void initESPNow();                                                                          // Initialize ESP-NOW
void handleEspNowRequestReceived(const uint8_t *mac, const uint8_t *incomingData, int len); // ESP-NOW data receive callback
void handleEspNowSendStatus(const uint8_t *mac_addr, esp_now_send_status_t status);         // ESP-NOW data send callback
void IRAM_ATTR pressButton();                                                               // Button pressed interrupt service routine
void sendFanCommand(bool state);                                                            // Send fan command to slave device
void querySlaveStatus();                                                                    // Query slave device status
void updateDisplay();                                                                       // Update OLED display with control panel status

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

    // Configure button pin and attach interrupt
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Button pin with internal pull-up
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), pressButton, FALLING);

    // Initialize all LEDs to OFF state
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);

    // Initialize LED flags to match the actual LED states
    isGreenLEDOn = false;
    isRedLEDOn = false;
    isBlueLEDOn = false;

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

    // Handle button press to control slave fan
    if (buttonPressed)
    {
        Serial.println("Button pressed - will toggle slave fan");
        // Toggle slave fan state
        slaveFanState = !slaveFanState;
        // Send command to slave device
        sendFanCommand(slaveFanState);
        // Clear button flag
        buttonPressed = false;
        // Small delay to prevent multiple triggers
        delay(200);
    }

    // Update OLED display
    updateDisplay();

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

// Callback function to handle incoming ESP-NOW data
void handleEspNowRequestReceived(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(ESPNOW_COMMAND))
    {
        ESPNOW_COMMAND receivedMessage;
        memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));

        switch (receivedMessage.commandType)
        {
        case COMMAND_ACK:
            // Only update and print if the state actually changes
            if (slaveFanState != receivedMessage.fanState)
            {
                Serial.println("Slave fan state changed from " + String(slaveFanState ? "ON" : "OFF") + " to " + String(receivedMessage.fanState ? "ON" : "OFF"));
                slaveFanState = receivedMessage.fanState;
            }
            break;

        case STATUS_QUERY_RESPONSE:
            // Only update and print if the state actually changes
            if (slaveFanState != receivedMessage.fanState)
            {
                Serial.println("Slave fan status changed from " + String(slaveFanState ? "ON" : "OFF") + " to " + String(receivedMessage.fanState ? "ON" : "OFF"));
                slaveFanState = receivedMessage.fanState;
            }
            break;

        default:
            break;
        }
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

void IRAM_ATTR pressButton()
{
    buttonPressed = true; // Only set the flag, no Serial or blocking code
}

void sendFanCommand(bool state)
{
    // Create command message
    ESPNOW_COMMAND command;
    command.commandType = COMMAND;
    command.fanState = state;

    // Print target MAC address
    Serial.print("Sending fan command to MAC: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(remoteMacAddress[i], HEX);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();

    // Send command to slave device
    esp_err_t result = esp_now_send(remoteMacAddress, (uint8_t *)&command, sizeof(command));

    if (result == ESP_OK)
    {
        Serial.println("Fan command sent to slave: " + String(state ? "ON" : "OFF"));
    }
    else
    {
        Serial.println("Failed to send fan command to slave. Error: " + String(result));
    }
}

void querySlaveStatus()
{
    // Create status query message
    ESPNOW_COMMAND query;
    query.commandType = STATUS_QUERY;
    query.fanState = false; // Not used for status query

    // Send status query to slave device
    esp_err_t result = esp_now_send(remoteMacAddress, (uint8_t *)&query, sizeof(query));

    if (result == ESP_OK)
    {
        Serial.println("Status query sent to slave device");
    }
    else
    {
        Serial.println("Failed to send status query to slave");
    }
}

void updateDisplay()
{
    // Only update display if OTA is not in progress
    if (!isOtaInProgress)
    {
        // Update OLED display with control panel status
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Master Control Panel");
        display.println("DeviceID: " + deviceID);
        display.println("Date: " + currentDate);
        display.println("Time: " + String(timeClient.getFormattedTime()));
        display.println("WiFi: " + String(WiFi.isConnected() == 1 ? "Connected" : "Disconnected"));
        display.println("IP: " + String(WiFi.localIP().toString()));
        display.println("Slave Fan: " + String(slaveFanState ? "ON" : "OFF"));
        display.println("Press Button to Toggle");
        display.display();
    }
}