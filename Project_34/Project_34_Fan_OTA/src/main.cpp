#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

constexpr int FAN_PIN = 8;
// Define a constant for the default WiFi channel for ESP-NOW communication
constexpr int ESP_NOW_NETWORK_MODE = 1; // 0: Through Same WiFi Network, 1: Point-to-Point

// Declare the MAC address of the slave and master devices for ESP-NOW communication
//uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA8, 0xE6, 0xA4}; // the MAC address of the master device (present one) B0:81:84:A8:F1:BC
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA8, 0xE4, 0x04};

// Replace with your WiFi network credentials
const char *ssid = "sesplearningstudios";
const char *password = "@nn3nb3rg";

// LED pins
constexpr int LED_SETUP_PIN = 12; // LED 1: setup completion
constexpr int LED_WIFI_PIN = 13;  // LED 2: WiFi connected

// function declarations
void callbackCommandRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

void setup()
{
  Serial.begin(115200);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Start with fan OFF

  // Initialize LEDs
  pinMode(LED_SETUP_PIN, OUTPUT);
  pinMode(LED_WIFI_PIN, OUTPUT);
  digitalWrite(LED_SETUP_PIN, LOW); // Off initially
  digitalWrite(LED_WIFI_PIN, LOW);  // Off initially

  WiFi.mode(WIFI_STA);
  if (ESP_NOW_NETWORK_MODE == 0)
  {
    // Connect to WiFi network
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_WIFI_PIN, HIGH); // Turn on WiFi LED
  }
  else
  {
    // Point-to-point mode (no WiFi connection)
    WiFi.disconnect();
  }

  // ESP-NOW can be initialized after WiFi is ready
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(callbackCommandRecv);

  digitalWrite(LED_SETUP_PIN, HIGH); // Turn on setup complete LED
}

void loop()
{
  // Nothing needed here for slave device
}

// doxygen comment
/**
 * @brief Callback function for receiving commands from the master device
 * @param mac The MAC address of the sender
 * @param incomingData The incoming data
 * @param len The length of the incoming data
 */
void callbackCommandRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len < 1)
    return;
  // Only accept commands from the known master
  if (memcmp(mac, remoteMacAddress, 6) != 0)
  {
    Serial.print("Ignored command from unknown MAC: ");
    for (int i = 0; i < 6; ++i)
    {
      if (i > 0)
        Serial.print(":");
      Serial.print(mac[i], HEX);
    }
    Serial.println();
    return;
  }
  uint8_t command = incomingData[0];
  Serial.print("Received command: ");
  Serial.println(command);
  if (command == 1)
  {
    digitalWrite(FAN_PIN, HIGH); // Turn fan ON
  }
  else if (command == 0)
  {
    digitalWrite(FAN_PIN, LOW); // Turn fan OFF
  }
}