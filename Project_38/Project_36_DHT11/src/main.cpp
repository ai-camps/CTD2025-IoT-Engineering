#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

constexpr int TILT_PIN = 6;
// Define a constant for the default WiFi channel for ESP-NOW communication
constexpr int ESP_NOW_NETWORK_MODE = 1; // 0: Through Same WiFi Network, 1: Point-to-Point

// Declare the MAC address of the slave and master devices for ESP-NOW communication
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA8, 0xF1, 0xBC}; // the MAC address of the master device (present one) B0:81:84:A8:F1:BC

// Replace with your WiFi network credentials
const char *ssid = "sesplearningstudios";
const char *password = "@nn3nb3rg";

// LED pins
constexpr int LED_SETUP_PIN = 12; // LED 1: setup completion
constexpr int LED_WIFI_PIN = 13;  // LED 2: WiFi connected

volatile bool tiltChanged = false;
volatile int tiltState = HIGH;

// function declarations
void callbackMessageSend(const uint8_t *mac_addr, esp_now_send_status_t status);
void IRAM_ATTR onTiltInterrupt();

void setup()
{
  Serial.begin(115200);
  pinMode(TILT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TILT_PIN), onTiltInterrupt, CHANGE);

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

  // Add peer registration for master device
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, remoteMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(remoteMacAddress))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  esp_now_register_send_cb(callbackMessageSend);

  digitalWrite(LED_SETUP_PIN, HIGH); // Turn on setup complete LED
}

void loop()
{
  if (tiltChanged)
  {
    tiltChanged = false;
    uint8_t message = (tiltState == LOW) ? 1 : 0; // LOW = triggered, HIGH = reset
    esp_err_t result = esp_now_send(remoteMacAddress, &message, sizeof(message));
    Serial.print("Tilt event sent to master. Value: ");
    Serial.println(message);
    if (result != ESP_OK)
    {
      Serial.print("Error sending tilt event: ");
      Serial.println(result);
    }
  }
}

// doxygen comment
/**
 * @brief Callback function for receiving commands from the master device
 * @param mac_addr The MAC address of the sender
 * @param status The status of the send operation
 */
void callbackMessageSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send to: ");
  for (int i = 0; i < 6; ++i)
  {
    if (i > 0)
      Serial.print(":");
    Serial.print(mac_addr[i], HEX);
  }
  Serial.print(" Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Doxygen comment
/**
 * @brief Interrupt service routine for the tilt sensor
 */
void IRAM_ATTR onTiltInterrupt()
{
  tiltState = digitalRead(TILT_PIN);
  tiltChanged = true;
}
