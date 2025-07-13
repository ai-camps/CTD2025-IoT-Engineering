#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

constexpr int RED_LED_PIN = 3;
constexpr int ESP_NOW_NETWORK_MODE = 1; // 0: Through Same WiFi Network, 1: Point-to-Point
constexpr int LED_SETUP_PIN = 12;
constexpr int LED_WIFI_PIN = 13;

// MAC address of the slave device (fan controller)
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA9, 0x1F, 0x3C};

// WiFi credentials
const char *ssid = "sesplearningstudios";
const char *password = "@nn3nb3rg";

// function declaration
void callbackMessageReceived(const uint8_t *mac, const uint8_t *data, int len);

void setup()
{
  Serial.begin(115200);
  pinMode(LED_SETUP_PIN, OUTPUT);
  pinMode(LED_WIFI_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(LED_SETUP_PIN, LOW); // Off initially
  digitalWrite(LED_WIFI_PIN, LOW);  // Off initially

  WiFi.mode(WIFI_STA);
  if (ESP_NOW_NETWORK_MODE == 0)
  {
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
    WiFi.disconnect();
  }

  Serial.print("Local MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(callbackMessageReceived);

  // Register peer (slave)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, remoteMacAddress, 6);
  peerInfo.channel = 0; // use current WiFi channel
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(remoteMacAddress))
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
  }

  digitalWrite(LED_SETUP_PIN, HIGH); // Turn on setup complete LED
}

void loop()
{
  // Remove buttonPressed logic
}

/**
 * @brief ESP-NOW receive callback
 *
 * This function is called when data is received from the slave ESP32.
 * It turns the RED LED on or off based on the received message (1 = ON, 0 = OFF).
 */
void callbackMessageReceived(const uint8_t *mac, const uint8_t *data, int len)
{
  if (len > 0)
  {
    uint8_t command = data[0];
    Serial.print("Received data from slave: ");
    Serial.println(command);
    if (command == 1)
    {
      digitalWrite(RED_LED_PIN, HIGH); // Turn ON RED LED
      Serial.println("RED LED ON");
    }
    else if (command == 0)
    {
      digitalWrite(RED_LED_PIN, LOW); // Turn OFF RED LED
      Serial.println("RED LED OFF");
    }
  }
}