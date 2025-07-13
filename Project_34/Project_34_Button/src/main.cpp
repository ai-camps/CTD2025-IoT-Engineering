#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

constexpr int BUTTON_PIN = 10;
constexpr int ESP_NOW_NETWORK_MODE = 1; // 0: Through Same WiFi Network, 1: Point-to-Point
constexpr int LED_SETUP_PIN = 12;
constexpr int LED_WIFI_PIN = 13;

// MAC address of the remote device (fan motor controller)
uint8_t remoteMacAddress[] = {0xB0, 0x81, 0x84, 0xA9, 0x1F, 0x3C};

// WiFi credentials
const char *ssid = "sesplearningstudios";
const char *password = "@nn3nb3rg";

// Button state tracking
volatile bool buttonPressed = false;
bool fanOn = false;

// function declaration
void IRAM_ATTR handleButtonInterrupt();

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_SETUP_PIN, OUTPUT);
  pinMode(LED_WIFI_PIN, OUTPUT);
  digitalWrite(LED_SETUP_PIN, LOW); // Off initially
  digitalWrite(LED_WIFI_PIN, LOW);  // Off initially
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);

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
  if (buttonPressed)
  {
    fanOn = !fanOn; // Toggle fan state
    uint8_t command = fanOn ? 1 : 0;
    esp_err_t result = esp_now_send(remoteMacAddress, &command, 1);
    Serial.print("Button pressed. Sent command: ");
    Serial.println(command);
    if (result == ESP_OK)
    {
      Serial.println("Send OK");
    }
    else
    {
      Serial.print("Send Error: ");
      Serial.println(result);
    }
    buttonPressed = false;
  }
}

// Doxygen comment
/**
 * @brief Handle button interrupt
 *
 * This function is called when the button is pressed.
 * It sets the buttonPressed flag to true.
 */
void IRAM_ATTR handleButtonInterrupt()
{
  buttonPressed = true;
}