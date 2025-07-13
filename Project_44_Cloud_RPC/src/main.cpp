#include <Arduino.h>          // Include Arduino library
#include <WiFi.h>             // Include WiFi library
#include <Arduino.h>          // Include Arduino library
#include <DHT.h>              // Include DHT library for temperature and humidity sensor
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#include <ArduinoOTA.h>       // Library for OTA updates
#include <ThingsBoard.h>      // Include ThingsBoard library for data transmission
#include <ArduinoJson.h>      // Include ArduinoJson library for JSON parsing

// Global device ID
String deviceID;

// Pin definitions
constexpr auto SETUP_DONE_PIN = 12;     // Pin for setup done indicator
constexpr auto WIFI_CONNECTED_PIN = 13; // Pin for WiFi connected indicator
constexpr auto RED_LED_PIN = 3;         // Pin for red LED
constexpr auto GREEN_LED_PIN = 2;       // Pin for green LED
constexpr auto BLUE_LED_PIN = 0;        // Pin for blue LED
constexpr auto FAN_PIN = 1;             // Pin for fan
constexpr auto COLLISION_PIN = 10;      // Pin for collision sensor
constexpr auto TILT_PIN = 6;            // Pin for tilt sensor
constexpr auto PIR_PIN = 7;             // Pin for PIR sensor
constexpr auto DHT_PIN = 8;             // Pin for DHT sensor

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;         // OLED display width
constexpr auto SCREEN_HEIGHT = 64;         // OLED display height
constexpr auto OLED_RESET = -1;            // Reset pin (or -1 if sharing Arduino reset pin)
constexpr auto SDA_PIN = 4;                // I2C SDA pin
constexpr auto SCL_PIN = 5;                // I2C SCL pin
constexpr auto SSD1306_I2C_ADDRESS = 0x3C; // OLED display address

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT sensor
DHT dht(DHT_PIN, DHT11); // Initialize DHT sensor

// WiFi settings
constexpr auto WIFI_SSID = "sesplearningstudios"; // CTD WiFi SSID
constexpr auto WIFI_PASSWORD = "@nn3nb3rg";       // CTD WiFi password

// OTA settings
constexpr auto OTA_PORT = 3232; // OTA port

// ThingsBoard settings
WiFiClient espClient;

// Set ThingsBoard server, port, access token, client ID, and retry delay
constexpr auto thingsboardServer = "dashboard.ai-camps.com";
constexpr auto thingsboardPort = 1884;
constexpr auto accessToken = "j5jbbcwi1fgffl1aqxc8";

// Define the desired MaxFieldsAmt (max number of fields in the JSON document)
constexpr unsigned int MAX_FIELDS_AMT = 64;

// Initialize ThingsBoard instance
ThingsBoardSized<MAX_FIELDS_AMT> tb(espClient);

// Volatile status variables for ISR
volatile bool collisionStatus = false; // Collision status
volatile bool tiltStatus = false;      // Tilt status
volatile bool pirStatus = false;       // PIR status

// Timing for non-blocking OLED update
unsigned long lastOledUpdate = 0;               // Last OLED update time
const unsigned long oledUpdateInterval = 5000; // 1 minute (60000 ms) for OLED update interval

// Store last sensor values
float lastTempC = 0.0;    // Last temperature in Celsius
float lastTempF = 0.0;    // Last temperature in Fahrenheit
float lastHumidity = 0.0; // Last humidity

// Function declarations
bool connectWiFi();                          // Connect to WiFi
bool connectCloud();                         // Connect to ThingsBoard cloud
bool sendDataToCloud();                      // Send data to ThingsBoard cloud
void initOled();                             // Initialize OLED display
void initOTA();                              // Initialize OTA
void updateOLEDInfo();                       // Update OLED display
void readDHT11();                            // Read DHT11 sensor
void IRAM_ATTR onCollisionInterrupt();       // Interrupt Service Routine for collision sensor
void IRAM_ATTR onTiltInterrupt();            // Interrupt Service Routine for tilt sensor
void IRAM_ATTR onPIRInterrupt();             // Interrupt Service Routine for PIR sensor
RPC_Response setState(const RPC_Data &data); // Rotate fan through RPC function from ThingsBoard
RPC_Response getState(const RPC_Data &data); // Get fan state through RPC function from ThingsBoard

/**
 * @brief Setup function for the program.
 *
 * Initializes serial communication, device ID, and pins.
 * Connects to WiFi, attaches interrupts, initializes DHT sensor,
 * and initializes OLED display.
 *
 * @return void
 */
void setup()
{
  Serial.begin(115200); // Initialize serial communication

  deviceID = String(ESP.getEfuseMac(), HEX); // Get device ID from ESP32

  pinMode(SETUP_DONE_PIN, OUTPUT);     // Setup done indicator
  pinMode(WIFI_CONNECTED_PIN, OUTPUT); // WiFi connected indicator
  pinMode(RED_LED_PIN, OUTPUT);        // Red LED
  pinMode(GREEN_LED_PIN, OUTPUT);      // Green LED
  pinMode(BLUE_LED_PIN, OUTPUT);       // Blue LED
  pinMode(FAN_PIN, OUTPUT);            // Fan
  pinMode(COLLISION_PIN, INPUT);       // Collision sensor
  pinMode(TILT_PIN, INPUT);            // Tilt sensor
  pinMode(PIR_PIN, INPUT);             // PIR sensor
  pinMode(DHT_PIN, INPUT);             // DHT sensor

  digitalWrite(SETUP_DONE_PIN, LOW);     // Setup done indicator
  digitalWrite(WIFI_CONNECTED_PIN, LOW); // WiFi connected indicator

  // Connect to WiFi
  connectWiFi(); // Connect to WiFi

  // Indicate WiFi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(WIFI_CONNECTED_PIN, HIGH); // Indicate WiFi is connected
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(COLLISION_PIN), onCollisionInterrupt, CHANGE); // Attach collision interrupt
  attachInterrupt(digitalPinToInterrupt(TILT_PIN), onTiltInterrupt, CHANGE);           // Attach tilt interrupt
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), onPIRInterrupt, CHANGE);             // Attach PIR interrupt

  // Initialize DHT sensor
  dht.begin(); // Initialize DHT sensor
  Serial.println("DHT sensor initialized");

  // Initialize OLED
  initOled(); // Initialize OLED
  Serial.println("OLED initialized");

  // Initialize OTA
  initOTA(); // Initialize OTA
  Serial.println("OTA initialized");

  // Connect to ThingsBoard cloud
  connectCloud();    // Maintain ThingsBoard connection
  sendDataToCloud(); // Send initial data to ThingsBoard

  // Register RPC functions
  static RPC_Callback setStateCallback;
  setStateCallback.Set_Name("setState");
  setStateCallback.Set_Callback([](const RPC_Data &data) -> RPC_Response
                                { return setState(data); });
  tb.RPC_Subscribe(setStateCallback);

  static RPC_Callback getStateCallback;
  getStateCallback.Set_Name("getState");
  getStateCallback.Set_Callback([](const RPC_Data &data) -> RPC_Response
                                { return getState(data); });
  tb.RPC_Subscribe(getStateCallback);
}

/**
 * @brief Main loop for the program. Handles OTA updates, reads DHT11 sensor, and updates OLED display.
 *
 * @return void
 */
void loop()
{
  // Handle OTA updates
  ArduinoOTA.handle();

  // Process incoming RPCs
  tb.loop(); // <-- Add this line to process incoming RPCs

  unsigned long now = millis();
  if (now - lastOledUpdate >= oledUpdateInterval)
  {
    lastOledUpdate = now; // Update last OLED update time
    readDHT11();          // Only read sensor every 10 seconds
    sendDataToCloud();    // Send data to ThingsBoard cloud
  }

  updateOLEDInfo(); // Update OLED as often as possible for responsiveness
}

/**
 * @brief Initialize the OLED display with I2C and show a startup message.
 *
 * @return void
 */
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

/**
 * @brief Update the OLED display with device ID, IP, temperature, humidity, and sensor statuses.
 *
 * @return void
 */
void updateOLEDInfo()
{
  float tempC = lastTempC;
  float tempF = lastTempF;
  float humidity = lastHumidity;

  String collisionStatusStr = collisionStatus ? "ON" : "OFF";
  String tiltStatusStr = tiltStatus ? "ON" : "OFF";
  String pirStatusStr = pirStatus ? "ON" : "OFF";

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); // 1st row
  display.println("ID: " + deviceID);
  display.setCursor(0, 10); // 2nd row
  display.println("IP: " + WiFi.localIP().toString());
  display.setCursor(0, 20); // 3rd row
  display.println("Temp_F/C: " + String(tempF, 1) + "/" + String(tempC, 1));
  display.setCursor(0, 30); // 4th row
  display.println("Humidity: " + String(humidity, 1));
  display.setCursor(0, 40); // 5th row
  display.println("Collision: " + collisionStatusStr);
  display.setCursor(0, 50); // 6th row
  display.println("Tilt: " + tiltStatusStr);
  display.setCursor(0, 60); // 7th row
  display.println("PIR: " + pirStatusStr);
  display.display();
}

/**
 * @brief Interrupt Service Routine for collision sensor. Updates collisionStatus.
 *
 * This function is attached to the collision sensor interrupt pin and updates the global
 * collisionStatus variable based on the sensor state.
 *
 * @return void
 */
void IRAM_ATTR onCollisionInterrupt()
{
  collisionStatus = digitalRead(COLLISION_PIN) == LOW; // HIGH when collision detected
  digitalWrite(FAN_PIN, collisionStatus ? HIGH : LOW); // Turn on fan if collision detected
}

/**
 * @brief Interrupt Service Routine for tilt sensor. Updates tiltStatus.
 *
 * This function is attached to the tilt sensor interrupt pin and updates the global
 * tiltStatus variable based on the sensor state.
 *
 * @return void
 */
void IRAM_ATTR onTiltInterrupt()
{
  tiltStatus = digitalRead(TILT_PIN) == LOW;          // HIGH when tilt detected
  digitalWrite(RED_LED_PIN, tiltStatus ? HIGH : LOW); // Turn on red LED if tilt detected
}

/**
 * @brief Interrupt Service Routine for PIR sensor. Updates pirStatus.
 *
 * This function is attached to the PIR sensor interrupt pin and updates the global
 * pirStatus variable based on the sensor state.
 *
 * @return void
 */
void IRAM_ATTR onPIRInterrupt()
{
  pirStatus = digitalRead(PIR_PIN) == HIGH; // HIGH when motion detected
  digitalWrite(BLUE_LED_PIN, pirStatus ? HIGH : LOW); // Turn on green LED if motion detected
}

/**
 * @brief Read temperature and humidity from the DHT11 sensor and update last values.
 *
 * Reads the temperature (Celsius and Fahrenheit) and humidity from the DHT11 sensor.
 * Updates the global variables lastTempC, lastTempF, and lastHumidity.
 *
 * @return void
 */
void readDHT11()
{
  float tC = dht.readTemperature();
  float tF = dht.readTemperature(true);
  float h = dht.readHumidity();
  if (!isnan(tC))
    lastTempC = tC;
  if (!isnan(tF))
    lastTempF = tF;
  if (!isnan(h))
    lastHumidity = h;
}

/**
 * @brief Connect to WiFi and setup mDNS for OTA updates.
 *
 * @return bool True if connected, false otherwise
 */
bool connectWiFi()
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
  return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief Initialize Arduino OTA (Over-the-Air) update functionality and set up OTA event handlers.
 *
 * Sets the OTA port and hostname, and attaches event handlers for OTA start, end, progress, and error events.
 *
 * @return void
 */
void initOTA()
{
  ArduinoOTA.setPort(OTA_PORT);

  ArduinoOTA.onStart([]()
                     {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.println("Progress: " + String((progress * 100) / total) + "%"); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
        String errorMsg = "Error[" + String(error) + "]: ";
        if (error == OTA_AUTH_ERROR) {
            errorMsg += "Auth Failed";
        } else if (error == OTA_BEGIN_ERROR) {
            errorMsg += "Begin Failed";
        } else if (error == OTA_CONNECT_ERROR) {
            errorMsg += "Connect Failed";
        } else if (error == OTA_RECEIVE_ERROR) {
            errorMsg += "Receive Failed";
        } else if (error == OTA_END_ERROR) {
            errorMsg += "End Failed";
        }
        Serial.println(errorMsg); });

  // Start OTA
  ArduinoOTA.begin();
}

/**
 * @brief Connect to ThingsBoard cloud.
 * @param
 * @return bool True if connected, false otherwise
 */
bool connectCloud()
{
  if (!tb.connected())
  {
    Serial.print("Connecting to ThingsBoard at ");
    Serial.print(thingsboardServer);
    Serial.print(":");
    Serial.println(thingsboardPort);
    if (!tb.connect(thingsboardServer, accessToken, thingsboardPort))
    {
      Serial.println("Failed to connect to ThingsBoard");
      return false;
    }
    else
    {
      Serial.println("Connected to ThingsBoard!");
      return true;
    }
  }
  // Already connected
  return true;
}

/**
 * @brief Send data to ThingsBoard cloud.
 *
 * @param jsonDoc JSON document
 * @return bool True if telemetry sent successfully, false otherwise
 */
bool sendDataToCloud()
{
  if (!tb.connected())
  {
    if (!connectCloud())
    {
      Serial.println("Still not connected. Skipping telemetry send.");
      return false;
    }
  }

  // Set the buffer size by bytes for the JSON document
  const size_t jsonBufferSize = 256;
  StaticJsonDocument<256> jsonDoc;

  // Set the buffer size for the JSON document (if supported by tb)
  tb.setBufferSize(jsonBufferSize);

  // Clear the JSON document
  jsonDoc.clear();

  // Fill in the requested fields
  jsonDoc["deviceID"] = deviceID;
  jsonDoc["Owner"] = "Jun";
  jsonDoc["IP"] = WiFi.localIP().toString();
  jsonDoc["tempF"] = lastTempF;
  jsonDoc["tempC"] = lastTempC;
  jsonDoc["humidity"] = lastHumidity;
  jsonDoc["tilt"] = tiltStatus ? "On" : "Off";
  jsonDoc["collision"] = collisionStatus ? "On" : "Off";
  jsonDoc["pir"] = pirStatus ? "On" : "Off";
  jsonDoc["fan"] = digitalRead(FAN_PIN) == HIGH ? "On" : "Off";

  // Serialize the JSON document into a string
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  Serial.println("Publishing message: ");
  Serial.println(jsonString);

  // Send the JSON document directly
  if (!tb.sendTelemetryJson(jsonString.c_str()))
  {
    Serial.println("Failed to send telemetry");
    return false;
  }
  else
  {
    Serial.println("Telemetry sent successfully");
    return true;
  }
}

/**
 * @brief Rotate the fan via RPC using JSON params and respond with JSON.
 *
 * @param data RPC data
 * @return RPC_Response RPC response as JSON string
 */
RPC_Response setState(const RPC_Data &data)
{
  bool turnOn = false;

  // Only accept object with a "fan" bool property
  if (data["fan"].is<bool>())
  {
    turnOn = data["fan"];
  }
  else
  {
    Serial.println("Invalid RPC params for setState (expecting {\"fan\":bool})");
    StaticJsonDocument<64> errDoc;
    errDoc["error"] = "Invalid params";
    String errResp;
    serializeJson(errDoc, errResp);
    return RPC_Response(nullptr, errResp.c_str());
  }

  digitalWrite(FAN_PIN, turnOn ? HIGH : LOW);
  Serial.println(turnOn ? "Fan turned ON via RPC" : "Fan turned OFF via RPC");

  StaticJsonDocument<64> doc;
  doc["fan"] = turnOn;
  String response;
  serializeJson(doc, response);
  return RPC_Response(nullptr, response.c_str());
}

/**
 * @brief Get the state of the fan as JSON.
 *
 * @param data RPC data
 * @return RPC_Response RPC response as JSON string
 */
RPC_Response getState(const RPC_Data &data)
{
  bool isOn = digitalRead(FAN_PIN) == HIGH;
  StaticJsonDocument<64> doc;
  doc["fan"] = isOn;
  String response;
  serializeJson(doc, response);
  // Optionally, for debugging:
  Serial.println("getState response: " + response);
  return RPC_Response(nullptr, response.c_str());
}