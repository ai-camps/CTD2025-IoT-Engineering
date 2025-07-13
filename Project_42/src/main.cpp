#include <Arduino.h>
#include <WiFi.h>
#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>             // Include Wire library for I2C communication
#include <Adafruit_GFX.h>     // Include Adafruit GFX library for OLED display
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 library for OLED display
#include <WiFiManager.h>
#include <ThingsBoard.h>

// Global device ID
String deviceID;

// Pin definitions
constexpr auto SETUP_DONE_PIN = 12;
constexpr auto WIFI_CONNECTED_PIN = 13;
constexpr auto RED_PIN = 3;
constexpr auto GREEN_PIN = 2;
constexpr auto BLUE_PIN = 0;
constexpr auto FAN_PIN = 1;
constexpr auto COLLISION_PIN = 10;
constexpr auto TILT_PIN = 6;
constexpr auto PIR_PIN = 7;
constexpr auto DHT_PIN = 8;

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
DHT dht(DHT_PIN, DHT11);

// Function declarations
void initOled();

void updateOLEDInfo(); // New function
void readDHT11();
void controlLED();
void rotateFan();

void IRAM_ATTR onCollisionInterrupt();
void IRAM_ATTR onTiltInterrupt();
void IRAM_ATTR onPIRInterrupt();

void connectCloud();

// Volatile status variables for ISR
volatile bool collisionStatus = false;
volatile bool tiltStatus = false;
volatile bool pirStatus = false;

// Timing for non-blocking OLED update
unsigned long lastOledUpdate = 0;
const unsigned long oledUpdateInterval = 10000; // 10 seconds

// Store last sensor values
float lastTempC = 0.0;
float lastTempF = 0.0;
float lastHumidity = 0.0;

//======Cloud Access Settings======
// ThingsBoard settings
WiFiClient espClient;

// Set ThingsBoard server, port, access token, client ID, and retry delay
constexpr auto thingsboardServer = "dashboard.ai-camps.com";
constexpr auto thingsboardPort = 1884;
constexpr auto accessToken = "j5jbbcwi1fgffl1aqxc8"; // change to your own device access token when provided by the cloud providerin Thingsboard

// Define the desired MaxFieldsAmt (max number of fields in the JSON document)
constexpr unsigned int MAX_FIELDS_AMT = 64;

// Initialize ThingsBoard instance
ThingsBoardSized<MAX_FIELDS_AMT> tb(espClient);
//======Cloud Access Settings======

void setup()
{
  Serial.begin(115200);

  deviceID = String(ESP.getEfuseMac(), HEX);

  pinMode(SETUP_DONE_PIN, OUTPUT);
  pinMode(WIFI_CONNECTED_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(COLLISION_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(DHT_PIN, INPUT);

  digitalWrite(SETUP_DONE_PIN, LOW);
  digitalWrite(WIFI_CONNECTED_PIN, LOW);

  // Indicate setup() is completed
  digitalWrite(SETUP_DONE_PIN, HIGH);

  WiFi.mode(WIFI_STA);

  WiFiManager wm;

  wm.autoConnect("ESP32JUN");

  Serial.println("Connected!");

  // Indicate WiFi is connected
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(WIFI_CONNECTED_PIN, HIGH);
  }

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(COLLISION_PIN), onCollisionInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TILT_PIN), onTiltInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), onPIRInterrupt, CHANGE);

  // Initialize DHT sensor
  dht.begin();

  // Initialize OLED
  initOled();

  connectCloud(); // Maintain ThingsBoard connection
}

void loop()
{
  unsigned long now = millis();
  if (now - lastOledUpdate >= oledUpdateInterval)
  {
    lastOledUpdate = now;
    readDHT11(); // Only read sensor every 10 seconds
  }
  updateOLEDInfo(); // Update OLED as often as possible for responsiveness
}

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

// Function to update OLED with deviceID, IP, and temp_F
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

// ISR implementations
void IRAM_ATTR onCollisionInterrupt()
{
  collisionStatus = digitalRead(COLLISION_PIN) == LOW;
}
void IRAM_ATTR onTiltInterrupt()
{
  tiltStatus = digitalRead(TILT_PIN) == LOW;
}
void IRAM_ATTR onPIRInterrupt()
{
  pirStatus = digitalRead(PIR_PIN) == HIGH;
}

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

void connectCloud()
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
      return;
    }
    else
    {
      Serial.println("Connected to ThingsBoard!");
    }
  }
}
