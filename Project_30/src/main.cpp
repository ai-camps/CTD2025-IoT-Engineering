#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// Global device ID
String deviceID;

// Pin definitions
constexpr auto SETUP_LED_PIN = 12;
constexpr auto WIFI_LED_PIN = 13;
constexpr auto DHT11_PIN = 8;
constexpr auto GREEN_LED_PIN = 2;
constexpr auto RED_LED_PIN = 3;
constexpr auto BLUE_LED_PIN = 0;
constexpr auto FAN_PIN = 7;
constexpr auto BUZZER_PIN = 1;

// PWM settings
constexpr auto PWM_FREQUENCY = 5000;
constexpr auto PWM_RESOLUTION = 8;
constexpr auto PWM_CHANNEL = 0; // Red LED
constexpr auto BLINK_INTERVAL = 500;
constexpr auto PWM_DUTY_CYCLE = 128;
constexpr auto PWM_CHANNELG = 1;       // Green LED
constexpr auto PWM_CHANNELB = 2;       // Blue LED
constexpr auto PWM_CHANNEL_BUZZER = 3; // Buzzer

// OLED display settings
constexpr auto SCREEN_WIDTH = 128;
constexpr auto SCREEN_HEIGHT = 64;
constexpr auto OLED_RESET = -1;
constexpr auto SDA_PIN = 4;
constexpr auto SCL_PIN = 5;
constexpr auto SSD1306_I2C_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT11
DHT dht11(DHT11_PIN, DHT11);
constexpr auto DHT11_UPDATE_INTERVAL = 5000;
unsigned long lastDht11Read = 0;
bool isTemperatureTooHigh = false;
bool isTemperatureTooLow = false;
bool isTemperatureNormal = false;
constexpr auto TEMP_C_HIGH_THRESHOLD = 24;
constexpr auto TEMP_C_LOW_THRESHOLD = 22;

// WiFi
constexpr auto WIFI_SSID = "sesplearningstudios";
constexpr auto WIFI_PASSWORD = "@nn3nb3rg";
bool isWiFiConnected = false;

// NTP
constexpr auto NTP_SERVER = "pool.ntp.org";
constexpr auto NTP_UPDATE_INTERVAL = 3600000;
constexpr auto NTP_OFFSET = -8 * 3600;
constexpr auto NTP_INTERVAL = 60000;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, NTP_UPDATE_INTERVAL);
String currentDate = "";

// OTA
constexpr auto OTA_PORT = 3232;
constexpr auto OTA_HOSTNAME = "a011a98481b0";
bool isOtaInProgress = false;

// LED flags
bool isGreenLEDOn = false;
bool isRedLEDOn = false;
bool isBlueLEDOn = false;
bool isRedLEDBlinking = false;

// PWM blinking variables
unsigned long lastBlinkTime = 0;

// Siren state
unsigned long lastToneChange = 0;
bool highPitch = true;

// Function prototypes
void initOled();
void connectWiFi();
String syncTime();
void initOTA();
void readDht11();
void setLED();
void setFan();
void updateRedLEDBlink();
void soundBuzzer();

void setup()
{
    Serial.begin(115200);
    deviceID = String(ESP.getEfuseMac(), HEX);

    pinMode(SETUP_LED_PIN, OUTPUT);
    pinMode(WIFI_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(RED_LED_PIN, PWM_CHANNEL);
    ledcSetup(PWM_CHANNELG, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(GREEN_LED_PIN, PWM_CHANNELG);
    ledcSetup(PWM_CHANNELB, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(BLUE_LED_PIN, PWM_CHANNELB);
    ledcSetup(PWM_CHANNEL_BUZZER, 2000, PWM_RESOLUTION); // Initial freq 2kHz
    ledcAttachPin(BUZZER_PIN, PWM_CHANNEL_BUZZER);

    ledcWrite(PWM_CHANNELG, 0);
    ledcWrite(PWM_CHANNEL, 0);
    ledcWrite(PWM_CHANNELB, 0);
    ledcWriteTone(PWM_CHANNEL_BUZZER, 0);

    isGreenLEDOn = false;
    isRedLEDOn = false;
    isBlueLEDOn = false;
    isRedLEDBlinking = false;

    dht11.begin();
    initOled();
    connectWiFi();
    digitalWrite(WIFI_LED_PIN, isWiFiConnected ? HIGH : LOW);
    currentDate = syncTime();
    initOTA();
    digitalWrite(SETUP_LED_PIN, HIGH);

    Serial.println("Setup completed successfully");
}

void loop()
{
    ArduinoOTA.handle();
    if (isTemperatureTooHigh)
    {
        updateRedLEDBlink();
        soundBuzzer();
    }
    else
    {
        ledcWriteTone(PWM_CHANNEL_BUZZER, 0); // Silence
    }

    if (millis() - lastDht11Read >= DHT11_UPDATE_INTERVAL)
    {
        readDht11();
        setLED();
        setFan();
        lastDht11Read = millis();
    }

    timeClient.update();
}

void soundBuzzer()
{
    // Non-blocking siren (switches pitch every 350ms)
    if (isTemperatureTooHigh)
    {
        unsigned long now = millis();
        if (now - lastToneChange > 350) // Change tone every 0.35 sec
        {
            if (highPitch)
            {
                ledcWriteTone(PWM_CHANNEL_BUZZER, 1400); // High pitch
            }
            else
            {
                ledcWriteTone(PWM_CHANNEL_BUZZER, 500); // Low pitch
            }
            highPitch = !highPitch;
            lastToneChange = now;
        }
    }
}

void setLED()
{
    if (isTemperatureNormal)
    {
        ledcWrite(PWM_CHANNELG, 255);
        ledcWrite(PWM_CHANNEL, 0);
        ledcWrite(PWM_CHANNELB, 0);
        isGreenLEDOn = true;
        isRedLEDOn = false;
        isBlueLEDOn = false;
        isRedLEDBlinking = false;
        Serial.println("Temperature is normal - Green LED ON");
    }
    else if (isTemperatureTooLow)
    {
        ledcWrite(PWM_CHANNELG, 0);
        ledcWrite(PWM_CHANNEL, 0);
        ledcWrite(PWM_CHANNELB, 255);
        isGreenLEDOn = false;
        isRedLEDOn = false;
        isBlueLEDOn = true;
        isRedLEDBlinking = false;
        Serial.println("Temperature is too low - Blue LED ON");
    }
    else if (isTemperatureTooHigh)
    {
        ledcWrite(PWM_CHANNELG, 0);
        ledcWrite(PWM_CHANNELB, 0);
        isGreenLEDOn = false;
        isRedLEDOn = true;
        isBlueLEDOn = false;
        isRedLEDBlinking = true;
        Serial.println("Temperature is too high - Red LED blinking with PWM");
    }
}

void setFan()
{
    digitalWrite(FAN_PIN, isTemperatureTooHigh ? HIGH : LOW);
}

void updateRedLEDBlink()
{
    if (isRedLEDBlinking && millis() - lastBlinkTime >= BLINK_INTERVAL)
    {
        if (isRedLEDOn)
        {
            ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE);
        }
        else
        {
            ledcWrite(PWM_CHANNEL, 0);
        }
        isRedLEDOn = !isRedLEDOn;
        lastBlinkTime = millis();
    }
}

void initOled()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
}

void connectWiFi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    isWiFiConnected = true;

    if (!MDNS.begin(OTA_HOSTNAME))
    {
        Serial.println("Error setting up MDNS responder!");
    }
    else
    {
        Serial.println("mDNS responder started for OTA updates: " + String(OTA_HOSTNAME));
    }
}

String syncTime()
{
    if (!isWiFiConnected)
    {
        Serial.println("WiFi not connected. Cannot sync time.");
        return "WiFi Not Connected";
    }
    timeClient.begin();
    timeClient.setTimeOffset(NTP_OFFSET);
    Serial.println("Waiting for NTP synchronization...");
    timeClient.update();

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

void initOTA()
{
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        isOtaInProgress = true;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Start OTA " + type);
        display.display(); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        int progressPercent = (progress * 100) / total;
        display.fillRect(0, 10, SCREEN_WIDTH, 8, SSD1306_BLACK);
        display.setCursor(0, 10);
        display.println("Progress: " + String(progressPercent) + "%");
        display.display(); });

    ArduinoOTA.onEnd([]()
                     {
        display.println("\nOTA End");
        display.display();
        delay(2000);
        isOtaInProgress = false; });

    ArduinoOTA.onError([](ota_error_t error)
                       {
        String errorMsg = "Error[" + String(error) + "]: ";
        if (error == OTA_AUTH_ERROR)      errorMsg += "Auth Failed";
        else if (error == OTA_BEGIN_ERROR)   errorMsg += "Begin Failed";
        else if (error == OTA_CONNECT_ERROR) errorMsg += "Connect Failed";
        else if (error == OTA_RECEIVE_ERROR) errorMsg += "Receive Failed";
        else if (error == OTA_END_ERROR)     errorMsg += "End Failed";
        display.println(errorMsg);
        display.display();
        delay(3000);
        isOtaInProgress = false; });

    ArduinoOTA.begin();
}

void readDht11()
{
    float humidity = dht11.readHumidity();
    float temperatureC = dht11.readTemperature();
    float temperatureF = dht11.readTemperature(true);

    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF))
    {
        Serial.println("Failed to read from DHT11 sensor!");
    }
    else
    {
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

        if (!isOtaInProgress)
        {
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