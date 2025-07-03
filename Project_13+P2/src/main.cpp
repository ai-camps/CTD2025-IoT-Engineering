#include <Arduino.h>
#include <DHT.h>

// LED pin definition
constexpr auto DHT11_PIN = 8;     // DHT11 pin
constexpr auto INIT_LED_PIN = 13; // Initial LED pin
constexpr auto LOOP_LED_PIN = 12; // Initial LED pin
constexpr auto PIR_PIN = 7;       // PIR pin
constexpr auto TILT_PIN = 6;      // Tilt pin
constexpr auto COLLISION_PIN = 1; // Collision pin
constexpr auto BUTTON_PIN = 10;   // Buzzer pin

// User email for notifications
constexpr auto USER_EMAIL = "your@example.com"; // Replace with your email

// DHT sensor object
DHT dht11(DHT11_PIN, DHT11);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(DHT11_PIN, INPUT);     // Setup LED
    pinMode(INIT_LED_PIN, OUTPUT); // Setup LED
    pinMode(LOOP_LED_PIN, OUTPUT); // Setup LED
    pinMode(PIR_PIN, INPUT);       // Setup LED
    pinMode(TILT_PIN, INPUT);      // Setup LED
    pinMode(COLLISION_PIN, INPUT); // Setup LED
    pinMode(BUTTON_PIN, INPUT);    // Setup LED

    // Initialize DHT sensor
    dht11.begin();

    // Wait for 3 seconds
    delay(3000);
    digitalWrite(INIT_LED_PIN, HIGH);
}

void loop()
{
    digitalWrite(LOOP_LED_PIN, HIGH);

    // Read DHT11 sensor data
    float humidity = dht11.readHumidity();            // Read humidity
    float temperatureC = dht11.readTemperature();     // Read temperature in Celsius
    float temperatureF = dht11.readTemperature(true); // Read temperature in Fahrenheit

    // Check if readings are valid
    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF))
    {
        Serial.println("Failed to read from DHT11 sensor!");
        return;
    }

    // Print sensor readings
    Serial.println("========= DHT11 Sensor Readings =========");
    Serial.print("Temperature (Celsius): ");
    Serial.print(temperatureC);
    Serial.println(" °C");
    Serial.print("Temperature (Fahrenheit): ");
    Serial.print(temperatureF);
    Serial.println(" °F");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println(USER_EMAIL);
    Serial.println("========= LED Readings =========");
    Serial.print("INIT LED STATUS: ");
    Serial.println(digitalRead(INIT_LED_PIN) ? "ON" : "OFF");
    Serial.print("LOOP LED STATUS: ");
    Serial.println(digitalRead(LOOP_LED_PIN) ? "ON" : "OFF");
    Serial.print("PIR STATUS: ");
    Serial.println(digitalRead(PIR_PIN) ? "NOT DETECTED" : "DETECTED");
    Serial.print("TILT STATUS: ");
    Serial.println(digitalRead(TILT_PIN) ? "NOT TILTED" : "TILTED");
    Serial.print("COLLISION STATUS: ");
    Serial.println(digitalRead(COLLISION_PIN) ? "NOT DETECTED" : "DETECTED");
    Serial.print("BUTTON STATUS: ");
    Serial.println(digitalRead(BUTTON_PIN) ? "RELEASED" : "PRESSED");
    Serial.println("=========================================");
    // Add a delay before the next reading
    delay(3000);
    digitalWrite(LOOP_LED_PIN, LOW);
    delay(3000);
}
