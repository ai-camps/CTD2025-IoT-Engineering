#include <Arduino.h>
#include <DHT.h>

// LED pin definition
constexpr auto DHT11_PIN = 18; // DHT11 pin

// User email for notifications
constexpr auto USER_EMAIL = "you@example.com"; // Replace with your email

// DHT sensor object
DHT dht11(DHT11_PIN, DHT11);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(DHT11_PIN, OUTPUT); // Setup LED

    // Initialize DHT sensor
    dht11.begin();

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
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
    Serial.println("=========================================");
    // Add a delay before the next reading
    delay(3000);
}
