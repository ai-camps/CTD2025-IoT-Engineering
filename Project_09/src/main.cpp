#include <Arduino.h>
#include <DHT.h>

// LED pin definition
constexpr auto SETUP_LED_PIN = 12; // Setup LED pin
constexpr auto LOOP_LED_PIN = 13;  // Loop LED pin
constexpr auto DHT11_PIN = 8;      // DHT11 pin

// DHT sensor object
DHT dht11(DHT11_PIN, DHT11);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(SETUP_LED_PIN, OUTPUT); // Setup LED
    pinMode(LOOP_LED_PIN, OUTPUT);  // Loop LED
    pinMode(DHT11_PIN, INPUT);      // DHT11 pin

    // Initialize DHT sensor
    dht11.begin();

    // Turn on LED to indicate setup is complete
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED

    // Print confirmation message
    Serial.println("Setup completed successfully - LED on GPIO 12 is now ON");
    Serial.println("DHT11 sensor initialized");

    // Wait for 3 seconds
    delay(3000);
}

void loop()
{
    // Main loop - LED remains on
    digitalWrite(LOOP_LED_PIN, HIGH);  // Turn on loop LED
    digitalWrite(SETUP_LED_PIN, LOW);  // Turn off setup LED
    delay(5000);                       // Wait for 5 seconds
    digitalWrite(LOOP_LED_PIN, LOW);   // Turn off loop LED
    digitalWrite(SETUP_LED_PIN, HIGH); // Turn on setup LED
    delay(5000);                       // Wait for 5 seconds

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
    Serial.println("=== DHT11 Sensor Readings ===");
    Serial.print("Temperature (Celsius): ");
    Serial.print(temperatureC);
    Serial.println(" °C");
    Serial.print("Temperature (Fahrenheit): ");
    Serial.print(temperatureF);
    Serial.println(" °F");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println("=============================");
}
