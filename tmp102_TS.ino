#include <Wire.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// Define TMP102 I2C address
#define TMP102_ADDR 0x48  

// WiFi credentials
const char* ssid = "Badma'";
const char* password = "8122322893";

// ThingSpeak settings
unsigned long channelID = 2876792; // Replace with your channel ID
const char* apiKey = "JOS5L83U0Y3N8SZC"; // Replace with your ThingSpeak Write API key

WiFiClient client;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 19); // SDA -> GPIO21, SCL -> GPIO19 (or use GPIO22 for SCL)
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Initialize ThingSpeak
    ThingSpeak.begin(client);
}

void loop() {
    // Read the temperature from TMP102
    Wire.beginTransmission(TMP102_ADDR);
    Wire.write(0x00); // Select temperature register
    Wire.endTransmission(false);

    Wire.requestFrom(TMP102_ADDR, 2);  // Request 2 bytes from TMP102
    if (Wire.available() == 2) {
        int16_t rawData = (Wire.read() << 8) | Wire.read();  // Read 16-bit data

        rawData = rawData >> 4;  // Shift right to get the 12-bit temperature value

        // Handle negative temperatures
        if (rawData & 0x800) {
            rawData |= 0xF000;  // Extend sign for negative temperature
        }

        float temperature = rawData * 0.0625;  // Convert to Celsius

        // Print temperature to Serial Monitor
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");

        // Upload to ThingSpeak
        ThingSpeak.setField(1, temperature);  // Set the temperature value in field 1
        int httpCode = ThingSpeak.writeFields(channelID, apiKey); // Send data to ThingSpeak

        if (httpCode == 200) {
            Serial.println("Data uploaded to ThingSpeak successfully");
        } else {
            Serial.print("Error uploading data. HTTP Code: ");
            Serial.println(httpCode);
        }
    } else {
        Serial.println("Failed to read temperature.");
    }

    delay(15000); // Upload data every 20 seconds (minimum interval for ThingSpeak)
}
