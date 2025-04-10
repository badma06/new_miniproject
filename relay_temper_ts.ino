#include <Wire.h>
#include <WiFi.h>
#include <ThingSpeak.h>

#define TMP102_ADDR 0x48  // TMP102 default I2C address
#define RELAY_PIN 4       // GPIO pin for relay control

// Temperature thresholds
const float HIGH_TEMP_THRESHOLD = 32.0;
const float LOW_TEMP_THRESHOLD = 30.0;

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ThingSpeak settings
unsigned long channelID = 2876792;
const char* apiKey = "YOUR_API_KEY";

WiFiClient client;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);  // SDA -> GPIO21, SCL -> GPIO19
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Initialize relay OFF

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
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

    // Control Peltier module via relay
    if (temperature >= HIGH_TEMP_THRESHOLD) {
      if (digitalRead(RELAY_PIN) == HIGH) {
        digitalWrite(RELAY_PIN, LOW); // Turn relay ON (Peltier ON)
        Serial.println("Peltier ON");
      }
    } else if (temperature <= LOW_TEMP_THRESHOLD) {
      if (digitalRead(RELAY_PIN) == LOW) {
        digitalWrite(RELAY_PIN, HIGH);  // Turn relay OFF (Peltier OFF)
        Serial.println("Peltier OFF");
      }
    }

    // Upload to ThingSpeak
    if (WiFi.status() == WL_CONNECTED) {
      ThingSpeak.setField(1, temperature); // Field 1 for temperature
      ThingSpeak.setField(2, digitalRead(RELAY_PIN) == LOW ? 1 : 0); // Field 2 for Peltier status (1=ON, 0=OFF)
      
      int response = ThingSpeak.writeFields(channelID, apiKey);
      if (response == 200) {
        Serial.println("Data uploaded to ThingSpeak successfully");
      } else {
        Serial.println("Problem uploading to ThingSpeak. HTTP error code " + String(response));
      }
    } else {
      Serial.println("WiFi disconnected");
    }

  } else {
    Serial.println("Failed to read temperature.");
  }

  delay(15000); // Wait 30 seconds between updates (ThingSpeak free account limit)
}
