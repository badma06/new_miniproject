#include <TinyGPS++.h>
#include <WiFi.h>
#include <Wire.h>
#include <ThingSpeak.h>

// ---------- Configuration ----------
#define TMP102_ADDR 0x48        // TMP102 I2C address
#define RELAY_PIN 4             // GPIO for relay control
#define GPS_RX 16               // GPS TX -> GPIO16
#define GPS_TX 17               // Optional

const float HIGH_TEMP_THRESHOLD = 32.0;
const float LOW_TEMP_THRESHOLD = 30.0;

const char* ssid = "Badma'";
const char* password = "8122322893";

unsigned long channelID = 22880817;
const char* apiKey = "UDVBO3GMCLSWNR6Q";  // ThingSpeak Write API Key

// ---------- Objects ----------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);     // UART2 for GPS
WiFiClient client;

// ---------- Variables ----------
float temperature = 0.0;
float latitude = 0.0;
float longitude = 0.0;
int peltierStatus = 0;           // 1 = ON, 0 = OFF

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);  // TMP102 I2C (SDA=21, SCL=19)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Peltier initially OFF (relay OFF)

  // Start GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Connect Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println("IP address: " + WiFi.localIP().toString());

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  readTemperature();
  readGPS();

  // Upload to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.setField(1, temperature);
    ThingSpeak.setField(2, peltierStatus);
    ThingSpeak.setField(3, latitude);
    ThingSpeak.setField(4, longitude);

    int result = ThingSpeak.writeFields(channelID, apiKey);

    if (result == 200) {
      Serial.println("Data uploaded successfully.");
    } else {
      Serial.println("Upload failed. HTTP error: " + String(result));
    }
  } else {
    Serial.println("WiFi not connected.");
  }

  delay(15000);  // 15s delay due to ThingSpeak limit
}

void readTemperature() {
  Wire.beginTransmission(TMP102_ADDR);
  Wire.write(0x00);  // Select temperature register
  Wire.endTransmission(false);
  Wire.requestFrom(TMP102_ADDR, 2);

  if (Wire.available() == 2) {
    int16_t rawData = (Wire.read() << 8) | Wire.read();
    rawData >>= 4;
    if (rawData & 0x800) rawData |= 0xF000;  // Handle negative temps
    temperature = rawData * 0.0625;

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Peltier control logic
    if (temperature >= HIGH_TEMP_THRESHOLD) {
      if (digitalRead(RELAY_PIN) == HIGH) {
        digitalWrite(RELAY_PIN, LOW);  // Peltier ON
        Serial.println("Peltier ON");
      }
    } else if (temperature <= LOW_TEMP_THRESHOLD) {
      if (digitalRead(RELAY_PIN) == LOW) {
        digitalWrite(RELAY_PIN, HIGH);  // Peltier OFF
        Serial.println("Peltier OFF");
      }
    }

    peltierStatus = (digitalRead(RELAY_PIN) == LOW) ? 1 : 0;
  } else {
    Serial.println("Failed to read temperature.");
  }
}

void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
  } else {
    Serial.println("Waiting for valid GPS fix...");
  }
}
