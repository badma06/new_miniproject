#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Replace with your actual credentials and ThingSpeak API key
const char* ssid = "Badma'";
const char* password = "8122322893";
const char* apiKey = " UDVBO3GMCLSWNR6Q";

// Define GPS UART pins — choose free GPIOs (safe for UART2)
#define GPS_RX 16  // GPS TX -> ESP32 16
#define GPS_TX 17  // GPS RX -> ESP32 17

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // Use UART2 (Serial2 on ESP32)

float latitude, longitude;

void setup() {
  Serial.begin(115200);  // USB serial monitor

  // Start GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Read and decode GPS data
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);

    uploadToThingSpeak(latitude, longitude);
  } else {
    Serial.println("Waiting for valid GPS fix...");
  }

  delay(15000);  // ThingSpeak limit is 15 seconds minimum
}

void uploadToThingSpeak(float lat, float lng) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "http://api.thingspeak.com/update?api_key=" + String(apiKey) +
                 "&field1=" + String(lat, 6) +
                 "&field2=" + String(lng, 6);

    http.begin(url);
    int httpCode = http.GET();
    http.end();

    if (httpCode > 0) {
      Serial.print("ThingSpeak response code: ");
      Serial.println(httpCode);
    } else {
      Serial.print("Error sending data: ");
      Serial.println(http.errorToString(httpCode));
    }
  } else {
    Serial.println("WiFi not connected.");
  }
}
