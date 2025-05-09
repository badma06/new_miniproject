#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <ESP_Mail_Client.h>
#include <TinyGPS++.h>
#include <ThingSpeak.h>

// Hardware Configuration
#define TMP102_ADDR 0x48        // TMP102 I2C address
#define RELAY_PIN 18             // GPIO for relay control (Peltier)
#define GPS_RX 16               // GPS TX -> GPIO16
#define GPS_TX 17               // Optional GPS RX

// Wi-Fi Credentials
const char* ssid = "Redmi 10";
const char* password = "3h4xhjeh";

// Email Configuration
#define emailSenderAccount    "industrialcoldchain@gmail.com"
#define emailSenderPassword   "qvfaicvckgfzcxos"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "[ALERT] ESP32-S3 Temperature"

// ThingSpeak Configuration
unsigned long channelID = 2880817;
const char* apiKey = "UDVBO3GMCLSWNR6Q";  // ThingSpeak Write API Key

// Default Settings
String recipientEmail = "mailtovignesh19@gmail.com";  // Default recipient
bool emailAlertsEnabled = true;                      // Alerts ON by default
float tempThreshold = 33.00;                         // Default threshold (°C)
const float HIGH_TEMP_THRESHOLD = 33.00;              // Peltier ON threshold (Modified)
const float LOW_TEMP_THRESHOLD = 32.50;               // Peltier OFF threshold (Modified)

// HTML Web Page
const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head>
  <title>ESP32-S3 Monitoring System</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h2>System Monitoring</h2>
  <h3>Temperature: %TEMPERATURE% °C</h3>
  <h3>Peltier Status: %PELTIER_STATUS%</h3>
  <h3>GPS: %GPS_DATA%</h3>
  
  <h2>Notification Settings</h2>
  <form action="/get">
    Email Address <input type="email" name="email_input" value="%EMAIL_INPUT%" required><br>
    Enable Email Notification <input type="checkbox" name="enable_email_input" value="true" %ENABLE_EMAIL%><br>
    Alert Threshold (°C) <input type="number" step="0.1" name="threshold_input" value="%THRESHOLD%" required><br>
    <input type="submit" value="Submit">
  </form></body></html>)rawliteral";

// Global Objects
AsyncWebServer server(80);
SMTPSession smtp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);     // UART2 for GPS
WiFiClient client;

// Global Variables
float lastValidTemperature = 0.0;
float latitude = 0.0;
float longitude = 0.0;
int peltierStatus = 0;           // 1 = ON, 0 = OFF
bool emailSent = false;
unsigned long lastEmailSentTime = 0;
const long emailCooldown = 300000;  // 5 minutes between emails

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for TMP102
  Wire.begin(8, 9);  // SDA=21, SCL=19 (verify for ESP32-S3)
  
  // Initialize Peltier control
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Peltier initially OFF
  
  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Configure Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Update email recipient
    if (request->hasParam("email_input")) {
      recipientEmail = request->getParam("email_input")->value();
    }

    // Update email alerts status
    emailAlertsEnabled = request->hasParam("enable_email_input");

    // Update temperature threshold
    if (request->hasParam("threshold_input")) {
      tempThreshold = request->getParam("threshold_input")->value().toFloat();
    }

    Serial.println("Settings Updated:");
    Serial.println("Email: " + recipientEmail);
    Serial.println("Alerts: " + String(emailAlertsEnabled ? "ON" : "OFF"));
    Serial.println("Threshold: " + String(tempThreshold) + "°C");

    request->send(200, "text/html", 
      "Settings saved!<br><a href=\"/\">Return to Home</a>");
  });

  server.begin();
  MailClient.networkReconnect(true);  // Auto-reconnect SMTP
}

void loop() {
  static unsigned long lastReadTime = 0;
  static unsigned long lastUploadTime = 0;
  const long readInterval = 5000;    // Read every 5 sec
  const long uploadInterval = 15000; // Upload every 15 sec (ThingSpeak limit)

  unsigned long currentMillis = millis();
  digitalWrite(14,HIGH);
 // digitalWrite(13,HIGH);
 // digitalWrite(12,HIGH);
 // digitalWrite(11,HIGH);
  // Read sensors periodically
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    
    // Read temperature and control Peltier
    readTemperature();
    
    // Read GPS data
    readGPS();
  }

  // Upload to ThingSpeak periodically
  if (currentMillis - lastUploadTime >= uploadInterval) {
    lastUploadTime = currentMillis;
    uploadToThingSpeak();
  }

  delay(100);
}

// ==================== HELPER FUNCTIONS ====================

float readTMP102Temperature() {
  Wire.beginTransmission(TMP102_ADDR);
  Wire.write(0x00);  // Temp register
  if (Wire.endTransmission(false) != 0) {
    Serial.println("I2C Error: TMP102 not responding");
    return -999.0;  // Error value
  }

  Wire.requestFrom(TMP102_ADDR, 2);
  if (Wire.available() == 2) {
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
    rawTemp >>= 4;  // 12-bit precision
    if (rawTemp & 0x800) rawTemp |= 0xF000;  // Sign extend if negative
    return rawTemp * 0.0625;  // Convert to Celsius
  }
  return -999.0;  // Error value
}

void readTemperature() {
  float temperature = readTMP102Temperature();

  if (isValidTemperature(temperature)) {
    lastValidTemperature = temperature;
    Serial.println("Temperature: " + String(temperature) + "°C");

    // Peltier control logic
    if (temperature >= HIGH_TEMP_THRESHOLD) {
      digitalWrite(RELAY_PIN, LOW);  // Peltier ON
      peltierStatus = 1;
      Serial.println("Peltier ON");
    } else if (temperature <= LOW_TEMP_THRESHOLD) {
      digitalWrite(RELAY_PIN, HIGH);  // Peltier OFF
      peltierStatus = 0;
      Serial.println("Peltier OFF");
    }

    // Check if alert should be sent
    checkTemperatureAlert(temperature);
  } else {
    Serial.println("⚠ Invalid temperature reading!");
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
  }
}

void uploadToThingSpeak() {
  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.setField(1, lastValidTemperature);
    ThingSpeak.setField(2, peltierStatus);
    ThingSpeak.setField(3, latitude);
    ThingSpeak.setField(4, longitude);

    int result = ThingSpeak.writeFields(channelID, apiKey);

    if (result == 200) {
      Serial.println("Data uploaded to ThingSpeak successfully.");
    } else {
      Serial.println("ThingSpeak upload failed. HTTP error: " + String(result));
    }
  } else {
    Serial.println("WiFi not connected. Cannot upload to ThingSpeak.");
  }
}

bool isValidTemperature(float temp) {
  return (temp > -50.0 && temp < 150.0);  // Reasonable range for most applications
}

void checkTemperatureAlert(float temp) {
  if (!emailAlertsEnabled) return;

  unsigned long currentTime = millis();
  bool isAboveThreshold = (temp > tempThreshold);
  bool isBelowLowThreshold = (temp <= LOW_TEMP_THRESHOLD);

  // Alert when above the threshold
  if (isAboveThreshold && !emailSent) {
    if (currentTime - lastEmailSentTime >= emailCooldown) {
      String message = "🚨 Temperature ALERT: " + String(temp) + "°C (Above " + String(tempThreshold) + "°C)\n";
      message += "Peltier Status: " + String(peltierStatus ? "ON" : "OFF") + "\n";
      message += "GPS Location: " + String(latitude, 6) + ", " + String(longitude, 6);
      
      if (sendEmailNotification(message)) {
        Serial.println("Email sent: " + message);
        emailSent = true;
        lastEmailSentTime = currentTime;
      }
    }
  }
  // Recovery when below the LOW_TEMP_THRESHOLD
  else if (isBelowLowThreshold && emailSent) {
    String message = "✅ Temperature back to normal: " + String(temp) + "°C\n";
    message += "Peltier Status: " + String(peltierStatus ? "ON" : "OFF") + "\n";
    message += "GPS Location: " + String(latitude, 6) + ", " + String(longitude, 6);
    
    if (sendEmailNotification(message)) {
      Serial.println("Recovery email sent: " + message);
      emailSent = false;
      lastEmailSentTime = currentTime;
    }
  }
}

bool sendEmailNotification(String message) {
  ESP_Mail_Session session;
  session.server.host_name = smtpServer;
  session.server.port = smtpServerPort;
  session.login.email = emailSenderAccount;
  session.login.password = emailSenderPassword;

  SMTP_Message mail;
  mail.sender.name = "ESP32-S3 Alert";
  mail.sender.email = emailSenderAccount;
  mail.subject = emailSubject;
  mail.addRecipient("Recipient", recipientEmail);
  mail.text.content = message.c_str();

  if (!smtp.connect(&session)) {
    Serial.println("SMTP Connection Error: " + smtp.errorReason());
    return false;
  }

  if (!MailClient.sendMail(&smtp, &mail)) {
    Serial.println("Email Error: " + smtp.errorReason());
    smtp.closeSession();
    return false;
  }

  smtp.closeSession();
  return true;
}

String processor(const String& var) {
  if (var == "TEMPERATURE") return String(lastValidTemperature);
  else if (var == "PELTIER_STATUS") return peltierStatus ? "ON" : "OFF";
  else if (var == "GPS_DATA") {
    if (latitude == 0.0 && longitude == 0.0) return "No GPS fix";
    return String(latitude, 6) + ", " + String(longitude, 6);
  }
  else if (var == "EMAIL_INPUT") return recipientEmail;
  else if (var == "ENABLE_EMAIL") return emailAlertsEnabled ? "checked" : "";
  else if (var == "THRESHOLD") return String(tempThreshold);
  return String();
}
