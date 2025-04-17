#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <ESP_Mail_Client.h>

#define TMP102_ADDR 0x48  // Default I2C address for TMP102

// Wi-Fi Credentials (Replace with yours)
const char* ssid = "Redmi 10";
const char* password = "3h4xhjeh";

// Email Configuration (Use App Password for Gmail)
#define emailSenderAccount    "industrialcoldchain@gmail.com"
#define emailSenderPassword   "qvfaicvckgfzcxos"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "[ALERT] ESP32-S3 Temperature"

// Default Settings
String recipientEmail = "mailtovignesh19@gmail.com";  // Default recipient
bool emailAlertsEnabled = true;                      // Alerts ON by default
float tempThreshold = 25.00;                          // Default threshold (°C)

// HTML Web Page (Same as before)
const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head>
  <title>Email Notification with Temperature</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h2>TMP102 Temperature</h2>
  <h3>%TEMPERATURE% C</h3>
  <h2>ESP Email Notification</h2>
  <form action="/get">
    Email Address <input type="email" name="email_input" value="%EMAIL_INPUT%" required><br>
    Enable Email Notification <input type="checkbox" name="enable_email_input" value="true" %ENABLE_EMAIL%><br>
    Temperature Threshold <input type="number" step="0.1" name="threshold_input" value="%THRESHOLD%" required><br>
    <input type="submit" value="Submit">
  </form></body></html>)rawliteral";

AsyncWebServer server(80);
SMTPSession smtp;

// Variables for temperature monitoring
float lastValidTemperature = 0.0;
bool emailSent = false;
unsigned long lastEmailSentTime = 0;
const long emailCooldown = 300000;  // 5 minutes between emails (avoid spam)

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 19);  // SDA=21, SCL=19 (verify for ESP32-S3)

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

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

// ==================== MAIN LOOP ====================
void loop() {
  static unsigned long lastReadTime = 0;
  const long readInterval = 5000;  // Read every 5 sec

  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    float temperature = readTMP102Temperature();

    if (isValidTemperature(temperature)) {
      lastValidTemperature = temperature;
      Serial.println("Temperature: " + String(temperature) + "°C");

      // Check if alert should be sent
      checkTemperatureAlert(temperature);
    } else {
      Serial.println("⚠ Invalid temperature reading!");
    }
  }
  delay(100);
}

// ==================== HELPER FUNCTIONS ====================

// Reads temperature from TMP102
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

// Checks if temperature is within a realistic range
bool isValidTemperature(float temp) {
  return (temp > -50.0 && temp < 150.0);  // Reasonable range for most applications
}

// Checks if an email alert should be sent
void checkTemperatureAlert(float temp) {
  if (!emailAlertsEnabled) return;

  unsigned long currentTime = millis();
  bool isAboveThreshold = (temp > tempThreshold);

  if (isAboveThreshold && !emailSent) {
    if (currentTime - lastEmailSentTime >= emailCooldown) {
      String message = "🚨 Temperature ALERT: " + String(temp) + "°C (Above " + String(tempThreshold) + "°C)";
      if (sendEmailNotification(message)) {
        Serial.println("Email sent: " + message);
        emailSent = true;
        lastEmailSentTime = currentTime;
      }
    }
  } 
  else if (!isAboveThreshold && emailSent) {
    String message = "✅ Temperature back to normal: " + String(temp) + "°C";
    if (sendEmailNotification(message)) {
      Serial.println("Email sent: " + message);
      emailSent = false;
      lastEmailSentTime = currentTime;
    }
  }
}

// Sends email via SMTP
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

// Web Server HTML Processor
String processor(const String& var) {
  if (var == "TEMPERATURE") return String(lastValidTemperature);
  else if (var == "EMAIL_INPUT") return recipientEmail;
  else if (var == "ENABLE_EMAIL") return emailAlertsEnabled ? "checked" : "";
  else if (var == "THRESHOLD") return String(tempThreshold);
  return String();
}