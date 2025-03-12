#include <Wire.h>

#define TMP102_ADDR 0x48  // TMP102 default I2C address

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 19); // SDA -> GPIO21, SCL -> GPIO19 (or use GPIO22 for SCL)
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
    } else {
        Serial.println("Failed to read temperature.");
    }

    delay(1000); // Read temperature every second
}
