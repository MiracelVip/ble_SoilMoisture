#include "Arduino.h"
#include "NimBLEDevice.h"
#include "esp_sleep.h"

// Define the sensor GPIO pin
const int sensorPin = 33;

// Initialize global variables for sensor reading
const int numReadings = 5;
int readIndex = 0;
long total = 0;
int average = 0;

// Define UUID and Object ID for BLE advertisement
#define BTHOME_UUID "FC-D2"
#define OBJECT_ID_MOISTURE 0x14  // Object ID for soil moisture

/**
 * Calculate the moisture percentage from sensor value.
 */
int calculateMoisturePercentage(int value) {
    const int airValue = 3616;  // Air value representing 0% moisture
    const int waterValue = 1836; // Water value representing 100% moisture
    int percentage = (value - airValue) * 100 / (waterValue - airValue);
    Serial.print("calculateMoisturePercentage: ");
    Serial.println(percentage);
    return max(0, min(percentage, 100));  // Ensure percentage is between 0 and 100
}

/**
 * Send out the new value with BLE.
 */
void updateAdvertising(int moistureLevel) {
    uint16_t uuid = 0xFCD2; // BTHome UUID
    uint8_t flags = 0x06;   // General discoverability, BR/EDR not supported

    // Ensure moisture level is within the 0-100 range
    uint16_t encodedMoisture = static_cast<uint16_t>(max(0, min(moistureLevel, 100)) * 100); // Scale to 0.01

    // Prepare advertising payload
    uint8_t advPayload[] = {
        0x02, 0x01, flags,                             // Flags (2 bytes + flags)
        0x03, 0x03, static_cast<uint8_t>(uuid),        // 16-bit UUID
        static_cast<uint8_t>(uuid >> 8),
        0x08, 0x16, static_cast<uint8_t>(uuid),        // Service Data (including the BTHome data)
        static_cast<uint8_t>(uuid >> 8),
        0x40, OBJECT_ID_MOISTURE,
        static_cast<uint8_t>(encodedMoisture & 0xFF),  // Low byte of moisture
        static_cast<uint8_t>(encodedMoisture >> 8),    // High byte of moisture
        0x0E, 0x09, 'D', 'I', 'Y', '-', 's', 'e',     // Local Name "DIY-sensor"
        'n', 's', 'o', 'r'
    };

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->stop();
    NimBLEAdvertisementData advertisementData;
    advertisementData.addData(std::string((char*)advPayload, sizeof(advPayload)));
    pAdvertising->setAdvertisementData(advertisementData);
    pAdvertising->start();
}

/**
 * Create an average reading from the sensor.
 */
int readSensorAvg(int readings, int wait) {
    total = 0;
    average = 0;
    pinMode(sensorPin, INPUT_PULLUP);
    for (int i = 1; i <= readings + 1; i++) {
        int currentReading = analogRead(sensorPin);
        total += currentReading;
        average = total / i;
        delay(wait);
        Serial.print("readSensorAvg: ");
        Serial.println(average);
    }
    return average;
}

void setup() {

    Serial.begin(115200);

    NimBLEDevice::init("");
    String macAddress = NimBLEDevice::getAddress().toString().c_str();
    String deviceName = "Sensor-" + macAddress;
    NimBLEDevice::init(deviceName.c_str());

    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();

    Serial.println("Discarding initial sensor values");
    readSensorAvg(5, 10);
    Serial.println("Collecting real sensor values");
    updateAdvertising(calculateMoisturePercentage(readSensorAvg(10, 100)));
    delay (5000);
    Serial.println("Entering deep sleep mode...");
    //esp_sleep_enable_timer_wakeup(1200 * 1000000);  // Set deep sleep to wake up after 30 seconds
    //esp_deep_sleep_start();
}

void loop() {
    // The loop does not execute because the ESP enters deep sleep in setup()
    Serial.println ("alive");
    delay(1000);
}
