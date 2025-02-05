/*
 * v1.0
 * Test Scenario for Power Modes and Functionality
 * 
 * This code interfaces the LM75A temperature sensor with an ESP32 and demonstrates how the system behaves
 * in different power modes based on temperature thresholds. The thresholds for entering different power modes
 * are defined by parameters, making it easy to test different scenarios and modify them as required.
 * 
 * The power modes and their corresponding temperature thresholds:
 * - Active Mode: Triggered when the temperature exceeds 17°C.
 * - Light Sleep Mode: Triggered when the temperature is between 16°C and 17°C.
 * - Deep Sleep Mode: Triggered when the temperature falls below 16°C.
 * 
 * **Low Power Optimization:**
 * - **Deep Sleep** is used as much as possible to minimize power consumption. The ESP32 is kept in Deep Sleep by default and wakes up only when needed.
 * - The **RTC (Real-Time Clock) Timer** is used for scheduling periodic tasks such as temperature readings and UART transmission to ensure low-power operation.
 * - **LED Blinking** frequency has been reduced to only occur in Active Mode. The LED is turned off during Light and Deep Sleep Modes to save power.
 * - **Temperature Read Frequency**: The ESP32 wakes up periodically to read the LM75 sensor and send the data over UART. The wake-up frequency can be adjusted for your use case.
 * - **I2C Sensor**: Ensure that the LM75 sensor is in low-power mode when not actively being read to further minimize power consumption.
 * 
 * This code is designed to test overall functionality and is customizable to suit low-power application needs.
 * The thresholds can be easily adjusted for different test conditions or application requirements.
 */

#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>

#define LM75A_ADDRESS 0x48  // Default I2C address of LM75A
#define TEMPERATURE_REGISTER 0x00
#define LED 4  // Pin for LED
#define WAKEUP_TIME 10  // Wake-up time in seconds (adjustable for testing)

// Thresholds for temperature-based power modes (can be adjusted as needed)
#define ACTIVE_MODE_THRESHOLD 17  // Active mode if temperature is above 17°C
#define LIGHT_SLEEP_MODE_MIN 16  // Light sleep mode if temperature is between 16°C and 17°C
#define LIGHT_SLEEP_MODE_MAX 17  // Light sleep mode if temperature is between 16°C and 17°C
#define DEEP_SLEEP_MODE_THRESHOLD 16  // Deep sleep mode if temperature is below 16°C

hw_timer_t *My_timer = NULL;  // Hardware timer pointer
volatile bool blinkEnabled = false;  // Flag to control LED blinking
float temperature_in_fahrenheit;

// Timer interrupt function - toggles LED every second (1 Hz)
void IRAM_ATTR onTimer() {
  if (blinkEnabled) {
    digitalWrite(LED, !digitalRead(LED));  // Toggle LED state
  }
}

// Function to read temperature from LM75A
float readTemperature() {
  Wire.beginTransmission(LM75A_ADDRESS);
  Wire.write(TEMPERATURE_REGISTER);
  Wire.endTransmission(false);  // Restart condition for read operation

  Wire.requestFrom(LM75A_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    
    int16_t rawTemperature = (msb << 8) | lsb;
    rawTemperature >>= 5;  // Right-align 11-bit value
    
    return rawTemperature * 0.125;  // Convert to Celsius
  }
  return -1000;  // Error value if data is unavailable
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED, OUTPUT);

  // Initialize hardware timer (Optional: can be skipped for lower power)
  My_timer = timerBegin(1);  // Set frequency to 1 Hz (1 second period)
  timerAttachInterrupt(My_timer, &onTimer);  // Attach interrupt function
  timerAlarm(My_timer, 1000000, true, 0);  // Set alarm for 1 second

  Serial.println("Scanning for I2C devices....");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at address 0x");
      Serial.println(i, HEX);
    }
  }

  // Enter Deep Sleep by default, adjust as necessary
  esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000ULL);  // Wake up after 10 seconds
  esp_deep_sleep_start();
}

void loop() {
  float temperature_in_degrees = readTemperature();
  temperature_in_fahrenheit = temperature_in_degrees * 1.8 + 32;

  if (temperature_in_degrees == -1000) {
    Serial.println("Error while getting temperature");
    blinkEnabled = false;
    digitalWrite(LED, LOW);
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature_in_degrees);
    Serial.print("°C (");
    Serial.print(temperature_in_fahrenheit);
    Serial.println("°F)");

    // Deep Sleep Mode if temperature is below DEEP_SLEEP_MODE_THRESHOLD
    if (temperature_in_degrees < DEEP_SLEEP_MODE_THRESHOLD) {
      Serial.println("Temperature below 16°C. Entering deep sleep...");
      delay(1000);

      // Configure deep sleep
      esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000ULL);
      esp_deep_sleep_start();
    }
    // Light Sleep Mode if temperature is between LIGHT_SLEEP_MODE_MIN and LIGHT_SLEEP_MODE_MAX
    else if (temperature_in_degrees >= LIGHT_SLEEP_MODE_MIN && temperature_in_degrees < LIGHT_SLEEP_MODE_MAX) {
      Serial.println("Temperature between 16°C and 17°C. Entering light sleep...");
      delay(1000);
      esp_sleep_enable_timer_wakeup(WAKEUP_TIME * 1000000ULL);
      esp_light_sleep_start();
    }
    // Active Mode if temperature is above ACTIVE_MODE_THRESHOLD
    else {
      Serial.println("Temperature is normal. Running in active mode.");
      blinkEnabled = true;
    }
  }
  delay(2000);  // Can adjust or reduce to optimize wake-up rate
}
