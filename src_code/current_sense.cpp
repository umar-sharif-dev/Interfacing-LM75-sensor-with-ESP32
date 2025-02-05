/*
 * v1.0
 * INA219 Current Measurement Utility for Arduino
 * 
 * This code interfaces the INA219 current sensor with an Arduino microcontroller and demonstrates how the system 
 * measures the current consumption of a connected load. The measured current is then displayed on the serial monitor.
 * 
 * **Functionality:**
 * - The INA219 sensor is initialized at the start, and the system continuously reads the current consumption in milliamps.
 * - The current reading is updated every second and displayed on the serial monitor.
 * 
 * **Key Considerations:**
 * - **I2C Communication**: The INA219 communicates over I2C. Make sure the SDA and SCL lines are connected correctly to the Arduino.
 * - **Serial Communication**: The Arduino sends data over the serial monitor at a baud rate of 115200 to display the current readings.
 * 
 * **Power Efficiency:**
 * - This example is designed to monitor the current but does not implement any low-power features. However, for low-power applications, you can integrate sleep modes for the Arduino and sensor when not actively measuring the current.
 * - For more advanced power management, you can implement periodic readings with deep sleep in between to minimize overall power consumption.
 * 
 * **Next Steps:**
 * - You can modify the code to take voltage readings or calculate power by using the INA219's `getBusVoltage_V()` and `getPower_mW()` functions.
 * - The reading frequency can be adjusted or optimized based on the application requirements.
 * 
 * This code provides a simple utility for measuring and monitoring the current consumption in a circuit.
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219; // Create an INA219 object

void setup() {
    Serial.begin(115200); // Start serial communication with baud rate 115200
    
    // Initialize INA219
    if (!ina219.begin()) {  // Attempt to initialize the INA219 sensor
        Serial.println("Failed to find INA219 chip");  // If initialization fails, print error message
        while (1);  // Stay here forever if initialization fails
    }

    Serial.println("INA219 Current Sensor Initialized.");  // Successful initialization message
}

void loop() {
    float current_mA = ina219.getCurrent_mA();  // Read the current in milliamps

    Serial.print("Current: ");  // Print the label
    Serial.print(current_mA);  // Print the current measurement
    Serial.println(" mA");  // Print units

    delay(1000);  // Wait 1 second before taking the next reading
}
