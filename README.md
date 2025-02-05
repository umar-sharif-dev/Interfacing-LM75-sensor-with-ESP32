# Interfacing LM75 Sensor with ESP32

## v1.0 - Overview
This project demonstrates how to interface the **LM75** temperature sensor with the **ESP32** microcontroller and measure current using the **INA219** sensor on **Arduino**. The system reads temperature data from the LM75 sensor and uses the ESP32 to handle power management based on the temperature readings. Additionally, the project features current sensing using the INA219 sensor to monitor current consumption in a circuit.

This repository includes the following:
- **ESP32 Code**: Code for reading temperature from the LM75 sensor and testing power modes.
- **Arduino Code**: Code for current measurement using the INA219 sensor.
- **Documentation**: Specifications and design documents.
- **Videos**: Test and demonstration videos showing the system in action.

## Project Structure


## Features

- **Temperature Measurement with LM75**: The LM75 sensor is read using I2C communication, and the temperature is output in both **Celsius** and **Fahrenheit**.
- **Power Modes on ESP32**: The ESP32 enters different power modes based on the temperature readings from the sensor:
  - **Active Mode**: Temperature exceeds 17°C
  - **Light Sleep Mode**: Temperature between 16°C and 17°C
  - **Deep Sleep Mode**: Temperature below 16°C
- **Low-Power Operation**: The system minimizes power consumption by using sleep modes whenever the device is not actively reading the temperature or blinking the LED.
- **Current Sensing on Arduino**: Using the **INA219 current sensor** on an Arduino, the code measures and displays the current consumption in milliamps (mA).
- **Serial Output**: Both the ESP32 and Arduino send data to the Serial Monitor for real-time temperature and current monitoring.

## Getting Started

### Requirements
- **Hardware**:
  - **ESP32 Microcontroller** (for LM75 interfacing and power management)
  - **LM75 Temperature Sensor**
  - **Arduino Board** (for current sensing with INA219)
  - **INA219 Current Sensor**
  - **Jumper Wires** (for connections)
  - **LED** (optional, for visual feedback)

- **Software**:
  - **Arduino IDE** or any compatible C++ IDE
  - **Adafruit INA219 library** (for current measurement on Arduino)
  - **Wire library** (for I2C communication)

### Installation

1. Clone the repository to your local machine:

   ```bash
   git clone https://github.com/umar-sharif-dev/Interfacing-LM75-sensor-with-ESP32.git
