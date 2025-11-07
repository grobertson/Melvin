/**
 * Configuration Header for Abstract Member Controller
 * 
 * This file contains all configuration constants and definitions
 * that can be customized for specific member controller implementations.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Network Configuration
#define WIFI_SSID "Melvin_Control_Network"
#define WIFI_PASSWORD "MelvinController2025"
#define MASTER_CONTROLLER_IP "192.168.4.1"
#define MASTER_CONTROLLER_PORT 80

// Device Configuration - GPS Interface Controller
#define DEVICE_TYPE 2                    // 2=sensor_input
#define DEVICE_NAME "GPS Interface Controller"
#define DEVICE_ID "GPS_001"

// Timing Configuration
#define HEARTBEAT_INTERVAL_MS 25000      // Send heartbeat every 25 seconds (master timeout is 30s)
#define CONTROL_UPDATE_INTERVAL_MS 100   // Check for control changes every 100ms
#define WIFI_CONNECT_TIMEOUT_MS 10000    // WiFi connection timeout
#define HTTP_REQUEST_TIMEOUT_MS 5000     // HTTP request timeout

// Control Configuration
#define MAX_CONTROLS 16                  // Maximum number of controls this device can manage
#define CONTROL_BASE_ADDRESS 3000        // Base address for GPS controls

// GPS Configuration
#define GPS_SERIAL_BAUD 9600             // GPS module baud rate
#define GPS_UPDATE_INTERVAL_MS 1000      // Send to master every 1 second
#define GPS_SERIAL_TIMEOUT_MS 100        // GPS serial timeout
#define NMEA_BUFFER_SIZE 256             // NMEA sentence buffer size
#define GPS_TEST_MODE 1                  // Enable test mode (1) or use real GPS (0)
#define GPS_TEST_INTERVAL_MS 2000        // Test data generation interval

// Control value constants (matching master controller)
#define CONTROL_VALUE_OFF 0
#define CONTROL_VALUE_MAX 32768
#define CONTROL_VALUE_BOOLEAN_ON 1

// Control Types (matching master controller)
enum ControlType {
    CONTROL_BOOLEAN = 0,    // Toggle switches, buttons (0 or 1)
    CONTROL_CONTINUOUS,     // Potentiometers, sliders (0-32768)
    CONTROL_ROTARY,         // Rotary encoders (0-32768 with wrap)
    CONTROL_JOYSTICK_2AXIS, // 2-axis joystick (two 0-32768 values)
    CONTROL_RGB_GROUP       // RGB group (three 0-32768 values)
};

// Device Types (matching master controller)
enum DeviceType {
    DEVICE_VEHICLE_INPUT = 0,
    DEVICE_HUMAN_INTERFACE,
    DEVICE_SENSOR_INPUT,
    DEVICE_EXTERNAL_LIGHT,
    DEVICE_CABIN_LIGHT,
    DEVICE_AUX_DEVICE
};

// GPIO Pin Assignments - GPS Controller Specific
// GPS Serial Communication
#define GPS_RX_PIN 16               // ESP32 RX (connects to GPS TX)
#define GPS_TX_PIN 17               // ESP32 TX (connects to GPS RX)

// Status LEDs
#define PIN_GPS_FIX_LED 19          // LED indicating GPS fix status
#define PIN_DATA_LED 21             // LED showing GPS data activity
#define PIN_ERROR_LED 4             // LED indicating errors/timeouts
#define PIN_STATUS_LED 2            // LED showing system status

// Test Mode Control
#define PIN_TEST_MODE 22            // Switch for test mode (LOW = test mode)
#define GPS_TX_PIN 17               // ESP32 TX (connects to GPS RX)
#define GPS_BAUD_RATE 9600          // GPS module baud rate

// Status LEDs
#define PIN_STATUS_LED 2            // General status LED
#define GPS_STATUS_LED_PIN 19       // GPS fix status LED
#define DATA_LED_PIN 21             // Data activity LED
#define ERROR_LED_PIN 4             // Error indicator LED

// Test mode toggle
#define TEST_MODE_SWITCH_PIN 22     // Switch to toggle test mode

// GPS Configuration
#define GPS_TIMEOUT 5000            // GPS timeout in milliseconds
#define TEST_DATA_INTERVAL 2000     // Test data generation interval

// ADC Configuration
#define ADC_RESOLUTION 12               // 12-bit ADC resolution
#define ADC_MAX_VALUE 4095              // Maximum ADC reading (2^12 - 1)
#define ADC_SAMPLES 4                   // Number of samples to average for stability

// Debug Configuration
#define DEBUG_SERIAL_ENABLED 1          // Enable serial debug output
#define DEBUG_VERBOSE 0                 // Enable verbose debug output

// Status LED Patterns
#define LED_PATTERN_CONNECTING 250      // Fast blink while connecting
#define LED_PATTERN_REGISTERED 1000     // Slow blink when registered
#define LED_PATTERN_ERROR 100           // Very fast blink on error

#endif // CONFIG_H