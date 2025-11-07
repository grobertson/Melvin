/**
 * Configuration Header for OBD-II Interface Controller
 * 
 * This file contains all configuration constants and definitions
 * specific to the OBD-II interface member controller.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Network Configuration
#define WIFI_SSID "Melvin_Control_Network"
#define WIFI_PASSWORD "MelvinController2025"
#define MASTER_CONTROLLER_IP "192.168.4.1"
#define MASTER_CONTROLLER_PORT 80

// Device Configuration - OBD-II Interface Specific
#define DEVICE_TYPE 6                           // DEVICE_ODBII_INTERFACE
#define DEVICE_NAME "OBD-II Interface"
#define DEVICE_ID "ODBII_001"

// Timing Configuration
#define HEARTBEAT_INTERVAL_MS 25000             // Send heartbeat every 25 seconds
#define CONTROL_UPDATE_INTERVAL_MS 100          // Check for control changes every 100ms
#define WIFI_CONNECT_TIMEOUT_MS 10000           // WiFi connection timeout
#define HTTP_REQUEST_TIMEOUT_MS 5000            // HTTP request timeout

// Control Configuration
#define MAX_CONTROLS 16                         // Maximum number of controls
#define CONTROL_BASE_ADDRESS 5000               // Base address for OBD-II controls

// Control value constants
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
    DEVICE_AUX_DEVICE,
    DEVICE_ODBII_INTERFACE
};

// OBD-II Specific Configuration
// Bluetooth OBD-II Adapter Settings
#define ODBII_BLUETOOTH_NAME "OBDII"            // Default adapter name to search for
#define ODBII_BLUETOOTH_PIN "1234"              // Default pairing PIN
#define ODBII_BAUD_RATE 38400                   // ELM327 default baud rate
#define ODBII_CONNECTION_TIMEOUT_MS 10000       // Bluetooth connection timeout

// Data Update Intervals (milliseconds)
#define ODBII_FAST_UPDATE_INTERVAL 250          // Engine RPM, speed (4 Hz)
#define ODBII_MEDIUM_UPDATE_INTERVAL 1000       // Temperatures, pressures (1 Hz)
#define ODBII_SLOW_UPDATE_INTERVAL 5000         // Fuel level, diagnostics (0.2 Hz)

// Data Filtering and Processing
#define MAX_ODBII_PARAMETERS 32                 // Maximum trackable parameters
#define ENABLE_DTC_MONITORING true              // Monitor diagnostic trouble codes
#define ENABLE_FREEZE_FRAME true                // Capture freeze frame data
#define ODBII_RESPONSE_TIMEOUT_MS 2000          // Timeout for OBD-II responses
#define ODBII_MAX_RETRIES 3                     // Maximum retries for failed commands

// OBD-II Parameter Update Groups
#define ODBII_UPDATE_GROUP_FAST 0               // High frequency updates
#define ODBII_UPDATE_GROUP_MEDIUM 1             // Medium frequency updates  
#define ODBII_UPDATE_GROUP_SLOW 2               // Low frequency updates

// Test Mode Configuration
#define TEST_MODE_DEFAULT_STATE false           // Default test mode state
#define TEST_RPM_IDLE 800                       // Simulated idle RPM
#define TEST_RPM_MAX 6000                       // Maximum simulated RPM
#define TEST_SPEED_MAX 120                      // Maximum simulated speed (km/h)
#define TEST_TEMP_NORMAL 90                     // Normal operating temperature (°C)

// GPIO Pin Assignments (for status indicators)
#define PIN_STATUS_LED 2                        // Status LED (built-in)
#define PIN_ERROR_LED 4                         // Error indicator LED
#define PIN_BLUETOOTH_LED 5                     // Bluetooth connection LED
#define PIN_TEST_MODE_BUTTON 12                 // Test mode toggle button (optional)

// Status LED Patterns
#define LED_PATTERN_CONNECTING 250              // Fast blink while connecting
#define LED_PATTERN_REGISTERED 1000             // Slow blink when registered
#define LED_PATTERN_ERROR 100                   // Very fast blink on error
#define LED_PATTERN_BLUETOOTH_CONNECTED 2000    // Very slow blink when BT connected

// Debug Configuration
#define DEBUG_SERIAL_ENABLED 1                  // Enable serial debug output
#define DEBUG_VERBOSE 1                         // Enable verbose debug output
#define ODBII_DEBUG_ENABLED 1                   // Enable OBD-II specific debugging

// Error Recovery Configuration
#define BLUETOOTH_RECONNECT_INTERVAL_MS 30000   // Retry Bluetooth connection every 30s
#define ODBII_INIT_RETRY_INTERVAL_MS 5000       // Retry OBD-II initialization every 5s
#define MAX_CONSECUTIVE_ERRORS 5                // Max errors before reset attempt

#endif // CONFIG_H