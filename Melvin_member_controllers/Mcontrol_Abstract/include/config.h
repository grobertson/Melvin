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

// MQTT Configuration
#define MQTT_BROKER_IP MASTER_CONTROLLER_IP  // MQTT broker runs on master controller
#define MQTT_BROKER_PORT 1883                // Standard MQTT port
#define MQTT_CLIENT_ID_PREFIX "melvin_"      // Prefix for MQTT client IDs
#define MQTT_KEEPALIVE 60                    // MQTT keepalive interval (seconds)
#define MQTT_RECONNECT_INTERVAL 5000         // Time between reconnection attempts (ms)
#define MQTT_ENABLED 1                       // Enable MQTT functionality (set to 0 to disable)

// Device Configuration - CUSTOMIZE THESE FOR EACH CONTROLLER
#define DEVICE_TYPE 0                    // 0=vehicle_input, 1=human_interface, 2=sensor_input, 3=external_light, 4=cabin_light, 5=aux_device
#define DEVICE_NAME "Abstract_Controller"
#define DEVICE_ID "ABSTRACT_001"

// Timing Configuration
#define HEARTBEAT_INTERVAL_MS 25000      // Send heartbeat every 25 seconds (master timeout is 30s)
#define CONTROL_UPDATE_INTERVAL_MS 100   // Check for control changes every 100ms
#define WIFI_CONNECT_TIMEOUT_MS 10000    // WiFi connection timeout
#define HTTP_REQUEST_TIMEOUT_MS 5000     // HTTP request timeout

// Connection Retry Configuration
#define WIFI_FAST_RETRY_COUNT 5          // Number of fast retry attempts
#define WIFI_FAST_RETRY_DELAY_MS 2000    // Delay between fast retries (2 seconds)
#define WIFI_SLOW_RETRY_INTERVAL_MS 300000 // Retry interval after fast attempts fail (5 minutes)
#define REGISTRATION_RETRY_COUNT 3       // Number of registration attempts
#define REGISTRATION_RETRY_DELAY_MS 3000 // Delay between registration retries (3 seconds)

// Control Configuration
#define MAX_CONTROLS 16                  // Maximum number of controls this device can manage
#define CONTROL_BASE_ADDRESS 1000        // Base address for controls (customize per device type)

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

// MQTT Topic Structure (matching master controller)
#define MQTT_TOPIC_ROOT "melvin"
#define MQTT_TOPIC_EVENTS "melvin/events"
#define MQTT_TOPIC_CONTROLS "melvin/controls"
#define MQTT_TOPIC_DEVICES "melvin/devices"
#define MQTT_TOPIC_STATUS "melvin/status"

// MQTT Event Types for control routing
enum MQTTRoutingMode {
    MQTT_ROUTE_NONE = 0,        // No automatic routing
    MQTT_ROUTE_DIRECT,          // Route control changes directly to outputs
    MQTT_ROUTE_MAPPED,          // Route using control ID mapping
    MQTT_ROUTE_CUSTOM           // Custom routing via callback
};

// GPIO Pin Assignments - CUSTOMIZE FOR EACH CONTROLLER
// Digital Input Pins
#define PIN_BUTTON_1 12
#define PIN_BUTTON_2 13
#define PIN_SWITCH_1 14
#define PIN_SWITCH_2 15

// Analog Input Pins (ESP32 GPIO numbers)
#define PIN_POT_1 36        // GPIO36 (ADC1_CH0)
#define PIN_POT_2 39        // GPIO39 (ADC1_CH3)  
#define PIN_JOYSTICK_X 34   // GPIO34 (ADC1_CH6)
#define PIN_JOYSTICK_Y 35   // GPIO35 (ADC1_CH7)

// Digital Output Pins (for local indicators, relays, etc.)
#define PIN_STATUS_LED 2
#define PIN_ERROR_LED 4
#define PIN_RELAY_1 16
#define PIN_RELAY_2 17

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