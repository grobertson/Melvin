#ifndef MCONTROL_HID_CONFIG_H
#define MCONTROL_HID_CONFIG_H

// Note: Control/Device type enums are now defined in MelvinMemberController library

// ============================================================================
// DEVICE IDENTIFICATION
// ============================================================================
#define DEVICE_ID "HID_001"
#define DEVICE_TYPE "HumanInterface"
#define DEVICE_DESCRIPTION "Human Interface Device - Switches, Buttons, Encoders, Joysticks"

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================
#define WIFI_SSID "Melvin_Control_Network"
#define WIFI_PASSWORD "MelvinController2025"
#define MASTER_CONTROLLER_IP "192.168.4.1"
#define REGISTRATION_URL "http://192.168.4.1/register"

// WiFi connection retry configuration
#define WIFI_FAST_RETRY_COUNT 5
#define WIFI_FAST_RETRY_DELAY_MS 2000
#define WIFI_SLOW_RETRY_INTERVAL_MS 300000  // 5 minutes

// Registration retry configuration
#define REGISTRATION_RETRY_COUNT 3
#define REGISTRATION_RETRY_DELAY_MS 5000

// ============================================================================
// MQTT CONFIGURATION
// ============================================================================
#define MQTT_ENABLED true
#define MQTT_BROKER_HOST "192.168.4.1"
#define MQTT_BROKER_PORT 1883

#define MQTT_TOPIC_PREFIX "melvin"
#define MQTT_TOPIC_DEVICES "melvin/devices"
#define MQTT_TOPIC_CONTROLS "melvin/controls"
#define MQTT_TOPIC_EVENTS "melvin/events"

// ============================================================================
// CONTROL ID ALLOCATION
// ============================================================================
// HID Controllers use range: 1000-1999 (per CONTROL_ID_ALLOCATION.md)
#define HID_CONTROL_BASE 1000

// Control ID assignments
#define CTRL_TOGGLE_SWITCH_1    1001
#define CTRL_TOGGLE_SWITCH_2    1002
#define CTRL_TOGGLE_SWITCH_3    1003
#define CTRL_TOGGLE_SWITCH_4    1004

#define CTRL_MOMENTARY_BTN_1    1010
#define CTRL_MOMENTARY_BTN_2    1011
#define CTRL_MOMENTARY_BTN_3    1012
#define CTRL_MOMENTARY_BTN_4    1013

#define CTRL_POTENTIOMETER_1    1020
#define CTRL_POTENTIOMETER_2    1021
#define CTRL_POTENTIOMETER_3    1022

#define CTRL_ROTARY_ENCODER_1   1030
#define CTRL_ROTARY_ENCODER_2   1031

#define CTRL_JOYSTICK_1         1040  // 2-axis joystick (X and Y)
#define CTRL_JOYSTICK_2         1041

// ============================================================================
// GPIO PIN ASSIGNMENTS
// ============================================================================

// Toggle Switches (with pull-up resistors)
#define PIN_TOGGLE_SWITCH_1     GPIO_NUM_12
#define PIN_TOGGLE_SWITCH_2     GPIO_NUM_13
#define PIN_TOGGLE_SWITCH_3     GPIO_NUM_14
#define PIN_TOGGLE_SWITCH_4     GPIO_NUM_15

// Momentary Buttons (with pull-up resistors)
#define PIN_MOMENTARY_BTN_1     GPIO_NUM_16
#define PIN_MOMENTARY_BTN_2     GPIO_NUM_17
#define PIN_MOMENTARY_BTN_3     GPIO_NUM_18
#define PIN_MOMENTARY_BTN_4     GPIO_NUM_19

// Potentiometers (analog inputs)
#define PIN_POTENTIOMETER_1     GPIO_NUM_32  // ADC1_CH4
#define PIN_POTENTIOMETER_2     GPIO_NUM_33  // ADC1_CH5
#define PIN_POTENTIOMETER_3     GPIO_NUM_34  // ADC1_CH6

// Rotary Encoders (2 pins each + optional button)
#define PIN_ENCODER_1_A         GPIO_NUM_25
#define PIN_ENCODER_1_B         GPIO_NUM_26
#define PIN_ENCODER_1_BTN       GPIO_NUM_27  // Optional push button

#define PIN_ENCODER_2_A         GPIO_NUM_21
#define PIN_ENCODER_2_B         GPIO_NUM_22
#define PIN_ENCODER_2_BTN       GPIO_NUM_23  // Optional push button

// Joysticks (2 analog axes each + optional button)
#define PIN_JOYSTICK_1_X        GPIO_NUM_35  // ADC1_CH7
#define PIN_JOYSTICK_1_Y        GPIO_NUM_36  // ADC1_CH0
#define PIN_JOYSTICK_1_BTN      GPIO_NUM_4   // Optional push button

#define PIN_JOYSTICK_2_X        GPIO_NUM_39  // ADC1_CH3
#define PIN_JOYSTICK_2_Y        GPIO_NUM_34  // ADC1_CH6 (shared with POT3)
#define PIN_JOYSTICK_2_BTN      GPIO_NUM_5   // Optional push button

// Status LED
#define PIN_STATUS_LED          GPIO_NUM_2   // Built-in LED on most ESP32 boards

// ============================================================================
// INPUT PROCESSING CONFIGURATION
// ============================================================================

// Debounce settings
#define DEBOUNCE_DELAY_MS       50    // Debounce time for switches/buttons
#define ENCODER_DEBOUNCE_MS     2     // Faster for rotary encoders

// Analog input settings
#define ADC_RESOLUTION          12    // 12-bit ADC (0-4095)
#define ADC_MAX_VALUE           4095  // Maximum ADC value
#define ADC_SAMPLES             8     // Number of samples to average
#define ADC_DEADBAND            50    // Ignore changes smaller than this

// Potentiometer settings
#define POT_PUBLISH_THRESHOLD   100   // Minimum change to trigger publish (in ADC units)
#define POT_UPDATE_INTERVAL_MS  100   // Minimum time between updates

// Joystick settings
#define JOYSTICK_CENTER         2048  // Center position (12-bit)
#define JOYSTICK_DEADZONE       200   // Deadzone around center
#define JOYSTICK_MIN            0     // Minimum value
#define JOYSTICK_MAX            4095  // Maximum value
#define JOYSTICK_UPDATE_MS      50    // Update rate for joysticks

// Rotary encoder settings
#define ENCODER_PULSES_PER_REV  20    // Pulses per full rotation
#define ENCODER_WRAP_AROUND     true  // Wrap from max to min
#define ENCODER_MIN_VALUE       0
#define ENCODER_MAX_VALUE       32768

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define STATUS_UPDATE_INTERVAL_MS    30000  // Send status every 30 seconds
#define CONTROL_SCAN_INTERVAL_MS     10     // Scan inputs every 10ms
#define MQTT_PUBLISH_BATCH_INTERVAL  50     // Batch publish changed controls

// ============================================================================
// FEATURE FLAGS
// ============================================================================
#define ENABLE_TOGGLE_SWITCHES   true
#define ENABLE_MOMENTARY_BUTTONS true
#define ENABLE_POTENTIOMETERS    true
#define ENABLE_ROTARY_ENCODERS   true
#define ENABLE_JOYSTICKS         true
#define ENABLE_STATUS_LED        true

// Advanced features
#define ENABLE_ENCODER_ACCELERATION false  // Speed-dependent encoder steps
#define ENABLE_KEYBOARD_EMULATION   false  // Future: USB HID keyboard
#define ENABLE_MULTI_PRESS_DETECT   false  // Detect double/triple press

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
#define DEBUG_PRINT_ENABLED     true
#define DEBUG_PRINT_INPUTS      false  // Print all input changes
#define DEBUG_PRINT_ADC_RAW     false  // Print raw ADC values
#define DEBUG_PRINT_ENCODER     false  // Print encoder steps

#endif // MCONTROL_HID_CONFIG_H
