/**
 * Config Stub for MelvinMemberController Library
 * 
 * This file provides default values for configuration constants.
 * The project using this library should define all these constants in their own config.h
 * which will be included first and override these defaults.
 */

#ifndef MELVIN_CONFIG_STUB_H
#define MELVIN_CONFIG_STUB_H

// If project hasn't defined these, provide stubs to allow compilation
// In practice, these will cause linker errors if not properly defined by project

#ifndef WIFI_SSID
#define WIFI_SSID "UNDEFINED"
#endif

#ifndef WIFI_PASSWORD  
#define WIFI_PASSWORD "UNDEFINED"
#endif

#ifndef MASTER_CONTROLLER_IP
#define MASTER_CONTROLLER_IP "192.168.4.1"
#endif

#ifndef CONTROL_BASE_ADDRESS
#define CONTROL_BASE_ADDRESS 0
#endif

#ifndef CONTROL_VALUE_OFF
#define CONTROL_VALUE_OFF 0
#endif

#ifndef CONTROL_VALUE_MAX
#define CONTROL_VALUE_MAX 32768
#endif

#ifndef CONTROL_VALUE_BOOLEAN_ON
#define CONTROL_VALUE_BOOLEAN_ON 32768
#endif

#ifndef CONTROL_UPDATE_INTERVAL_MS
#define CONTROL_UPDATE_INTERVAL_MS 100
#endif

#ifndef HEARTBEAT_INTERVAL_MS
#define HEARTBEAT_INTERVAL_MS 30000
#endif

#ifndef WIFI_FAST_RETRY_COUNT
#define WIFI_FAST_RETRY_COUNT 3
#endif

#ifndef WIFI_FAST_RETRY_DELAY_MS
#define WIFI_FAST_RETRY_DELAY_MS 5000
#endif

#ifndef WIFI_SLOW_RETRY_INTERVAL_MS
#define WIFI_SLOW_RETRY_INTERVAL_MS 300000
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 10000
#endif

#ifndef REGISTRATION_RETRY_COUNT
#define REGISTRATION_RETRY_COUNT 3
#endif

#ifndef REGISTRATION_RETRY_DELAY_MS
#define REGISTRATION_RETRY_DELAY_MS 5000
#endif

#ifndef HTTP_REQUEST_TIMEOUT_MS
#define HTTP_REQUEST_TIMEOUT_MS 5000
#endif

#ifndef PIN_STATUS_LED
#define PIN_STATUS_LED 2
#endif

#ifndef LED_PATTERN_CONNECTING
#define LED_PATTERN_CONNECTING 500
#endif

#ifndef LED_PATTERN_REGISTERED
#define LED_PATTERN_REGISTERED 2000
#endif

#ifndef LED_PATTERN_ERROR
#define LED_PATTERN_ERROR 200
#endif

#ifndef DEBUG_SERIAL_ENABLED
#define DEBUG_SERIAL_ENABLED 1
#endif

#if MQTT_ENABLED
#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP "192.168.4.1"
#endif

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#endif

#ifndef MQTT_CLIENT_ID_PREFIX
#define MQTT_CLIENT_ID_PREFIX "melvin"
#endif

#ifndef MQTT_RECONNECT_INTERVAL
#define MQTT_RECONNECT_INTERVAL 5000
#endif

#ifndef MQTT_TOPIC_DEVICES
#define MQTT_TOPIC_DEVICES "melvin/devices"
#endif

#ifndef MQTT_TOPIC_CONTROLS
#define MQTT_TOPIC_CONTROLS "melvin/controls"
#endif

#ifndef MQTT_TOPIC_EVENTS
#define MQTT_TOPIC_EVENTS "melvin/events"
#endif
#endif // MQTT_ENABLED

#endif // MELVIN_CONFIG_STUB_H
