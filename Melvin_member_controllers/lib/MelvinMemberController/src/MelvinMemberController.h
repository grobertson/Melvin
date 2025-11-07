/**
 * Melvin Member Controller Base Class Library
 * 
 * This header defines the abstract base class that all member controllers
 * should inherit from. It provides standardized communication, registration,
 * and control management functionality.
 * 
 * IMPORTANT: Before including this header, your project must define:
 * - enum ControlType (in config.h)
 * - enum DeviceType (in config.h)
 * - enum MQTTRoutingMode (in config.h)
 * - enum NetworkStatus (in config.h)
 * - Configuration constants (WIFI_SSID, MASTER_CONTROLLER_IP, etc.)
 */

#ifndef MELVIN_MEMBER_CONTROLLER_H
#define MELVIN_MEMBER_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <functional>

// The using project MUST include config.h before including this header
// config.h must define ControlType, DeviceType, MQTTRoutingMode, NetworkStatus enums
// If config.h hasn't been included, this will cause a compilation error (as intended)

#ifndef MQTT_ENABLED
#error "config.h must be included before MelvinMemberController.h and must define MQTT_ENABLED"
#endif

#if MQTT_ENABLED
#include <PicoMQTT.h>
#endif

/**
 * ENUMERATIONS - These must match the master controller definitions
 */

// Control Types
enum ControlType {
    CONTROL_BOOLEAN = 0,    // Toggle switches, buttons (0 or 1)
    CONTROL_CONTINUOUS,     // Potentiometers, sliders (0-32768)
    CONTROL_ROTARY,         // Rotary encoders (0-32768 with wrap)
    CONTROL_JOYSTICK_2AXIS, // 2-axis joystick (two 0-32768 values)
    CONTROL_RGB_GROUP       // RGB group (three 0-32768 values)
};

// Device Types
enum DeviceType {
    DEVICE_VEHICLE_INPUT = 0,
    DEVICE_HUMAN_INTERFACE,
    DEVICE_SENSOR_INPUT,
    DEVICE_EXTERNAL_LIGHT,
    DEVICE_CABIN_LIGHT,
    DEVICE_AUX_DEVICE
};

// MQTT Routing Modes
enum MQTTRoutingMode {
    MQTT_ROUTE_NONE = 0,        // No automatic routing
    MQTT_ROUTE_DIRECT,          // Route control changes directly to outputs
    MQTT_ROUTE_MAPPED,          // Route using control ID mapping
    MQTT_ROUTE_CUSTOM           // Custom routing via callback
};

// Network Status
enum NetworkStatus {
    NET_DISCONNECTED = 0,
    NET_CONNECTING,
    NET_CONNECTED,
    NET_REGISTERING,
    NET_REGISTERED,
    NET_ERROR
};

/**
 * Control Structure - Represents a control managed by this device
 */
struct MemberControl {
    uint16_t control_id;        // ID assigned by master controller
    uint16_t base_address;      // Base address for this control
    uint16_t control_number;    // Control number within base address
    ControlType type;           // Type of control
    uint16_t current_value;     // Current control value
    uint16_t current_value_y;   // Second axis (for joystick/RGB)
    uint16_t current_value_z;   // Third axis (for RGB)
    uint16_t last_sent_value;   // Last value sent to master (for change detection)
    uint16_t last_sent_value_y; // Last Y value sent
    uint16_t last_sent_value_z; // Last Z value sent
    String name;                // Human-readable name
    bool value_changed;         // Flag indicating value has changed
    
    MemberControl() : control_id(0), base_address(0), control_number(0), 
                      type(CONTROL_BOOLEAN), current_value(0), current_value_y(0), 
                      current_value_z(0), last_sent_value(0xFFFF), last_sent_value_y(0xFFFF), 
                      last_sent_value_z(0xFFFF), value_changed(false) {}
};

#if MQTT_ENABLED
/**
 * MQTT Event Structure - For publishing control changes and device events
 */
struct MQTTEvent {
    String device_id;
    String event_type;      // "control_change", "device_status", "alert"
    uint16_t control_id;
    uint16_t value;
    uint16_t value_y;       // For multi-axis controls
    uint16_t value_z;       // For RGB controls
    String metadata;        // Optional JSON metadata
    unsigned long timestamp;
    
    MQTTEvent() : control_id(0), value(0), value_y(0), value_z(0), timestamp(0) {}
};

/**
 * MQTT Control Command Structure - For receiving control commands from other devices
 */
struct MQTTControlCommand {
    String target_device_id;
    uint16_t control_id;
    uint16_t value;
    uint16_t value_y;       // For multi-axis controls
    uint16_t value_z;       // For RGB controls
    String command_type;    // "set", "toggle", "increment", "decrement"
    String source_device_id;
    unsigned long timestamp;
    
    MQTTControlCommand() : control_id(0), value(0), value_y(0), value_z(0), timestamp(0) {}
};

// MQTT subscription handler callback type
typedef std::function<void(const String& topic, const String& payload)> MQTTSubscriptionCallback;

// MQTT control routing callback type
typedef std::function<bool(const MQTTControlCommand& command)> MQTTControlCallback;
#endif

/**
 * Abstract Member Controller Class
 * 
 * This class provides the basic framework that all member controllers should follow.
 * Inherit from this class or use it as a template for specific implementations.
 */
class MelvinMemberController {
private:
    // Network state
    NetworkStatus network_status;
    unsigned long last_heartbeat_time;
    unsigned long last_control_update_time;
    unsigned long last_status_led_time;
    bool status_led_state;
    
    // Device information
    String device_id;
    String device_name;
    DeviceType device_type;
    
    // Controls managed by this device
    std::vector<MemberControl> controls;
    std::map<String, size_t> control_name_map; // Name to index mapping
    
    // HTTP client for API communication
    HTTPClient http_client;
    
#if MQTT_ENABLED
    // MQTT client and state
    PicoMQTT::Client mqtt_client;
    bool mqtt_connected;
    bool mqtt_enabled;
    unsigned long last_mqtt_reconnect;
    String mqtt_client_id;
    MQTTRoutingMode routing_mode;
    MQTTControlCallback control_routing_callback;
    
    // MQTT subscription tracking
    std::map<String, MQTTSubscriptionCallback> custom_subscriptions;
    std::map<uint16_t, uint16_t> control_id_mappings;  // input_id -> output_id
    
    // MQTT internal methods
    void handle_mqtt_message(const String& topic, const String& payload);
    void handle_control_command_message(const String& payload);
    void handle_device_status_message(const String& payload);
    void process_control_routing(const MQTTControlCommand& command);
    String build_device_topic(String sub_topic);
    String build_control_topic(uint16_t control_id, String sub_topic = "");
    MQTTEvent create_mqtt_event(const MemberControl& control, String event_type);
    MQTTControlCommand parse_control_command(const String& payload);
    void mqtt_auto_reconnect();
    bool init_mqtt_client();
    void shutdown_mqtt_client();
#endif
    
    // Internal methods
    bool connect_to_wifi();
    bool register_with_master();
    bool send_heartbeat();
    bool create_control_on_master(MemberControl& control);
    bool update_control_on_master(const MemberControl& control);
    void update_status_led();
    void handle_network_error();
    void auto_publish_changed_controls();
    
public:
    // Constructor
    MelvinMemberController(const String& id, const String& name, DeviceType type);
    
    // Destructor
    virtual ~MelvinMemberController();
    
    // Core lifecycle methods
    bool initialize();
    void update();
    void shutdown();
    
    // Control management methods
    bool add_control(const String& name, uint16_t control_number, ControlType type);
    bool set_control_value(const String& name, uint16_t value, uint16_t value_y = 0, uint16_t value_z = 0);
    bool get_control_value(const String& name, uint16_t& value, uint16_t& value_y, uint16_t& value_z);
    
    // Status methods
    NetworkStatus get_network_status() const { return network_status; }
    bool is_connected() const { return network_status >= NET_CONNECTED; }
    bool is_registered() const { return network_status == NET_REGISTERED; }
    
#if MQTT_ENABLED
    // MQTT client functionality
    bool is_mqtt_connected() const { return mqtt_connected; }
    void enable_mqtt(bool enabled = true) { mqtt_enabled = enabled; }
    
    // MQTT publishing
    bool publish_control_event(const String& control_name, String event_type = "control_change");
    bool publish_control_event(uint16_t control_id, String event_type = "control_change");
    bool publish_device_status(String status, String details = "");
    bool publish_custom_event(String event_type, String payload);
    
    // MQTT subscription management
    bool subscribe_to_device_controls(String device_id = "");  // Empty for all devices
    bool subscribe_to_control_events(uint16_t control_id = 0); // 0 for all controls
    bool subscribe_to_custom_topic(String topic, MQTTSubscriptionCallback callback);
    bool unsubscribe_from_topic(String topic);
    
    // MQTT control routing
    void set_mqtt_routing_mode(MQTTRoutingMode mode);
    void set_control_routing_callback(MQTTControlCallback callback);
    void add_control_id_mapping(uint16_t input_id, uint16_t output_id);
    void remove_control_id_mapping(uint16_t input_id);
    
    // MQTT event handling (virtual methods for derived classes)
    virtual void on_mqtt_control_command(const MQTTControlCommand& command);
    virtual void on_mqtt_device_status(const String& device_id, const String& status, const String& details);
    virtual void on_mqtt_custom_event(const String& topic, const String& payload);
    virtual void on_mqtt_connected();
    virtual void on_mqtt_disconnected();
#endif
    
    // Virtual methods for hardware-specific implementations
    virtual void setup_hardware() = 0;      // Initialize hardware (GPIO, ADC, etc.)
    virtual void read_inputs() = 0;          // Read physical inputs and update control values
    virtual void write_outputs() = 0;        // Write outputs based on control values (if applicable)
    virtual void handle_error(const String& error) = 0; // Handle errors (LED, buzzer, etc.)
    
    // Utility methods
    void debug_print(const String& message);
    void debug_printf(const char* format, ...);
};

// ============================================================================
// INLINE IMPLEMENTATION (Header-Only Library)
// ============================================================================

// Include config stub for library constants
#include "config_stub.h"

inline MelvinMemberController::~MelvinMemberController() {
    shutdown();
}

inline bool MelvinMemberController::initialize() {
    debug_print("Initializing member controller...");
    
    // Initialize hardware first
    setup_hardware();
    
    // Set up initial controls (customize this for each controller type)
    if (!add_control("Emergency Button", 1, CONTROL_BOOLEAN)) return false;
    if (!add_control("Auxiliary Button", 2, CONTROL_BOOLEAN)) return false;
    if (!add_control("Main Switch", 3, CONTROL_BOOLEAN)) return false;
    if (!add_control("Auxiliary Switch", 4, CONTROL_BOOLEAN)) return false;
    if (!add_control("Brightness Control", 5, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Volume Control", 6, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Spotlight Direction", 7, CONTROL_JOYSTICK_2AXIS)) return false;
    
    // Connect to WiFi and register with master
    if (!connect_to_wifi()) {
        handle_error("WiFi connection failed");
        return false;
    }
    
    if (!register_with_master()) {
        handle_error("Master registration failed");
        return false;
    }
    
#if MQTT_ENABLED
    // Initialize MQTT client if enabled
    if (mqtt_enabled) {
        if (!init_mqtt_client()) {
            debug_print("Warning: MQTT initialization failed, continuing without MQTT");
        }
    }
#endif
    
    debug_print("Member controller initialized successfully");
    return true;
}

inline void MelvinMemberController::update() {
    unsigned long current_time = millis();
    
    // Update status LED
    update_status_led();
    
    // Check WiFi connection and attempt reconnection if needed
    if (WiFi.status() != WL_CONNECTED) {
        if (network_status > NET_DISCONNECTED) {
            network_status = NET_DISCONNECTED;
            debug_print("WiFi connection lost - attempting reconnection");
        }
        
        // Attempt to reconnect using retry logic
        if (connect_to_wifi()) {
            debug_print("WiFi reconnected successfully");
            // Try to re-register with master
            if (register_with_master()) {
                debug_print("Re-registered with master after reconnection");
            }
        }
        return;
    }
    
    // If connected but not registered, try to register
    if (network_status == NET_CONNECTED && !is_registered()) {
        register_with_master();
    }
    
#if MQTT_ENABLED
    // Handle MQTT client updates
    if (mqtt_enabled) {
        mqtt_client.loop();
        mqtt_auto_reconnect();
    }
#endif
    
    // Read inputs at regular intervals
    if (current_time - last_control_update_time >= CONTROL_UPDATE_INTERVAL_MS) {
        read_inputs();
        
        // Send any changed control values to master and publish MQTT events
        for (auto& control : controls) {
            if (control.value_changed) {
                if (update_control_on_master(control)) {
                    control.last_sent_value = control.current_value;
                    control.last_sent_value_y = control.current_value_y;
                    control.last_sent_value_z = control.current_value_z;
                    control.value_changed = false;
                } else {
                    handle_error("Failed to update control: " + control.name);
                }
            }
        }
        
        // Auto-publish changed controls via MQTT
        auto_publish_changed_controls();
        
        last_control_update_time = current_time;
    }
    
    // Send heartbeat at regular intervals
    if (current_time - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
        if (!send_heartbeat()) {
            handle_error("Heartbeat failed");
            network_status = NET_ERROR;
        }
        last_heartbeat_time = current_time;
    }
    
    // Write outputs (if applicable)
    write_outputs();
}

inline void MelvinMemberController::shutdown() {
    debug_print("Shutting down member controller...");
    
#if MQTT_ENABLED
    // Shutdown MQTT client
    if (mqtt_enabled) {
        shutdown_mqtt_client();
    }
#endif
    
    // Attempt to deregister from master
    if (network_status == NET_REGISTERED) {
        DynamicJsonDocument doc(512);
        doc["device_id"] = device_id;
        
        String json_string;
        serializeJson(doc, json_string);
        
        http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/deregister/device/");
        http_client.addHeader("Content-Type", "application/json");
        http_client.POST(json_string);
        http_client.end();
    }
    
    WiFi.disconnect(true);
    network_status = NET_DISCONNECTED;
}

inline bool MelvinMemberController::add_control(const String& name, uint16_t control_number, ControlType type) {
    MemberControl control;
    control.base_address = CONTROL_BASE_ADDRESS;
    control.control_number = control_number;
    control.type = type;
    control.name = name;
    
    controls.push_back(control);
    control_name_map[name] = controls.size() - 1;
    
    debug_print("Added control: " + name + " (Type: " + String((int)type) + ")");
    return true;
}

inline bool MelvinMemberController::set_control_value(const String& name, uint16_t value, uint16_t value_y, uint16_t value_z) {
    auto it = control_name_map.find(name);
    if (it == control_name_map.end()) {
        return false;
    }
    
    MemberControl& control = controls[it->second];
    
    // Check if value actually changed
    bool changed = (control.current_value != value) || 
                   (control.current_value_y != value_y) || 
                   (control.current_value_z != value_z);
    
    if (changed) {
        control.current_value = value;
        control.current_value_y = value_y;
        control.current_value_z = value_z;
        control.value_changed = true;
    }
    
    return true;
}

inline bool MelvinMemberController::get_control_value(const String& name, uint16_t& value, uint16_t& value_y, uint16_t& value_z) {
    auto it = control_name_map.find(name);
    if (it == control_name_map.end()) {
        return false;
    }
    
    const MemberControl& control = controls[it->second];
    value = control.current_value;
    value_y = control.current_value_y;
    value_z = control.current_value_z;
    
    return true;
}

inline bool MelvinMemberController::connect_to_wifi() {
    static uint8_t fast_retry_count = 0;
    static unsigned long last_slow_retry = 0;
    
    debug_print("Connecting to WiFi: " + String(WIFI_SSID));
    
    // If we've exhausted fast retries, check if it's time for a slow retry
    if (fast_retry_count >= WIFI_FAST_RETRY_COUNT) {
        unsigned long current_time = millis();
        if (current_time - last_slow_retry < WIFI_SLOW_RETRY_INTERVAL_MS) {
            debug_print("WiFi: Waiting for slow retry interval...");
            return false;
        }
        // Reset for new attempt cycle
        fast_retry_count = 0;
        last_slow_retry = current_time;
        debug_print("WiFi: Starting new retry cycle");
    }
    
    // Increment fast retry counter
    fast_retry_count++;
    debug_printf("WiFi connection attempt %d of %d", fast_retry_count, WIFI_FAST_RETRY_COUNT);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < WIFI_CONNECT_TIMEOUT_MS) {
        network_status = NET_CONNECTING;
        delay(500);
        debug_print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        network_status = NET_CONNECTED;
        fast_retry_count = 0;  // Reset counter on success
        debug_print("WiFi connected! IP: " + WiFi.localIP().toString());
        return true;
    } else {
        network_status = NET_ERROR;
        
        if (fast_retry_count < WIFI_FAST_RETRY_COUNT) {
            debug_printf("WiFi connection failed. Retrying in %d seconds...", WIFI_FAST_RETRY_DELAY_MS / 1000);
            delay(WIFI_FAST_RETRY_DELAY_MS);
        } else {
            debug_printf("WiFi connection failed after %d attempts. Will retry in %d minutes.", 
                        WIFI_FAST_RETRY_COUNT, WIFI_SLOW_RETRY_INTERVAL_MS / 60000);
            last_slow_retry = millis();
        }
        return false;
    }
}

inline bool MelvinMemberController::register_with_master() {
    static uint8_t retry_count = 0;
    
    debug_printf("Registering with master controller (attempt %d/%d)...", 
                retry_count + 1, REGISTRATION_RETRY_COUNT);
    
    DynamicJsonDocument doc(1024);
    doc["device_id"] = device_id;
    doc["device_type"] = (int)device_type;
    doc["name"] = device_name;
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/register/device/");
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.POST(json_string);
    
    if (response_code == 200) {
        network_status = NET_REGISTERED;
        retry_count = 0;  // Reset counter on success
        debug_print("Successfully registered with master");
        
        // Create controls on master
        for (auto& control : controls) {
            if (!create_control_on_master(control)) {
                debug_print("Warning: Failed to create control: " + control.name);
            }
        }
        
        http_client.end();
        return true;
    } else {
        debug_printf("Registration failed. Response code: %d", response_code);
        http_client.end();
        
        retry_count++;
        if (retry_count < REGISTRATION_RETRY_COUNT) {
            debug_printf("Retrying registration in %d seconds...", REGISTRATION_RETRY_DELAY_MS / 1000);
            delay(REGISTRATION_RETRY_DELAY_MS);
            return register_with_master();  // Recursive retry
        } else {
            debug_print("Registration failed after maximum retries");
            retry_count = 0;  // Reset for future attempts
            return false;
        }
    }
}

inline bool MelvinMemberController::send_heartbeat() {
    return register_with_master(); // Re-registration serves as heartbeat
}

inline bool MelvinMemberController::create_control_on_master(MemberControl& control) {
    DynamicJsonDocument doc(512);
    doc["base_address"] = control.base_address;
    doc["control_number"] = control.control_number;
    doc["type"] = (int)control.type;
    doc["name"] = control.name;
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/controls/");
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.POST(json_string);
    
    if (response_code == 200) {
        String response = http_client.getString();
        DynamicJsonDocument response_doc(512);
        deserializeJson(response_doc, response);
        
        control.control_id = response_doc["control_id"];
        debug_print("Created control: " + control.name + " (ID: " + String(control.control_id) + ")");
        
        http_client.end();
        return true;
    } else {
        debug_print("Failed to create control: " + control.name + " (Code: " + String(response_code) + ")");
        http_client.end();
        return false;
    }
}

inline bool MelvinMemberController::update_control_on_master(const MemberControl& control) {
    if (control.control_id == 0) return false; // Not created yet
    
    DynamicJsonDocument doc(512);
    doc["value"] = control.current_value;
    
    if (control.type == CONTROL_JOYSTICK_2AXIS || control.type == CONTROL_RGB_GROUP) {
        doc["value_y"] = control.current_value_y;
        if (control.type == CONTROL_RGB_GROUP) {
            doc["value_z"] = control.current_value_z;
        }
    }
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/controls/" + String(control.control_id));
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.PUT(json_string);
    http_client.end();
    
    return (response_code == 200);
}

inline void MelvinMemberController::update_status_led() {
    unsigned long current_time = millis();
    unsigned long flash_interval;
    
    switch (network_status) {
        case NET_CONNECTING:
            flash_interval = LED_PATTERN_CONNECTING;
            break;
        case NET_REGISTERED:
            flash_interval = LED_PATTERN_REGISTERED;
            break;
        case NET_ERROR:
            flash_interval = LED_PATTERN_ERROR;
            break;
        default:
            digitalWrite(PIN_STATUS_LED, LOW);
            return;
    }
    
    if (current_time - last_status_led_time >= flash_interval) {
        status_led_state = !status_led_state;
        digitalWrite(PIN_STATUS_LED, status_led_state);
        last_status_led_time = current_time;
    }
}

inline void MelvinMemberController::debug_print(const String& message) {
    if (DEBUG_SERIAL_ENABLED) {
        Serial.println("[" + device_id + "] " + message);
    }
}

inline void MelvinMemberController::debug_printf(const char* format, ...) {
    if (DEBUG_SERIAL_ENABLED) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        debug_print(String(buffer));
    }
}

// Utility function implementations
inline String control_type_to_string(ControlType type) {
    switch (type) {
        case CONTROL_BOOLEAN: return "Boolean";
        case CONTROL_CONTINUOUS: return "Continuous";
        case CONTROL_ROTARY: return "Rotary";
        case CONTROL_JOYSTICK_2AXIS: return "Joystick";
        case CONTROL_RGB_GROUP: return "RGB";
        default: return "Unknown";
    }
}

inline String network_status_to_string(NetworkStatus status) {
    switch (status) {
        case NET_DISCONNECTED: return "Disconnected";
        case NET_CONNECTING: return "Connecting";
        case NET_CONNECTED: return "Connected";
        case NET_REGISTERED: return "Registered";
        case NET_ERROR: return "Error";
        default: return "Unknown";
    }
}

inline uint16_t map_analog_to_control(int analog_value, int analog_min, int analog_max) {
    return map(analog_value, analog_min, analog_max, CONTROL_VALUE_OFF, CONTROL_VALUE_MAX);
}

inline uint16_t apply_deadband(uint16_t current_value, uint16_t last_value, uint16_t deadband) {
    if (abs((int)current_value - (int)last_value) < deadband) {
        return last_value; // No change, within deadband
    }
    return current_value;
}

inline bool read_digital_debounced(int pin, unsigned long debounce_time) {
    static std::map<int, bool> last_states;
    static std::map<int, unsigned long> last_change_times;
    
    bool current_state = digitalRead(pin);
    bool last_state = last_states[pin];
    
    if (current_state != last_state) {
        last_change_times[pin] = millis();
    }
    
    if (millis() - last_change_times[pin] > debounce_time) {
        last_states[pin] = current_state;
        return current_state;
    }
    
    return last_state;
}

#if MQTT_ENABLED
// MQTT Implementation Methods

inline bool MelvinMemberController::init_mqtt_client() {
    if (!mqtt_enabled) return false;
    
    debug_print("Initializing MQTT client...");
    
    // Attempt initial connection
    if (mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, mqtt_client_id.c_str())) {
        mqtt_connected = true;
        debug_print("MQTT client initialized successfully");
        
        // Set up global message handler using subscribe with wildcard
        // This will catch all messages and route them appropriately
        mqtt_client.subscribe("#", [this](const char* topic, const char* payload) {
            this->handle_mqtt_message(String(topic), String(payload));
        });
        
        on_mqtt_connected();
        return true;
    } else {
        mqtt_connected = false;
        debug_print("MQTT initial connection failed");
        return false;
    }
}

inline void MelvinMemberController::shutdown_mqtt_client() {
    if (mqtt_connected) {
        // Publish disconnection status
        publish_device_status("disconnecting", "Device shutting down");
        
        // Unsubscribe from all custom topics
        for (auto& subscription : custom_subscriptions) {
            mqtt_client.unsubscribe(subscription.first);
        }
        custom_subscriptions.clear();
        
        mqtt_client.disconnect();
    }
    mqtt_connected = false;
}

inline void MelvinMemberController::mqtt_auto_reconnect() {
    if (!mqtt_enabled || mqtt_connected) return;
    
    unsigned long current_time = millis();
    if (current_time - last_mqtt_reconnect < MQTT_RECONNECT_INTERVAL) return;
    
    debug_print("Attempting MQTT reconnection...");
    if (mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, mqtt_client_id.c_str())) {
        mqtt_connected = true;
        debug_print("MQTT reconnected successfully");
        
        // Re-subscribe to global handler
        mqtt_client.subscribe("#", [this](const char* topic, const char* payload) {
            this->handle_mqtt_message(String(topic), String(payload));
        });
        
        // Call connected callback to re-establish subscriptions
        on_mqtt_connected();
        
        last_mqtt_reconnect = current_time;
    } else {
        mqtt_connected = false;
        debug_print("MQTT reconnection failed");
        last_mqtt_reconnect = current_time;
    }
}

inline bool MelvinMemberController::publish_control_event(const String& control_name, String event_type) {
    if (!mqtt_connected) return false;
    
    auto it = control_name_map.find(control_name);
    if (it == control_name_map.end()) return false;
    
    const MemberControl& control = controls[it->second];
    return publish_control_event(control.control_id, event_type);
}

inline bool MelvinMemberController::publish_control_event(uint16_t control_id, String event_type) {
    if (!mqtt_connected) return false;
    
    // Find control by ID
    MemberControl* control = nullptr;
    for (auto& ctrl : controls) {
        if (ctrl.control_id == control_id) {
            control = &ctrl;
            break;
        }
    }
    
    if (!control) return false;
    
    MQTTEvent event = create_mqtt_event(*control, event_type);
    
    // Build JSON payload
    DynamicJsonDocument doc(512);
    doc["device_id"] = event.device_id;
    doc["event_type"] = event.event_type;
    doc["control_id"] = event.control_id;
    doc["value"] = event.value;
    
    if (control->type == CONTROL_JOYSTICK_2AXIS || control->type == CONTROL_RGB_GROUP) {
        doc["value_y"] = event.value_y;
        if (control->type == CONTROL_RGB_GROUP) {
            doc["value_z"] = event.value_z;
        }
    }
    
    doc["timestamp"] = event.timestamp;
    if (event.metadata.length() > 0) {
        doc["metadata"] = event.metadata;
    }
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = build_control_topic(control_id, "events");
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::publish_device_status(String status, String details) {
    if (!mqtt_connected) return false;
    
    DynamicJsonDocument doc(256);
    doc["device_id"] = device_id;
    doc["status"] = status;
    doc["details"] = details;
    doc["timestamp"] = millis();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = build_device_topic("status");
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::publish_custom_event(String event_type, String payload) {
    if (!mqtt_connected) return false;
    
    String topic = build_device_topic("events/" + event_type);
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::subscribe_to_device_controls(String device_id_filter) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (device_id_filter.length() > 0) {
        topic = String(MQTT_TOPIC_DEVICES) + "/" + device_id_filter + "/controls/+";
    } else {
        topic = String(MQTT_TOPIC_CONTROLS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::subscribe_to_control_events(uint16_t control_id) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (control_id > 0) {
        topic = build_control_topic(control_id, "events");
    } else {
        topic = String(MQTT_TOPIC_EVENTS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::subscribe_to_custom_topic(String topic, MQTTSubscriptionCallback callback) {
    if (!mqtt_connected) return false;
    
    custom_subscriptions[topic] = callback;
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::unsubscribe_from_topic(String topic) {
    if (!mqtt_connected) return false;
    
    auto it = custom_subscriptions.find(topic);
    if (it != custom_subscriptions.end()) {
        custom_subscriptions.erase(it);
    }
    
    mqtt_client.unsubscribe(topic);
    return true; // PicoMQTT unsubscribe returns void, so assume success
}

inline void MelvinMemberController::set_mqtt_routing_mode(MQTTRoutingMode mode) {
    routing_mode = mode;
    debug_print("MQTT routing mode set to: " + String((int)mode));
}

inline void MelvinMemberController::set_control_routing_callback(MQTTControlCallback callback) {
    control_routing_callback = callback;
}

inline void MelvinMemberController::add_control_id_mapping(uint16_t input_id, uint16_t output_id) {
    control_id_mappings[input_id] = output_id;
    debug_print("Added control mapping: " + String(input_id) + " -> " + String(output_id));
}

inline void MelvinMemberController::remove_control_id_mapping(uint16_t input_id) {
    auto it = control_id_mappings.find(input_id);
    if (it != control_id_mappings.end()) {
        control_id_mappings.erase(it);
        debug_print("Removed control mapping for: " + String(input_id));
    }
}

inline void MelvinMemberController::handle_mqtt_message(const String& topic, const String& payload) {
    debug_print("MQTT message received: " + topic);
    
    // Check for custom subscriptions first
    auto it = custom_subscriptions.find(topic);
    if (it != custom_subscriptions.end()) {
        it->second(topic, payload);
        return;
    }
    
    // Handle control commands
    if (topic.startsWith(String(MQTT_TOPIC_CONTROLS) + "/") || 
        topic.indexOf("/controls/") >= 0) {
        handle_control_command_message(payload);
        return;
    }
    
    // Handle device status messages
    if (topic.indexOf("/status") >= 0) {
        handle_device_status_message(payload);
        return;
    }
    
    // Handle generic events
    on_mqtt_custom_event(topic, payload);
}

inline void MelvinMemberController::handle_control_command_message(const String& payload) {
    MQTTControlCommand command = parse_control_command(payload);
    
    // Check if this command is targeted at this device
    if (command.target_device_id.length() > 0 && command.target_device_id != device_id) {
        return; // Not for this device
    }
    
    // Process routing if enabled
    process_control_routing(command);
    
    // Call virtual handler
    on_mqtt_control_command(command);
}

inline void MelvinMemberController::handle_device_status_message(const String& payload) {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        return;
    }
    
    String device_id = doc["device_id"];
    String status = doc["status"];
    String details = doc["details"];
    
    on_mqtt_device_status(device_id, status, details);
}

inline void MelvinMemberController::process_control_routing(const MQTTControlCommand& command) {
    if (routing_mode == MQTT_ROUTE_NONE) return;
    
    // Try custom callback first
    if (control_routing_callback && control_routing_callback(command)) {
        return; // Custom handler processed it
    }
    
    // Handle direct routing
    if (routing_mode == MQTT_ROUTE_DIRECT) {
        // Find control and update its value
        for (auto& control : controls) {
            if (control.control_id == command.control_id) {
                control.current_value = command.value;
                control.current_value_y = command.value_y;
                control.current_value_z = command.value_z;
                control.value_changed = true;
                break;
            }
        }
    }
    
    // Handle mapped routing
    if (routing_mode == MQTT_ROUTE_MAPPED) {
        auto it = control_id_mappings.find(command.control_id);
        if (it != control_id_mappings.end()) {
            uint16_t output_id = it->second;
            for (auto& control : controls) {
                if (control.control_id == output_id) {
                    control.current_value = command.value;
                    control.current_value_y = command.value_y;
                    control.current_value_z = command.value_z;
                    control.value_changed = true;
                    break;
                }
            }
        }
    }
}

inline String MelvinMemberController::build_device_topic(String sub_topic) {
    return String(MQTT_TOPIC_DEVICES) + "/" + device_id + "/" + sub_topic;
}

inline String MelvinMemberController::build_control_topic(uint16_t control_id, String sub_topic) {
    String topic = String(MQTT_TOPIC_CONTROLS) + "/" + String(control_id);
    if (sub_topic.length() > 0) {
        topic += "/" + sub_topic;
    }
    return topic;
}

inline MQTTEvent MelvinMemberController::create_mqtt_event(const MemberControl& control, String event_type) {
    MQTTEvent event;
    event.device_id = device_id;
    event.event_type = event_type;
    event.control_id = control.control_id;
    event.value = control.current_value;
    event.value_y = control.current_value_y;
    event.value_z = control.current_value_z;
    event.timestamp = millis();
    
    // Add control name as metadata
    DynamicJsonDocument metadata(128);
    metadata["control_name"] = control.name;
    metadata["control_type"] = (int)control.type;
    serializeJson(metadata, event.metadata);
    
    return event;
}

inline MQTTControlCommand MelvinMemberController::parse_control_command(const String& payload) {
    MQTTControlCommand command;
    
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
        command.target_device_id = doc["target_device_id"].as<String>();
        command.control_id = doc["control_id"];
        command.value = doc["value"];
        command.value_y = doc["value_y"];
        command.value_z = doc["value_z"];
        command.command_type = doc["command_type"].as<String>();
        command.source_device_id = doc["source_device_id"].as<String>();
        command.timestamp = doc["timestamp"];
    }
    
    return command;
}

inline void MelvinMemberController::auto_publish_changed_controls() {
    if (!mqtt_connected) return;
    
    for (const auto& control : controls) {
        if (control.value_changed && control.control_id > 0) {
            publish_control_event(control.control_id, "control_change");
        }
    }
}

// Default virtual method implementations
inline void MelvinMemberController::on_mqtt_control_command(const MQTTControlCommand& command) {
    debug_print("MQTT control command received: Control " + String(command.control_id) + 
                " = " + String(command.value));
}

inline void MelvinMemberController::on_mqtt_device_status(const String& device_id, const String& status, const String& details) {
    debug_print("Device status: " + device_id + " - " + status + " (" + details + ")");
}

inline void MelvinMemberController::on_mqtt_custom_event(const String& topic, const String& payload) {
    debug_print("Custom MQTT event: " + topic);
}

inline void MelvinMemberController::on_mqtt_connected() {
    debug_print("MQTT connection established");
    publish_device_status("connected", "Device online and ready");
}

inline void MelvinMemberController::on_mqtt_disconnected() {
    debug_print("MQTT connection lost");
}

#endif // MQTT_ENABLED
/**
 * Utility Functions
 */

// Convert control type to string for debugging
String control_type_to_string(ControlType type);

// Convert network status to string for debugging  
String network_status_to_string(NetworkStatus status);

// Map analog reading to control value range
uint16_t map_analog_to_control(int analog_value, int analog_min = 0, int analog_max = 4095);

// Apply deadband to analog readings to reduce noise
uint16_t apply_deadband(uint16_t current_value, uint16_t last_value, uint16_t deadband = 50);

// Read digital input with debouncing
bool read_digital_debounced(int pin, unsigned long debounce_time = 50);


// ===== INLINE IMPLEMENTATION =====
#include " config_stub.h\n

// ============================================================================
// INLINE IMPLEMENTATION - Header-Only Library
// ============================================================================

#include "config_stub.h"

inline MelvinMemberController::~MelvinMemberController() {
    shutdown();
}

inline bool MelvinMemberController::initialize() {
    debug_print("Initializing member controller...");
    
    // Initialize hardware first
    setup_hardware();
    
    // Set up initial controls (customize this for each controller type)
    if (!add_control("Emergency Button", 1, CONTROL_BOOLEAN)) return false;
    if (!add_control("Auxiliary Button", 2, CONTROL_BOOLEAN)) return false;
    if (!add_control("Main Switch", 3, CONTROL_BOOLEAN)) return false;
    if (!add_control("Auxiliary Switch", 4, CONTROL_BOOLEAN)) return false;
    if (!add_control("Brightness Control", 5, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Volume Control", 6, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Spotlight Direction", 7, CONTROL_JOYSTICK_2AXIS)) return false;
    
    // Connect to WiFi and register with master
    if (!connect_to_wifi()) {
        handle_error("WiFi connection failed");
        return false;
    }
    
    if (!register_with_master()) {
        handle_error("Master registration failed");
        return false;
    }
    
#if MQTT_ENABLED
    // Initialize MQTT client if enabled
    if (mqtt_enabled) {
        if (!init_mqtt_client()) {
            debug_print("Warning: MQTT initialization failed, continuing without MQTT");
        }
    }
#endif
    
    debug_print("Member controller initialized successfully");
    return true;
}

inline void MelvinMemberController::update() {
    unsigned long current_time = millis();
    
    // Update status LED
    update_status_led();
    
    // Check WiFi connection and attempt reconnection if needed
    if (WiFi.status() != WL_CONNECTED) {
        if (network_status > NET_DISCONNECTED) {
            network_status = NET_DISCONNECTED;
            debug_print("WiFi connection lost - attempting reconnection");
        }
        
        // Attempt to reconnect using retry logic
        if (connect_to_wifi()) {
            debug_print("WiFi reconnected successfully");
            // Try to re-register with master
            if (register_with_master()) {
                debug_print("Re-registered with master after reconnection");
            }
        }
        return;
    }
    
    // If connected but not registered, try to register
    if (network_status == NET_CONNECTED && !is_registered()) {
        register_with_master();
    }
    
#if MQTT_ENABLED
    // Handle MQTT client updates
    if (mqtt_enabled) {
        mqtt_client.loop();
        mqtt_auto_reconnect();
    }
#endif
    
    // Read inputs at regular intervals
    if (current_time - last_control_update_time >= CONTROL_UPDATE_INTERVAL_MS) {
        read_inputs();
        
        // Send any changed control values to master and publish MQTT events
        for (auto& control : controls) {
            if (control.value_changed) {
                if (update_control_on_master(control)) {
                    control.last_sent_value = control.current_value;
                    control.last_sent_value_y = control.current_value_y;
                    control.last_sent_value_z = control.current_value_z;
                    control.value_changed = false;
                } else {
                    handle_error("Failed to update control: " + control.name);
                }
            }
        }
        
        // Auto-publish changed controls via MQTT
        auto_publish_changed_controls();
        
        last_control_update_time = current_time;
    }
    
    // Send heartbeat at regular intervals
    if (current_time - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
        if (!send_heartbeat()) {
            handle_error("Heartbeat failed");
            network_status = NET_ERROR;
        }
        last_heartbeat_time = current_time;
    }
    
    // Write outputs (if applicable)
    write_outputs();
}

inline void MelvinMemberController::shutdown() {
    debug_print("Shutting down member controller...");
    
#if MQTT_ENABLED
    // Shutdown MQTT client
    if (mqtt_enabled) {
        shutdown_mqtt_client();
    }
#endif
    
    // Attempt to deregister from master
    if (network_status == NET_REGISTERED) {
        DynamicJsonDocument doc(512);
        doc["device_id"] = device_id;
        
        String json_string;
        serializeJson(doc, json_string);
        
        http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/deregister/device/");
        http_client.addHeader("Content-Type", "application/json");
        http_client.POST(json_string);
        http_client.end();
    }
    
    WiFi.disconnect(true);
    network_status = NET_DISCONNECTED;
}

inline bool MelvinMemberController::add_control(const String& name, uint16_t control_number, ControlType type) {
    MemberControl control;
    control.base_address = CONTROL_BASE_ADDRESS;
    control.control_number = control_number;
    control.type = type;
    control.name = name;
    
    controls.push_back(control);
    control_name_map[name] = controls.size() - 1;
    
    debug_print("Added control: " + name + " (Type: " + String((int)type) + ")");
    return true;
}

inline bool MelvinMemberController::set_control_value(const String& name, uint16_t value, uint16_t value_y, uint16_t value_z) {
    auto it = control_name_map.find(name);
    if (it == control_name_map.end()) {
        return false;
    }
    
    MemberControl& control = controls[it->second];
    
    // Check if value actually changed
inline     bool changed = (control.current_value != value) || 
                   (control.current_value_y != value_y) || 
                   (control.current_value_z != value_z);
    
    if (changed) {
        control.current_value = value;
        control.current_value_y = value_y;
        control.current_value_z = value_z;
        control.value_changed = true;
    }
    
    return true;
}

inline bool MelvinMemberController::get_control_value(const String& name, uint16_t& value, uint16_t& value_y, uint16_t& value_z) {
    auto it = control_name_map.find(name);
    if (it == control_name_map.end()) {
        return false;
    }
    
    const MemberControl& control = controls[it->second];
    value = control.current_value;
    value_y = control.current_value_y;
    value_z = control.current_value_z;
    
    return true;
}

inline bool MelvinMemberController::connect_to_wifi() {
    static uint8_t fast_retry_count = 0;
    static unsigned long last_slow_retry = 0;
    
    debug_print("Connecting to WiFi: " + String(WIFI_SSID));
    
    // If we've exhausted fast retries, check if it's time for a slow retry
    if (fast_retry_count >= WIFI_FAST_RETRY_COUNT) {
        unsigned long current_time = millis();
        if (current_time - last_slow_retry < WIFI_SLOW_RETRY_INTERVAL_MS) {
            debug_print("WiFi: Waiting for slow retry interval...");
            return false;
        }
        // Reset for new attempt cycle
        fast_retry_count = 0;
        last_slow_retry = current_time;
        debug_print("WiFi: Starting new retry cycle");
    }
    
    // Increment fast retry counter
    fast_retry_count++;
    debug_printf("WiFi connection attempt %d of %d", fast_retry_count, WIFI_FAST_RETRY_COUNT);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < WIFI_CONNECT_TIMEOUT_MS) {
        network_status = NET_CONNECTING;
        delay(500);
        debug_print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        network_status = NET_CONNECTED;
        fast_retry_count = 0;  // Reset counter on success
        debug_print("WiFi connected! IP: " + WiFi.localIP().toString());
        return true;
    } else {
        network_status = NET_ERROR;
        
        if (fast_retry_count < WIFI_FAST_RETRY_COUNT) {
            debug_printf("WiFi connection failed. Retrying in %d seconds...", WIFI_FAST_RETRY_DELAY_MS / 1000);
            delay(WIFI_FAST_RETRY_DELAY_MS);
        } else {
            debug_printf("WiFi connection failed after %d attempts. Will retry in %d minutes.", 
                        WIFI_FAST_RETRY_COUNT, WIFI_SLOW_RETRY_INTERVAL_MS / 60000);
            last_slow_retry = millis();
        }
        return false;
    }
}

inline bool MelvinMemberController::register_with_master() {
    static uint8_t retry_count = 0;
    
    debug_printf("Registering with master controller (attempt %d/%d)...", 
                retry_count + 1, REGISTRATION_RETRY_COUNT);
    
    DynamicJsonDocument doc(1024);
    doc["device_id"] = device_id;
    doc["device_type"] = (int)device_type;
    doc["name"] = device_name;
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/register/device/");
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.POST(json_string);
    
    if (response_code == 200) {
        network_status = NET_REGISTERED;
        retry_count = 0;  // Reset counter on success
        debug_print("Successfully registered with master");
        
        // Create controls on master
        for (auto& control : controls) {
            if (!create_control_on_master(control)) {
                debug_print("Warning: Failed to create control: " + control.name);
            }
        }
        
        http_client.end();
        return true;
    } else {
        debug_printf("Registration failed. Response code: %d", response_code);
        http_client.end();
        
        retry_count++;
        if (retry_count < REGISTRATION_RETRY_COUNT) {
            debug_printf("Retrying registration in %d seconds...", REGISTRATION_RETRY_DELAY_MS / 1000);
            delay(REGISTRATION_RETRY_DELAY_MS);
            return register_with_master();  // Recursive retry
        } else {
            debug_print("Registration failed after maximum retries");
            retry_count = 0;  // Reset for future attempts
            return false;
        }
    }
}

inline bool MelvinMemberController::send_heartbeat() {
    return register_with_master(); // Re-registration serves as heartbeat
}

inline bool MelvinMemberController::create_control_on_master(MemberControl& control) {
    DynamicJsonDocument doc(512);
    doc["base_address"] = control.base_address;
    doc["control_number"] = control.control_number;
    doc["type"] = (int)control.type;
    doc["name"] = control.name;
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/controls/");
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.POST(json_string);
    
    if (response_code == 200) {
        String response = http_client.getString();
        DynamicJsonDocument response_doc(512);
        deserializeJson(response_doc, response);
        
        control.control_id = response_doc["control_id"];
        debug_print("Created control: " + control.name + " (ID: " + String(control.control_id) + ")");
        
        http_client.end();
        return true;
    } else {
        debug_print("Failed to create control: " + control.name + " (Code: " + String(response_code) + ")");
        http_client.end();
        return false;
    }
}

inline bool MelvinMemberController::update_control_on_master(const MemberControl& control) {
    if (control.control_id == 0) return false; // Not created yet
    
    DynamicJsonDocument doc(512);
    doc["value"] = control.current_value;
    
    if (control.type == CONTROL_JOYSTICK_2AXIS || control.type == CONTROL_RGB_GROUP) {
        doc["value_y"] = control.current_value_y;
        if (control.type == CONTROL_RGB_GROUP) {
            doc["value_z"] = control.current_value_z;
        }
    }
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://" + String(MASTER_CONTROLLER_IP) + "/controls/" + String(control.control_id));
    http_client.addHeader("Content-Type", "application/json");
    http_client.setTimeout(HTTP_REQUEST_TIMEOUT_MS);
    
    int response_code = http_client.PUT(json_string);
    http_client.end();
    
    return (response_code == 200);
}

inline void MelvinMemberController::update_status_led() {
    unsigned long current_time = millis();
    unsigned long flash_interval;
    
    switch (network_status) {
        case NET_CONNECTING:
            flash_interval = LED_PATTERN_CONNECTING;
            break;
        case NET_REGISTERED:
            flash_interval = LED_PATTERN_REGISTERED;
            break;
        case NET_ERROR:
            flash_interval = LED_PATTERN_ERROR;
            break;
        default:
            digitalWrite(PIN_STATUS_LED, LOW);
            return;
    }
    
    if (current_time - last_status_led_time >= flash_interval) {
        status_led_state = !status_led_state;
        digitalWrite(PIN_STATUS_LED, status_led_state);
        last_status_led_time = current_time;
    }
}

inline void MelvinMemberController::debug_print(const String& message) {
    if (DEBUG_SERIAL_ENABLED) {
        Serial.println("[" + device_id + "] " + message);
    }
}

inline void MelvinMemberController::debug_printf(const char* format, ...) {
    if (DEBUG_SERIAL_ENABLED) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        debug_print(String(buffer));
    }
}

// Utility function implementations
inline String control_type_to_string(ControlType type) {
    switch (type) {
        case CONTROL_BOOLEAN: return "Boolean";
        case CONTROL_CONTINUOUS: return "Continuous";
        case CONTROL_ROTARY: return "Rotary";
        case CONTROL_JOYSTICK_2AXIS: return "Joystick";
        case CONTROL_RGB_GROUP: return "RGB";
        default: return "Unknown";
    }
}

inline String network_status_to_string(NetworkStatus status) {
    switch (status) {
        case NET_DISCONNECTED: return "Disconnected";
        case NET_CONNECTING: return "Connecting";
        case NET_CONNECTED: return "Connected";
        case NET_REGISTERED: return "Registered";
        case NET_ERROR: return "Error";
        default: return "Unknown";
    }
}

inline uint16_t map_analog_to_control(int analog_value, int analog_min, int analog_max) {
    return map(analog_value, analog_min, analog_max, CONTROL_VALUE_OFF, CONTROL_VALUE_MAX);
}

inline uint16_t apply_deadband(uint16_t current_value, uint16_t last_value, uint16_t deadband) {
    if (abs((int)current_value - (int)last_value) < deadband) {
        return last_value; // No change, within deadband
    }
    return current_value;
}

inline bool read_digital_debounced(int pin, unsigned long debounce_time) {
    static std::map<int, bool> last_states;
    static std::map<int, unsigned long> last_change_times;
    
    bool current_state = digitalRead(pin);
    bool last_state = last_states[pin];
    
    if (current_state != last_state) {
        last_change_times[pin] = millis();
    }
    
    if (millis() - last_change_times[pin] > debounce_time) {
        last_states[pin] = current_state;
        return current_state;
    }
    
    return last_state;
}

#if MQTT_ENABLED
// MQTT Implementation Methods

inline bool MelvinMemberController::init_mqtt_client() {
    if (!mqtt_enabled) return false;
    
    debug_print("Initializing MQTT client...");
    
    // Attempt initial connection
    if (mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, mqtt_client_id.c_str())) {
        mqtt_connected = true;
        debug_print("MQTT client initialized successfully");
        
        // Set up global message handler using subscribe with wildcard
        // This will catch all messages and route them appropriately
        mqtt_client.subscribe("#", [this](const char* topic, const char* payload) {
            this->handle_mqtt_message(String(topic), String(payload));
        });
        
        on_mqtt_connected();
        return true;
    } else {
        mqtt_connected = false;
        debug_print("MQTT initial connection failed");
        return false;
    }
}

inline void MelvinMemberController::shutdown_mqtt_client() {
    if (mqtt_connected) {
        // Publish disconnection status
        publish_device_status("disconnecting", "Device shutting down");
        
        // Unsubscribe from all custom topics
        for (auto& subscription : custom_subscriptions) {
            mqtt_client.unsubscribe(subscription.first);
        }
        custom_subscriptions.clear();
        
        mqtt_client.disconnect();
    }
    mqtt_connected = false;
}

inline void MelvinMemberController::mqtt_auto_reconnect() {
    if (!mqtt_enabled || mqtt_connected) return;
    
    unsigned long current_time = millis();
    if (current_time - last_mqtt_reconnect < MQTT_RECONNECT_INTERVAL) return;
    
    debug_print("Attempting MQTT reconnection...");
    if (mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, mqtt_client_id.c_str())) {
        mqtt_connected = true;
        debug_print("MQTT reconnected successfully");
        
        // Re-subscribe to global handler
        mqtt_client.subscribe("#", [this](const char* topic, const char* payload) {
            this->handle_mqtt_message(String(topic), String(payload));
        });
        
        // Call connected callback to re-establish subscriptions
        on_mqtt_connected();
        
        last_mqtt_reconnect = current_time;
    } else {
        mqtt_connected = false;
        debug_print("MQTT reconnection failed");
        last_mqtt_reconnect = current_time;
    }
}

inline bool MelvinMemberController::publish_control_event(const String& control_name, String event_type) {
    if (!mqtt_connected) return false;
    
    auto it = control_name_map.find(control_name);
    if (it == control_name_map.end()) return false;
    
    const MemberControl& control = controls[it->second];
    return publish_control_event(control.control_id, event_type);
}

inline bool MelvinMemberController::publish_control_event(uint16_t control_id, String event_type) {
    if (!mqtt_connected) return false;
    
    // Find control by ID
    MemberControl* control = nullptr;
    for (auto& ctrl : controls) {
        if (ctrl.control_id == control_id) {
            control = &ctrl;
            break;
        }
    }
    
    if (!control) return false;
    
    MQTTEvent event = create_mqtt_event(*control, event_type);
    
    // Build JSON payload
    DynamicJsonDocument doc(512);
    doc["device_id"] = event.device_id;
    doc["event_type"] = event.event_type;
    doc["control_id"] = event.control_id;
    doc["value"] = event.value;
    
    if (control->type == CONTROL_JOYSTICK_2AXIS || control->type == CONTROL_RGB_GROUP) {
        doc["value_y"] = event.value_y;
        if (control->type == CONTROL_RGB_GROUP) {
            doc["value_z"] = event.value_z;
        }
    }
    
    doc["timestamp"] = event.timestamp;
    if (event.metadata.length() > 0) {
        doc["metadata"] = event.metadata;
    }
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = build_control_topic(control_id, "events");
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::publish_device_status(String status, String details) {
    if (!mqtt_connected) return false;
    
    DynamicJsonDocument doc(256);
    doc["device_id"] = device_id;
    doc["status"] = status;
    doc["details"] = details;
    doc["timestamp"] = millis();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = build_device_topic("status");
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::publish_custom_event(String event_type, String payload) {
    if (!mqtt_connected) return false;
    
    String topic = build_device_topic("events/" + event_type);
    return mqtt_client.publish(topic, payload);
}

inline bool MelvinMemberController::subscribe_to_device_controls(String device_id_filter) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (device_id_filter.length() > 0) {
        topic = String(MQTT_TOPIC_DEVICES) + "/" + device_id_filter + "/controls/+";
    } else {
        topic = String(MQTT_TOPIC_CONTROLS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::subscribe_to_control_events(uint16_t control_id) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (control_id > 0) {
        topic = build_control_topic(control_id, "events");
    } else {
        topic = String(MQTT_TOPIC_EVENTS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::subscribe_to_custom_topic(String topic, MQTTSubscriptionCallback callback) {
    if (!mqtt_connected) return false;
    
    custom_subscriptions[topic] = callback;
    return mqtt_client.subscribe(topic);
}

inline bool MelvinMemberController::unsubscribe_from_topic(String topic) {
    if (!mqtt_connected) return false;
    
    auto it = custom_subscriptions.find(topic);
    if (it != custom_subscriptions.end()) {
        custom_subscriptions.erase(it);
    }
    
    mqtt_client.unsubscribe(topic);
    return true; // PicoMQTT unsubscribe returns void, so assume success
}

inline void MelvinMemberController::set_mqtt_routing_mode(MQTTRoutingMode mode) {
    routing_mode = mode;
    debug_print("MQTT routing mode set to: " + String((int)mode));
}

inline void MelvinMemberController::set_control_routing_callback(MQTTControlCallback callback) {
    control_routing_callback = callback;
}

inline void MelvinMemberController::add_control_id_mapping(uint16_t input_id, uint16_t output_id) {
    control_id_mappings[input_id] = output_id;
    debug_print("Added control mapping: " + String(input_id) + " -> " + String(output_id));
}

inline void MelvinMemberController::remove_control_id_mapping(uint16_t input_id) {
    auto it = control_id_mappings.find(input_id);
    if (it != control_id_mappings.end()) {
        control_id_mappings.erase(it);
        debug_print("Removed control mapping for: " + String(input_id));
    }
}

inline void MelvinMemberController::handle_mqtt_message(const String& topic, const String& payload) {
    debug_print("MQTT message received: " + topic);
    
    // Check for custom subscriptions first
    auto it = custom_subscriptions.find(topic);
    if (it != custom_subscriptions.end()) {
        it->second(topic, payload);
        return;
    }
    
    // Handle control commands
    if (topic.startsWith(String(MQTT_TOPIC_CONTROLS) + "/") || 
        topic.indexOf("/controls/") >= 0) {
        handle_control_command_message(payload);
        return;
    }
    
    // Handle device status messages
    if (topic.indexOf("/status") >= 0) {
        handle_device_status_message(payload);
        return;
    }
    
    // Handle generic events
    on_mqtt_custom_event(topic, payload);
}

inline void MelvinMemberController::handle_control_command_message(const String& payload) {
    MQTTControlCommand command = parse_control_command(payload);
    
    // Check if this command is targeted at this device
    if (command.target_device_id.length() > 0 && command.target_device_id != device_id) {
        return; // Not for this device
    }
    
    // Process routing if enabled
    process_control_routing(command);
    
    // Call virtual handler
    on_mqtt_control_command(command);
}

inline void MelvinMemberController::handle_device_status_message(const String& payload) {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        return;
    }
    
    String device_id = doc["device_id"];
    String status = doc["status"];
    String details = doc["details"];
    
    on_mqtt_device_status(device_id, status, details);
}

inline void MelvinMemberController::process_control_routing(const MQTTControlCommand& command) {
    if (routing_mode == MQTT_ROUTE_NONE) return;
    
    // Try custom callback first
    if (control_routing_callback && control_routing_callback(command)) {
        return; // Custom handler processed it
    }
    
    // Handle direct routing
    if (routing_mode == MQTT_ROUTE_DIRECT) {
        // Find control and update its value
        for (auto& control : controls) {
            if (control.control_id == command.control_id) {
                control.current_value = command.value;
                control.current_value_y = command.value_y;
                control.current_value_z = command.value_z;
                control.value_changed = true;
                break;
            }
        }
    }
    
    // Handle mapped routing
    if (routing_mode == MQTT_ROUTE_MAPPED) {
        auto it = control_id_mappings.find(command.control_id);
        if (it != control_id_mappings.end()) {
            uint16_t output_id = it->second;
            for (auto& control : controls) {
                if (control.control_id == output_id) {
                    control.current_value = command.value;
                    control.current_value_y = command.value_y;
                    control.current_value_z = command.value_z;
                    control.value_changed = true;
                    break;
                }
            }
        }
    }
}

inline String MelvinMemberController::build_device_topic(String sub_topic) {
    return String(MQTT_TOPIC_DEVICES) + "/" + device_id + "/" + sub_topic;
}

inline String MelvinMemberController::build_control_topic(uint16_t control_id, String sub_topic) {
    String topic = String(MQTT_TOPIC_CONTROLS) + "/" + String(control_id);
    if (sub_topic.length() > 0) {
        topic += "/" + sub_topic;
    }
    return topic;
}

MQTTEvent MelvinMemberController::create_mqtt_event(const MemberControl& control, String event_type) {
    MQTTEvent event;
    event.device_id = device_id;
    event.event_type = event_type;
    event.control_id = control.control_id;
    event.value = control.current_value;
    event.value_y = control.current_value_y;
    event.value_z = control.current_value_z;
    event.timestamp = millis();
    
    // Add control name as metadata
    DynamicJsonDocument metadata(128);
    metadata["control_name"] = control.name;
    metadata["control_type"] = (int)control.type;
    serializeJson(metadata, event.metadata);
    
    return event;
}

MQTTControlCommand MelvinMemberController::parse_control_command(const String& payload) {
    MQTTControlCommand command;
    
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload) == DeserializationError::Ok) {
        command.target_device_id = doc["target_device_id"].as<String>();
        command.control_id = doc["control_id"];
        command.value = doc["value"];
        command.value_y = doc["value_y"];
        command.value_z = doc["value_z"];
        command.command_type = doc["command_type"].as<String>();
        command.source_device_id = doc["source_device_id"].as<String>();
        command.timestamp = doc["timestamp"];
    }
    
    return command;
}

inline void MelvinMemberController::auto_publish_changed_controls() {
    if (!mqtt_connected) return;
    
    for (const auto& control : controls) {
        if (control.value_changed && control.control_id > 0) {
            publish_control_event(control.control_id, "control_change");
        }
    }
}

// Default virtual method implementations
inline void MelvinMemberController::on_mqtt_control_command(const MQTTControlCommand& command) {
    debug_print("MQTT control command received: Control " + String(command.control_id) + 
                " = " + String(command.value));
}

inline void MelvinMemberController::on_mqtt_device_status(const String& device_id, const String& status, const String& details) {
    debug_print("Device status: " + device_id + " - " + status + " (" + details + ")");
}

inline void MelvinMemberController::on_mqtt_custom_event(const String& topic, const String& payload) {
    debug_print("Custom MQTT event: " + topic);
}

inline void MelvinMemberController::on_mqtt_connected() {
    debug_print("MQTT connection established");
    publish_device_status("connected", "Device online and ready");
}

inline void MelvinMemberController::on_mqtt_disconnected() {
    debug_print("MQTT connection lost");
}

#endif // MQTT_ENABLED
#endif // MELVIN_MEMBER_CONTROLLER_H
