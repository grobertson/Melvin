/**
 * Melvin Member Controller Base Class
 * 
 * This header defines the abstract base class that all member controllers
 * should inherit from or follow as a pattern. It provides standardized
 * communication, registration, and control management functionality.
 */

#ifndef MELVIN_MEMBER_H
#define MELVIN_MEMBER_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <functional>
#include "config.h"

#if MQTT_ENABLED
#include <PicoMQTT.h>
#endif

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
 * Network Status Enumeration
 */
enum NetworkStatus {
    NET_DISCONNECTED = 0,
    NET_CONNECTING,
    NET_CONNECTED,
    NET_REGISTERED,
    NET_ERROR
};

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

/**
 * Utility Functions
 */

// Convert control type to string for debugging
String control_type_to_string(ControlType type);

// Convert network status to string for debugging  
String network_status_to_string(NetworkStatus status);

// Map analog reading to control value range
uint16_t map_analog_to_control(int analog_value, int analog_min = 0, int analog_max = ADC_MAX_VALUE);

// Apply deadband to analog readings to reduce noise
uint16_t apply_deadband(uint16_t current_value, uint16_t last_value, uint16_t deadband = 50);

// Read digital input with debouncing
bool read_digital_debounced(int pin, unsigned long debounce_time = 50);

#endif // MELVIN_MEMBER_H