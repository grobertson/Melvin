/**
 * Melvin Abstract Member Controller – Example Implementation
 *
 * This file demonstrates a complete example of an abstract member controller,
 * designed as a reusable template for building specific controllers within the Melvin system.
 * 
 * Purpose:
 * - Provides a ready-to-customize base with all essential logic and structure.
 * - Ensures consistent design and easier implementation of new member controllers.
 * 
 * How to use this template to create your own controller:
 *   1. Duplicate this entire directory for your new device.
 *   2. Edit 'config.h' to match your device’s properties, pins, and settings.
 *   3. In 'main.cpp', override the virtual functions to add your specific control logic.
 *   4. Implement your device’s unique hardware behavior and features.
 * 
 * This template handles:
 *   - Hardware setup and safe initialization
 *   - Input reading with debouncing and analog sampling
 *   - Output control and state management
 *   - MQTT and network registration (if enabled)
 *   - Status/error handling and feedback
 * 
 * Start by reading through the main class below and follow the instructions in each section.
 */

#include <Arduino.h>
#include "melvin_member.h"
#include "config.h"

// Global instance of the member controller
MelvinMemberController* g_controller = nullptr;

/**
 * Concrete implementation of the abstract member controller
 * This demonstrates how to implement the virtual functions
 */
class ConcreteAbstractController : public MelvinMemberController {
private:
    // Hardware state tracking
    bool button_1_last_state;
    bool button_2_last_state;
    bool switch_1_last_state;
    bool switch_2_last_state;
    
    // Timing for debouncing
    unsigned long button_1_last_change;
    unsigned long button_2_last_change;
    unsigned long switch_1_last_change;
    unsigned long switch_2_last_change;
    
public:
    ConcreteAbstractController() 
        : MelvinMemberController(DEVICE_ID, DEVICE_NAME, (DeviceType)DEVICE_TYPE),
          button_1_last_state(false), button_2_last_state(false),
          switch_1_last_state(false), switch_2_last_state(false),
          button_1_last_change(0), button_2_last_change(0),
          switch_1_last_change(0), switch_2_last_change(0) {
    }
    
#if MQTT_ENABLED
    /**
     * Override MQTT event handlers to demonstrate usage
     */
    void on_mqtt_connected() override {
        MelvinMemberController::on_mqtt_connected(); // Call base implementation
        
        // Subscribe to specific control events for demonstration
        subscribe_to_control_events(0); // Subscribe to all control events
        subscribe_to_device_controls(""); // Subscribe to all device controls
        
        // Set up control routing for emergency response
        set_mqtt_routing_mode(MQTT_ROUTE_MAPPED);
        
        // Example: Map emergency button from another device to our relay 1
        // This would route emergency button (ID 100) to relay 1 (ID varies)
        // add_control_id_mapping(100, relay_1_control_id);
        
        debug_print("MQTT subscriptions configured");
    }
    
    void on_mqtt_control_command(const MQTTControlCommand& command) override {
        debug_print("Received MQTT control command for control " + String(command.control_id) + 
                   " from " + command.source_device_id);
        
        // Example: React to specific control commands
        if (command.command_type == "emergency_stop") {
            // Override all outputs to safe state
            set_control_value("Relay 1", 0);
            set_control_value("Relay 2", 0);
            debug_print("Emergency stop activated via MQTT");
        }
        
        // Call base implementation for automatic routing
        MelvinMemberController::on_mqtt_control_command(command);
    }
    
    void on_mqtt_device_status(const String& device_id, const String& status, const String& details) override {
        if (status == "error" || status == "emergency") {
            debug_print("Device " + device_id + " reported " + status + ": " + details);
            // Could trigger local emergency responses here
        }
    }
    
    void on_mqtt_custom_event(const String& topic, const String& payload) override {
        if (topic.indexOf("/alerts/") >= 0) {
            debug_print("Alert received: " + payload);
            // Flash status LED or trigger buzzer
        }
    }
#endif
    
    /**
     * Initialize hardware-specific components
     */
    void setup_hardware() override {
        // Configure digital input pins
        pinMode(PIN_BUTTON_1, INPUT_PULLUP);
        pinMode(PIN_BUTTON_2, INPUT_PULLUP);
        pinMode(PIN_SWITCH_1, INPUT_PULLUP);
        pinMode(PIN_SWITCH_2, INPUT_PULLUP);
        
        // Configure analog input pins (already set as inputs by default)
        analogReadResolution(ADC_RESOLUTION);
        
        // Configure output pins
        pinMode(PIN_STATUS_LED, OUTPUT);
        pinMode(PIN_ERROR_LED, OUTPUT);
        pinMode(PIN_RELAY_1, OUTPUT);
        pinMode(PIN_RELAY_2, OUTPUT);
        
        // Initialize outputs to safe state
        digitalWrite(PIN_STATUS_LED, LOW);
        digitalWrite(PIN_ERROR_LED, LOW);
        digitalWrite(PIN_RELAY_1, LOW);
        digitalWrite(PIN_RELAY_2, LOW);
        
        debug_print("Hardware initialized");
    }
    
    /**
     * Read physical inputs and update control values
     * This is called regularly by the main update loop
     */
    void read_inputs() override {
        unsigned long current_time = millis();
        
        // Read digital inputs with debouncing
        bool button_1_state = !digitalRead(PIN_BUTTON_1); // Inverted due to pullup
        bool button_2_state = !digitalRead(PIN_BUTTON_2);
        bool switch_1_state = !digitalRead(PIN_SWITCH_1);
        bool switch_2_state = !digitalRead(PIN_SWITCH_2);
        
        // Debounce button 1
        if (button_1_state != button_1_last_state) {
            if (current_time - button_1_last_change > 50) { // 50ms debounce
                set_control_value("Emergency Button", button_1_state ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF);
                button_1_last_state = button_1_state;
            }
            button_1_last_change = current_time;
        }
        
        // Debounce button 2
        if (button_2_state != button_2_last_state) {
            if (current_time - button_2_last_change > 50) {
                set_control_value("Auxiliary Button", button_2_state ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF);
                button_2_last_state = button_2_state;
            }
            button_2_last_change = current_time;
        }
        
        // Debounce switch 1
        if (switch_1_state != switch_1_last_state) {
            if (current_time - switch_1_last_change > 50) {
                set_control_value("Main Switch", switch_1_state ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF);
                switch_1_last_state = switch_1_state;
            }
            switch_1_last_change = current_time;
        }
        
        // Debounce switch 2
        if (switch_2_state != switch_2_last_state) {
            if (current_time - switch_2_last_change > 50) {
                set_control_value("Auxiliary Switch", switch_2_state ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF);
                switch_2_last_state = switch_2_state;
            }
            switch_2_last_change = current_time;
        }
        
        // Read analog inputs (potentiometers)
        int pot_1_raw = 0;
        int pot_2_raw = 0;
        
        // Average multiple readings for stability
        for (int i = 0; i < ADC_SAMPLES; i++) {
            pot_1_raw += analogRead(PIN_POT_1);
            pot_2_raw += analogRead(PIN_POT_2);
            delayMicroseconds(100); // Small delay between samples
        }
        
        pot_1_raw /= ADC_SAMPLES;
        pot_2_raw /= ADC_SAMPLES;
        
        // Convert to control values and apply deadband
        uint16_t pot_1_value = map_analog_to_control(pot_1_raw);
        uint16_t pot_2_value = map_analog_to_control(pot_2_raw);
        
        // Get current values for deadband comparison
        uint16_t current_pot_1, current_pot_2, dummy_y, dummy_z;
        get_control_value("Brightness Control", current_pot_1, dummy_y, dummy_z);
        get_control_value("Volume Control", current_pot_2, dummy_y, dummy_z);
        
        // Apply deadband to prevent noise
        pot_1_value = apply_deadband(pot_1_value, current_pot_1, 100);
        pot_2_value = apply_deadband(pot_2_value, current_pot_2, 100);
        
        set_control_value("Brightness Control", pot_1_value);
        set_control_value("Volume Control", pot_2_value);
        
        // Read joystick (2-axis analog)
        int joystick_x_raw = 0;
        int joystick_y_raw = 0;
        
        for (int i = 0; i < ADC_SAMPLES; i++) {
            joystick_x_raw += analogRead(PIN_JOYSTICK_X);
            joystick_y_raw += analogRead(PIN_JOYSTICK_Y);
            delayMicroseconds(100);
        }
        
        joystick_x_raw /= ADC_SAMPLES;
        joystick_y_raw /= ADC_SAMPLES;
        
        uint16_t joystick_x = map_analog_to_control(joystick_x_raw);
        uint16_t joystick_y = map_analog_to_control(joystick_y_raw);
        
        // Apply deadband to joystick
        uint16_t current_x, current_y;
        get_control_value("Spotlight Direction", current_x, current_y, dummy_z);
        joystick_x = apply_deadband(joystick_x, current_x, 200);
        joystick_y = apply_deadband(joystick_y, current_y, 200);
        
        set_control_value("Spotlight Direction", joystick_x, joystick_y);
    }
    
    /**
     * Write outputs based on control values (if this device has outputs)
     * This is called after network communication
     */
    void write_outputs() override {
        // This abstract controller primarily reads inputs, but you could
        // implement output control here. For example:
        
        uint16_t relay_1_value, relay_2_value, dummy_y, dummy_z;
        
        // Check if we have output controls (would be set by master or other devices)
        if (get_control_value("Relay 1", relay_1_value, dummy_y, dummy_z)) {
            digitalWrite(PIN_RELAY_1, relay_1_value > 0 ? HIGH : LOW);
        }
        
        if (get_control_value("Relay 2", relay_2_value, dummy_y, dummy_z)) {
            digitalWrite(PIN_RELAY_2, relay_2_value > 0 ? HIGH : LOW);
        }
    }
    
    /**
     * Handle errors by updating status LEDs, buzzers, etc.
     */
    void handle_error(const String& error) override {
        debug_print("ERROR: " + error);
        
        // Flash error LED rapidly
        static unsigned long last_error_flash = 0;
        if (millis() - last_error_flash > 100) {
            digitalWrite(PIN_ERROR_LED, !digitalRead(PIN_ERROR_LED));
            last_error_flash = millis();
        }
        
        // Could add buzzer, display updates, etc. here
    }
};

/**
 * Arduino setup function
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("========================================");
    Serial.println("Melvin Abstract Member Controller");
    Serial.println("Device: " + String(DEVICE_NAME));
    Serial.println("ID: " + String(DEVICE_ID));
    Serial.println("Type: " + String(DEVICE_TYPE));
    Serial.println("========================================");
    
    // Create controller instance
    g_controller = new ConcreteAbstractController();
    
    // Initialize the controller
    if (!g_controller->initialize()) {
        Serial.println("FATAL: Controller initialization failed!");
        while(1) {
            digitalWrite(PIN_ERROR_LED, HIGH);
            delay(200);
            digitalWrite(PIN_ERROR_LED, LOW);
            delay(200);
        }
    }
    
    Serial.println("Controller initialized successfully");
    Serial.println("Starting main loop...");
}

/**
 * Arduino main loop
 */
void loop() {
    if (g_controller) {
        g_controller->update();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

// Implementation of MelvinMemberController class methods
// Note: In a real implementation, these would be in a separate .cpp file

MelvinMemberController::MelvinMemberController(const String& id, const String& name, DeviceType type) 
    : network_status(NET_DISCONNECTED), last_heartbeat_time(0), last_control_update_time(0),
      last_status_led_time(0), status_led_state(false),
      device_id(id), device_name(name), device_type(type) {
#if MQTT_ENABLED
    mqtt_connected = false;
    mqtt_enabled = true;
    last_mqtt_reconnect = 0;
    mqtt_client_id = String(MQTT_CLIENT_ID_PREFIX) + "_" + device_id;
    routing_mode = MQTT_ROUTE_NONE;
#endif
}

MelvinMemberController::~MelvinMemberController() {
    shutdown();
}

bool MelvinMemberController::initialize() {
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

void MelvinMemberController::update() {
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

void MelvinMemberController::shutdown() {
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

bool MelvinMemberController::add_control(const String& name, uint16_t control_number, ControlType type) {
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

bool MelvinMemberController::set_control_value(const String& name, uint16_t value, uint16_t value_y, uint16_t value_z) {
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

bool MelvinMemberController::get_control_value(const String& name, uint16_t& value, uint16_t& value_y, uint16_t& value_z) {
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

bool MelvinMemberController::connect_to_wifi() {
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

bool MelvinMemberController::register_with_master() {
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

bool MelvinMemberController::send_heartbeat() {
    return register_with_master(); // Re-registration serves as heartbeat
}

bool MelvinMemberController::create_control_on_master(MemberControl& control) {
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

bool MelvinMemberController::update_control_on_master(const MemberControl& control) {
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

void MelvinMemberController::update_status_led() {
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

void MelvinMemberController::debug_print(const String& message) {
    if (DEBUG_SERIAL_ENABLED) {
        Serial.println("[" + device_id + "] " + message);
    }
}

void MelvinMemberController::debug_printf(const char* format, ...) {
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
String control_type_to_string(ControlType type) {
    switch (type) {
        case CONTROL_BOOLEAN: return "Boolean";
        case CONTROL_CONTINUOUS: return "Continuous";
        case CONTROL_ROTARY: return "Rotary";
        case CONTROL_JOYSTICK_2AXIS: return "Joystick";
        case CONTROL_RGB_GROUP: return "RGB";
        default: return "Unknown";
    }
}

String network_status_to_string(NetworkStatus status) {
    switch (status) {
        case NET_DISCONNECTED: return "Disconnected";
        case NET_CONNECTING: return "Connecting";
        case NET_CONNECTED: return "Connected";
        case NET_REGISTERED: return "Registered";
        case NET_ERROR: return "Error";
        default: return "Unknown";
    }
}

uint16_t map_analog_to_control(int analog_value, int analog_min, int analog_max) {
    return map(analog_value, analog_min, analog_max, CONTROL_VALUE_OFF, CONTROL_VALUE_MAX);
}

uint16_t apply_deadband(uint16_t current_value, uint16_t last_value, uint16_t deadband) {
    if (abs((int)current_value - (int)last_value) < deadband) {
        return last_value; // No change, within deadband
    }
    return current_value;
}

bool read_digital_debounced(int pin, unsigned long debounce_time) {
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

bool MelvinMemberController::init_mqtt_client() {
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

void MelvinMemberController::shutdown_mqtt_client() {
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

void MelvinMemberController::mqtt_auto_reconnect() {
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

bool MelvinMemberController::publish_control_event(const String& control_name, String event_type) {
    if (!mqtt_connected) return false;
    
    auto it = control_name_map.find(control_name);
    if (it == control_name_map.end()) return false;
    
    const MemberControl& control = controls[it->second];
    return publish_control_event(control.control_id, event_type);
}

bool MelvinMemberController::publish_control_event(uint16_t control_id, String event_type) {
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

bool MelvinMemberController::publish_device_status(String status, String details) {
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

bool MelvinMemberController::publish_custom_event(String event_type, String payload) {
    if (!mqtt_connected) return false;
    
    String topic = build_device_topic("events/" + event_type);
    return mqtt_client.publish(topic, payload);
}

bool MelvinMemberController::subscribe_to_device_controls(String device_id_filter) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (device_id_filter.length() > 0) {
        topic = String(MQTT_TOPIC_DEVICES) + "/" + device_id_filter + "/controls/+";
    } else {
        topic = String(MQTT_TOPIC_CONTROLS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

bool MelvinMemberController::subscribe_to_control_events(uint16_t control_id) {
    if (!mqtt_connected) return false;
    
    String topic;
    if (control_id > 0) {
        topic = build_control_topic(control_id, "events");
    } else {
        topic = String(MQTT_TOPIC_EVENTS) + "/+";
    }
    
    return mqtt_client.subscribe(topic);
}

bool MelvinMemberController::subscribe_to_custom_topic(String topic, MQTTSubscriptionCallback callback) {
    if (!mqtt_connected) return false;
    
    custom_subscriptions[topic] = callback;
    return mqtt_client.subscribe(topic);
}

bool MelvinMemberController::unsubscribe_from_topic(String topic) {
    if (!mqtt_connected) return false;
    
    auto it = custom_subscriptions.find(topic);
    if (it != custom_subscriptions.end()) {
        custom_subscriptions.erase(it);
    }
    
    mqtt_client.unsubscribe(topic);
    return true; // PicoMQTT unsubscribe returns void, so assume success
}

void MelvinMemberController::set_mqtt_routing_mode(MQTTRoutingMode mode) {
    routing_mode = mode;
    debug_print("MQTT routing mode set to: " + String((int)mode));
}

void MelvinMemberController::set_control_routing_callback(MQTTControlCallback callback) {
    control_routing_callback = callback;
}

void MelvinMemberController::add_control_id_mapping(uint16_t input_id, uint16_t output_id) {
    control_id_mappings[input_id] = output_id;
    debug_print("Added control mapping: " + String(input_id) + " -> " + String(output_id));
}

void MelvinMemberController::remove_control_id_mapping(uint16_t input_id) {
    auto it = control_id_mappings.find(input_id);
    if (it != control_id_mappings.end()) {
        control_id_mappings.erase(it);
        debug_print("Removed control mapping for: " + String(input_id));
    }
}

void MelvinMemberController::handle_mqtt_message(const String& topic, const String& payload) {
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

void MelvinMemberController::handle_control_command_message(const String& payload) {
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

void MelvinMemberController::handle_device_status_message(const String& payload) {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        return;
    }
    
    String device_id = doc["device_id"];
    String status = doc["status"];
    String details = doc["details"];
    
    on_mqtt_device_status(device_id, status, details);
}

void MelvinMemberController::process_control_routing(const MQTTControlCommand& command) {
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

String MelvinMemberController::build_device_topic(String sub_topic) {
    return String(MQTT_TOPIC_DEVICES) + "/" + device_id + "/" + sub_topic;
}

String MelvinMemberController::build_control_topic(uint16_t control_id, String sub_topic) {
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

void MelvinMemberController::auto_publish_changed_controls() {
    if (!mqtt_connected) return;
    
    for (const auto& control : controls) {
        if (control.value_changed && control.control_id > 0) {
            publish_control_event(control.control_id, "control_change");
        }
    }
}

// Default virtual method implementations
void MelvinMemberController::on_mqtt_control_command(const MQTTControlCommand& command) {
    debug_print("MQTT control command received: Control " + String(command.control_id) + 
                " = " + String(command.value));
}

void MelvinMemberController::on_mqtt_device_status(const String& device_id, const String& status, const String& details) {
    debug_print("Device status: " + device_id + " - " + status + " (" + details + ")");
}

void MelvinMemberController::on_mqtt_custom_event(const String& topic, const String& payload) {
    debug_print("Custom MQTT event: " + topic);
}

void MelvinMemberController::on_mqtt_connected() {
    debug_print("MQTT connection established");
    publish_device_status("connected", "Device online and ready");
}

void MelvinMemberController::on_mqtt_disconnected() {
    debug_print("MQTT connection lost");
}


#endif // MQTT_ENABLED
