/**
 * OBD-II Interface Controller Implementation
 * 
 * This controller connects to Bluetooth OBD-II adapters to retrieve vehicle
 * diagnostic data and proxy selected parameters to the Melvin master controller.
 * 
 * Features:
 * - Bluetooth ELM327 adapter support
 * - Configurable parameter selection and update rates
 * - Test mode for development without vehicle connection
 * - Automatic error recovery and reconnection
 * - Full integration with Melvin control network
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <cstdarg>
#include "melvin_member.h"
#include "config.h"

// Global instance of the member controller
MelvinMemberController* g_controller = nullptr;

/**
 * OBD-II Parameter Definition
 */
struct OBDIIParameter {
    uint8_t pid;                    // OBD-II Parameter ID
    const char* name;               // Human-readable name
    uint8_t update_group;           // Update frequency group (0=fast, 1=medium, 2=slow)
    bool enabled;                   // Enable/disable this parameter
    uint16_t scaling_factor;        // Scaling for Melvin 16-bit values
    uint16_t current_value;         // Current scaled value
    unsigned long last_update_time; // Last successful update time
    uint8_t error_count;           // Consecutive error count
    
    OBDIIParameter(uint8_t p, const char* n, uint8_t g, bool e = true, uint16_t s = 1) 
        : pid(p), name(n), update_group(g), enabled(e), scaling_factor(s), 
          current_value(0), last_update_time(0), error_count(0) {}
};

/**
 * Test Mode Data Generator
 */
class OBDIITestMode {
private:
    uint16_t simulated_rpm;
    uint16_t simulated_speed;
    uint16_t simulated_coolant_temp;
    uint16_t simulated_fuel_level;
    bool engine_running;
    unsigned long scenario_start_time;
    
public:
    OBDIITestMode() : simulated_rpm(0), simulated_speed(0), simulated_coolant_temp(20),
                      simulated_fuel_level(16384), engine_running(false), scenario_start_time(0) {}
    
    void start_engine_scenario() {
        engine_running = true;
        scenario_start_time = millis();
        simulated_rpm = TEST_RPM_IDLE;
        simulated_coolant_temp = 20; // Cold start
    }
    
    void update_scenario() {
        if (!engine_running) return;
        
        unsigned long elapsed = millis() - scenario_start_time;
        
        // Simulate warm-up period
        if (elapsed < 60000) { // First minute
            simulated_coolant_temp = map(elapsed, 0, 60000, 20, TEST_TEMP_NORMAL);
        } else {
            simulated_coolant_temp = TEST_TEMP_NORMAL;
        }
        
        // Simulate driving pattern
        unsigned long cycle_time = elapsed % 30000; // 30 second cycles
        if (cycle_time < 10000) {
            // Acceleration phase
            simulated_rpm = map(cycle_time, 0, 10000, TEST_RPM_IDLE, 3000);
            simulated_speed = map(cycle_time, 0, 10000, 0, 60);
        } else if (cycle_time < 20000) {
            // Cruise phase
            simulated_rpm = 2000;
            simulated_speed = 60;
        } else {
            // Deceleration phase
            simulated_rpm = map(cycle_time, 20000, 30000, 2000, TEST_RPM_IDLE);
            simulated_speed = map(cycle_time, 20000, 30000, 60, 0);
        }
        
        // Slowly consume fuel
        if (elapsed % 10000 == 0) {
            simulated_fuel_level = (simulated_fuel_level > 100) ? simulated_fuel_level - 100 : 0;
        }
    }
    
    uint16_t get_rpm() { return simulated_rpm; }
    uint16_t get_speed() { return simulated_speed; }
    uint16_t get_coolant_temp() { return simulated_coolant_temp; }
    uint16_t get_fuel_level() { return simulated_fuel_level; }
    uint16_t get_throttle_position() { 
        return engine_running ? map(simulated_rpm, TEST_RPM_IDLE, 4000, 0, 20000) : 0; 
    }
    uint16_t get_engine_load() { 
        return engine_running ? map(simulated_rpm, TEST_RPM_IDLE, 4000, 2000, 25000) : 0; 
    }
};

/**
 * OBD-II Interface Controller Implementation
 */
class OBDIIController : public MelvinMemberController {
private:
    // Bluetooth communication
    BluetoothSerial bluetooth;
    bool bluetooth_connected;
    unsigned long last_bluetooth_attempt;
    
    // OBD-II parameters and state
    std::vector<OBDIIParameter> parameters;
    bool obd_initialized;
    unsigned long last_fast_update;
    unsigned long last_medium_update;
    unsigned long last_slow_update;
    uint8_t init_step;
    uint8_t consecutive_errors;
    
    // Test mode
    OBDIITestMode test_mode;
    bool test_mode_active;
    bool test_mode_button_last_state;
    unsigned long test_mode_button_last_change;
    
    // Status tracking
    uint16_t active_dtc_count;
    
public:
    OBDIIController() 
        : MelvinMemberController(DEVICE_ID, DEVICE_NAME, (DeviceType)DEVICE_TYPE),
          bluetooth_connected(false), last_bluetooth_attempt(0),
          obd_initialized(false), last_fast_update(0), last_medium_update(0), 
          last_slow_update(0), init_step(0), consecutive_errors(0),
          test_mode_active(TEST_MODE_DEFAULT_STATE), test_mode_button_last_state(false),
          test_mode_button_last_change(0), active_dtc_count(0) {
        
        // Initialize OBD-II parameters
        setup_obd_parameters();
    }
    
    /**
     * Initialize hardware-specific components
     */
    void setup_hardware() override {
        // Configure status LEDs
        pinMode(PIN_STATUS_LED, OUTPUT);
        pinMode(PIN_ERROR_LED, OUTPUT);
        pinMode(PIN_BLUETOOTH_LED, OUTPUT);
        pinMode(PIN_TEST_MODE_BUTTON, INPUT_PULLUP);
        
        // Initialize outputs to safe state
        digitalWrite(PIN_STATUS_LED, LOW);
        digitalWrite(PIN_ERROR_LED, LOW);
        digitalWrite(PIN_BLUETOOTH_LED, LOW);
        
        // Initialize Bluetooth
        bluetooth.begin(DEVICE_NAME);
        
        debug_print("OBD-II Interface hardware initialized");
    }
    
    /**
     * Read OBD-II data and update control values
     */
    void read_inputs() override {
        unsigned long current_time = millis();
        
        // Check test mode button
        check_test_mode_button();
        
        if (test_mode_active) {
            // Update test mode scenario
            test_mode.update_scenario();
            update_test_mode_controls();
        } else {
            // Handle real OBD-II communication
            if (!bluetooth_connected) {
                attempt_bluetooth_connection();
                return;
            }
            
            if (!obd_initialized) {
                if (!initialize_obd_adapter()) {
                    return;
                }
            }
            
            // Update parameters based on their update groups
            if (current_time - last_fast_update >= ODBII_FAST_UPDATE_INTERVAL) {
                update_parameter_group(ODBII_UPDATE_GROUP_FAST);
                last_fast_update = current_time;
            }
            
            if (current_time - last_medium_update >= ODBII_MEDIUM_UPDATE_INTERVAL) {
                update_parameter_group(ODBII_UPDATE_GROUP_MEDIUM);
                last_medium_update = current_time;
            }
            
            if (current_time - last_slow_update >= ODBII_SLOW_UPDATE_INTERVAL) {
                update_parameter_group(ODBII_UPDATE_GROUP_SLOW);
                last_slow_update = current_time;
            }
        }
        
        // Update status LEDs
        update_bluetooth_led();
    }
    
    /**
     * Write outputs based on control values
     */
    void write_outputs() override {
        // This controller primarily reads data, but could implement
        // output controls for test mode activation, etc.
    }
    
    /**
     * Handle errors with appropriate status indication
     */
    void handle_error(const String& error) override {
        debug_print("OBD-II ERROR: " + error);
        
        consecutive_errors++;
        
        // Flash error LED rapidly
        static unsigned long last_error_flash = 0;
        if (millis() - last_error_flash > 100) {
            digitalWrite(PIN_ERROR_LED, !digitalRead(PIN_ERROR_LED));
            last_error_flash = millis();
        }
        
        // Reset connection if too many consecutive errors
        if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            debug_print("Too many consecutive errors, resetting connection");
            reset_obd_connection();
            consecutive_errors = 0;
        }
    }
    
private:
    /**
     * Set up the list of OBD-II parameters to monitor
     */
    void setup_obd_parameters() {
        // Fast update parameters (high priority)
        parameters.push_back(OBDIIParameter(0x0C, "Engine RPM", ODBII_UPDATE_GROUP_FAST, true, 4));
        parameters.push_back(OBDIIParameter(0x0D, "Vehicle Speed", ODBII_UPDATE_GROUP_FAST, true, 128));
        parameters.push_back(OBDIIParameter(0x04, "Engine Load", ODBII_UPDATE_GROUP_FAST, true, 327));
        
        // Medium update parameters
        parameters.push_back(OBDIIParameter(0x05, "Coolant Temperature", ODBII_UPDATE_GROUP_MEDIUM, true, 128));
        parameters.push_back(OBDIIParameter(0x11, "Throttle Position", ODBII_UPDATE_GROUP_MEDIUM, true, 327));
        parameters.push_back(OBDIIParameter(0x0F, "Intake Air Temperature", ODBII_UPDATE_GROUP_MEDIUM, true, 128));
        
        // Slow update parameters
        parameters.push_back(OBDIIParameter(0x2F, "Fuel Level", ODBII_UPDATE_GROUP_SLOW, true, 327));
        parameters.push_back(OBDIIParameter(0x01, "DTC Status", ODBII_UPDATE_GROUP_SLOW, true, 1));
    }
    
    /**
     * Check test mode button state
     */
    void check_test_mode_button() {
        bool current_state = !digitalRead(PIN_TEST_MODE_BUTTON);
        unsigned long current_time = millis();
        
        if (current_state != test_mode_button_last_state) {
            if (current_time - test_mode_button_last_change > 50) { // Debounce
                if (current_state) { // Button pressed
                    test_mode_active = !test_mode_active;
                    set_control_value("Test Mode Active", test_mode_active ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF);
                    
                    if (test_mode_active) {
                        debug_print("Test mode activated");
                        test_mode.start_engine_scenario();
                    } else {
                        debug_print("Test mode deactivated");
                        reset_obd_connection(); // Reset to try real connection
                    }
                }
                test_mode_button_last_state = current_state;
            }
            test_mode_button_last_change = current_time;
        }
    }
    
    /**
     * Update controls with test mode data
     */
    void update_test_mode_controls() {
        set_control_value("Engine RPM", map(test_mode.get_rpm(), 0, 8000, 0, CONTROL_VALUE_MAX));
        set_control_value("Vehicle Speed", map(test_mode.get_speed(), 0, 255, 0, CONTROL_VALUE_MAX));
        set_control_value("Engine Load", test_mode.get_engine_load());
        set_control_value("Coolant Temperature", map(test_mode.get_coolant_temp(), -40, 215, 0, CONTROL_VALUE_MAX));
        set_control_value("Fuel Level", test_mode.get_fuel_level());
        set_control_value("Throttle Position", test_mode.get_throttle_position());
        set_control_value("OBD Connection Status", CONTROL_VALUE_BOOLEAN_ON); // Simulate connection
        set_control_value("DTC Count", 0); // No errors in test mode
    }
    
    /**
     * Attempt to connect to Bluetooth OBD-II adapter
     */
    void attempt_bluetooth_connection() {
        unsigned long current_time = millis();
        
        if (current_time - last_bluetooth_attempt < BLUETOOTH_RECONNECT_INTERVAL_MS) {
            return;
        }
        
        debug_print("Attempting Bluetooth OBD-II connection...");
        last_bluetooth_attempt = current_time;
        
        // Try to connect to known OBD-II adapter names
        const char* adapter_names[] = {"OBDII", "OBD-II", "ELM327", "OBDLink"};
        
        for (int i = 0; i < 4; i++) {
            debug_print("Trying to connect to: " + String(adapter_names[i]));
            
            if (bluetooth.connect(adapter_names[i])) {
                bluetooth_connected = true;
                obd_initialized = false;
                init_step = 0;
                consecutive_errors = 0;
                debug_print("Bluetooth connected to: " + String(adapter_names[i]));
                set_control_value("OBD Connection Status", CONTROL_VALUE_BOOLEAN_ON);
                return;
            }
        }
        
        debug_print("Bluetooth connection failed");
        set_control_value("OBD Connection Status", CONTROL_VALUE_OFF);
    }
    
    /**
     * Initialize OBD-II adapter with ELM327 commands
     */
    bool initialize_obd_adapter() {
        const char* init_commands[] = {
            "ATZ",      // Reset
            "ATE0",     // Echo off
            "ATL0",     // Linefeeds off
            "ATS0",     // Spaces off
            "ATH1",     // Headers on
            "ATSP0"     // Auto protocol detection
        };
        
        if (init_step < 6) {
            debug_print("Sending OBD init command: " + String(init_commands[init_step]));
            
            bluetooth.println(init_commands[init_step]);
            
            // Wait for response
            unsigned long start_time = millis();
            String response = "";
            
            while (millis() - start_time < 2000) { // 2 second timeout
                if (bluetooth.available()) {
                    char c = bluetooth.read();
                    if (c == '\r' || c == '\n') {
                        if (response.length() > 0) break;
                    } else {
                        response += c;
                    }
                }
                delay(10);
            }
            
            if (response.indexOf("OK") >= 0 || response.indexOf(">") >= 0) {
                init_step++;
                debug_print("OBD init step " + String(init_step) + " completed");
            } else {
                debug_print("OBD init step " + String(init_step) + " failed: " + response);
                return false;
            }
        }
        
        if (init_step >= 6) {
            obd_initialized = true;
            debug_print("OBD-II adapter initialized successfully");
            return true;
        }
        
        return false;
    }
    
    /**
     * Update parameters in a specific update group
     */
    void update_parameter_group(uint8_t group) {
        for (auto& param : parameters) {
            if (param.enabled && param.update_group == group) {
                if (query_obd_parameter(param)) {
                    update_control_from_parameter(param);
                    param.error_count = 0;
                } else {
                    param.error_count++;
                    if (param.error_count >= 3) {
                        debug_print("Parameter " + String(param.name) + " consistently failing");
                    }
                }
            }
        }
    }
    
    /**
     * Query a specific OBD-II parameter
     */
    bool query_obd_parameter(OBDIIParameter& param) {
        String command = "01" + String(param.pid, HEX);
        if (param.pid < 0x10) command = "010" + String(param.pid, HEX);
        
        bluetooth.println(command);
        
        unsigned long start_time = millis();
        String response = "";
        
        while (millis() - start_time < ODBII_RESPONSE_TIMEOUT_MS) {
            if (bluetooth.available()) {
                char c = bluetooth.read();
                if (c == '\r' || c == '\n') {
                    if (response.length() > 0) break;
                } else {
                    response += c;
                }
            }
            delay(1);
        }
        
        if (response.length() > 0 && response.indexOf("41") >= 0) {
            uint16_t raw_value = parse_obd_response(response, param.pid);
            param.current_value = scale_obd_value(raw_value, param);
            param.last_update_time = millis();
            return true;
        }
        
        return false;
    }
    
    /**
     * Parse OBD-II response and extract value
     */
    uint16_t parse_obd_response(const String& response, uint8_t pid) {
        // Find the data bytes after "41 [PID]"
        String pid_str = String(pid, HEX);
        if (pid < 0x10) pid_str = "0" + pid_str;
        
        int data_start = response.indexOf("41" + pid_str);
        if (data_start < 0) return 0;
        
        data_start += 4; // Skip "41XX"
        
        // Extract hex bytes and convert to integer
        String hex_data = response.substring(data_start);
        hex_data.replace(" ", "");
        
        uint16_t value = 0;
        
        // Different PIDs have different data formats
        switch (pid) {
            case 0x0C: // Engine RPM (2 bytes)
                if (hex_data.length() >= 4) {
                    value = (hex_str_to_int(hex_data.substring(0, 2)) << 8) + 
                            hex_str_to_int(hex_data.substring(2, 4));
                    value = value / 4; // RPM = (A*256 + B) / 4
                }
                break;
                
            case 0x0D: // Vehicle Speed (1 byte)
                if (hex_data.length() >= 2) {
                    value = hex_str_to_int(hex_data.substring(0, 2));
                }
                break;
                
            case 0x04: // Engine Load (1 byte)
            case 0x11: // Throttle Position (1 byte)
            case 0x2F: // Fuel Level (1 byte)
                if (hex_data.length() >= 2) {
                    value = (hex_str_to_int(hex_data.substring(0, 2)) * 100) / 255;
                }
                break;
                
            case 0x05: // Coolant Temperature (1 byte)
            case 0x0F: // Intake Air Temperature (1 byte)
                if (hex_data.length() >= 2) {
                    value = hex_str_to_int(hex_data.substring(0, 2)) - 40;
                }
                break;
                
            case 0x01: // DTC Status (4 bytes)
                if (hex_data.length() >= 8) {
                    uint8_t dtc_byte = hex_str_to_int(hex_data.substring(0, 2));
                    active_dtc_count = dtc_byte & 0x7F; // Lower 7 bits = DTC count
                    value = active_dtc_count;
                }
                break;
                
            default:
                if (hex_data.length() >= 2) {
                    value = hex_str_to_int(hex_data.substring(0, 2));
                }
                break;
        }
        
        return value;
    }
    
    /**
     * Convert hex string to integer
     */
    uint8_t hex_str_to_int(const String& hex_str) {
        uint8_t value = 0;
        for (int i = 0; i < hex_str.length(); i++) {
            char c = hex_str.charAt(i);
            value *= 16;
            if (c >= '0' && c <= '9') {
                value += c - '0';
            } else if (c >= 'A' && c <= 'F') {
                value += c - 'A' + 10;
            } else if (c >= 'a' && c <= 'f') {
                value += c - 'a' + 10;
            }
        }
        return value;
    }
    
    /**
     * Scale OBD-II raw value to Melvin control value range
     */
    uint16_t scale_obd_value(uint16_t raw_value, const OBDIIParameter& param) {
        switch (param.pid) {
            case 0x0C: // Engine RPM
                return map(raw_value, 0, 8000, 0, CONTROL_VALUE_MAX);
                
            case 0x0D: // Vehicle Speed
                return map(raw_value, 0, 255, 0, CONTROL_VALUE_MAX);
                
            case 0x04: // Engine Load
            case 0x11: // Throttle Position  
            case 0x2F: // Fuel Level
                return map(raw_value, 0, 100, 0, CONTROL_VALUE_MAX);
                
            case 0x05: // Coolant Temperature
            case 0x0F: // Intake Air Temperature
                return map(raw_value, -40, 215, 0, CONTROL_VALUE_MAX);
                
            case 0x01: // DTC Count
                return raw_value;
                
            default:
                return raw_value * param.scaling_factor;
        }
    }
    
    /**
     * Update control value from OBD-II parameter
     */
    void update_control_from_parameter(const OBDIIParameter& param) {
        switch (param.pid) {
            case 0x0C:
                set_control_value("Engine RPM", param.current_value);
                break;
            case 0x0D:
                set_control_value("Vehicle Speed", param.current_value);
                break;
            case 0x04:
                set_control_value("Engine Load", param.current_value);
                break;
            case 0x05:
                set_control_value("Coolant Temperature", param.current_value);
                break;
            case 0x11:
                set_control_value("Throttle Position", param.current_value);
                break;
            case 0x2F:
                set_control_value("Fuel Level", param.current_value);
                break;
            case 0x01:
                set_control_value("DTC Count", param.current_value);
                break;
        }
    }
    
    /**
     * Reset OBD-II connection
     */
    void reset_obd_connection() {
        debug_print("Resetting OBD-II connection");
        
        if (bluetooth_connected) {
            bluetooth.disconnect();
        }
        
        bluetooth_connected = false;
        obd_initialized = false;
        init_step = 0;
        
        set_control_value("OBD Connection Status", CONTROL_VALUE_OFF);
        
        // Clear all parameter values
        for (auto& param : parameters) {
            param.current_value = 0;
            param.error_count = 0;
        }
    }
    
    /**
     * Update Bluetooth status LED
     */
    void update_bluetooth_led() {
        static unsigned long last_bt_led_update = 0;
        static bool bt_led_state = false;
        
        if (millis() - last_bt_led_update >= LED_PATTERN_BLUETOOTH_CONNECTED) {
            if (bluetooth_connected || test_mode_active) {
                bt_led_state = !bt_led_state;
                digitalWrite(PIN_BLUETOOTH_LED, bt_led_state);
            } else {
                digitalWrite(PIN_BLUETOOTH_LED, LOW);
            }
            last_bt_led_update = millis();
        }
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
    Serial.println("Melvin OBD-II Interface Controller");
    Serial.println("Device: " + String(DEVICE_NAME));
    Serial.println("ID: " + String(DEVICE_ID));
    Serial.println("Type: " + String(DEVICE_TYPE));
    Serial.println("========================================");
    
    // Create controller instance
    g_controller = new OBDIIController();
    
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
    
    Serial.println("OBD-II Controller initialized successfully");
    Serial.println("Press test mode button to toggle test mode");
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

// ============================================================================
// Member Controller Base Class Implementation
// ============================================================================

MelvinMemberController::MelvinMemberController(const String& id, const String& name, DeviceType type) 
    : network_status(NET_DISCONNECTED), last_heartbeat_time(0), last_control_update_time(0),
      last_status_led_time(0), status_led_state(false),
      device_id(id), device_name(name), device_type(type) {
}

MelvinMemberController::~MelvinMemberController() {
    shutdown();
}

bool MelvinMemberController::initialize() {
    debug_print("Initializing member controller...");
    
    // Initialize hardware first
    setup_hardware();
    
    // Set up OBD-II specific controls
    if (!add_control("Engine RPM", 1, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Vehicle Speed", 2, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Engine Load", 3, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Coolant Temperature", 4, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Fuel Level", 5, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Throttle Position", 6, CONTROL_CONTINUOUS)) return false;
    if (!add_control("OBD Connection Status", 7, CONTROL_BOOLEAN)) return false;
    if (!add_control("DTC Count", 8, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Test Mode Active", 9, CONTROL_BOOLEAN)) return false;
    
    // Configuration controls
    if (!add_control("Fast Update Rate", 10, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Medium Update Rate", 11, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Slow Update Rate", 12, CONTROL_CONTINUOUS)) return false;
    if (!add_control("Enable Test Mode", 13, CONTROL_BOOLEAN)) return false;
    
    // Connect to WiFi and register with master
    if (!connect_to_wifi()) {
        handle_error("WiFi connection failed");
        return false;
    }
    
    if (!register_with_master()) {
        handle_error("Master registration failed");
        return false;
    }
    
    debug_print("OBD-II member controller initialized successfully");
    return true;
}

void MelvinMemberController::update() {
    unsigned long current_time = millis();
    
    // Update status LED
    update_status_led();
    
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        if (network_status > NET_DISCONNECTED) {
            network_status = NET_DISCONNECTED;
            debug_print("WiFi connection lost");
        }
        return;
    }
    
    // Read inputs at regular intervals
    if (current_time - last_control_update_time >= CONTROL_UPDATE_INTERVAL_MS) {
        read_inputs();
        
        // Send any changed control values to master
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
    debug_print("Connecting to WiFi: " + String(WIFI_SSID));
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < WIFI_CONNECT_TIMEOUT_MS) {
        network_status = NET_CONNECTING;
        delay(500);
        debug_print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        network_status = NET_CONNECTED;
        debug_print("WiFi connected! IP: " + WiFi.localIP().toString());
        return true;
    } else {
        network_status = NET_ERROR;
        debug_print("WiFi connection failed");
        return false;
    }
}

bool MelvinMemberController::register_with_master() {
    debug_print("Registering with master controller...");
    
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
        debug_print("Registration failed. Response code: " + String(response_code));
        http_client.end();
        return false;
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

// ============================================================================
// Utility Functions Implementation
// ============================================================================

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