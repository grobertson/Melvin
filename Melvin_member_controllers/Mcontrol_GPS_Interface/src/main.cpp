/**
 * GPS Interface Controller for Melvin Vehicle System
 * 
 * This member controller interfaces with GPS hardware to provide position,
 * speed, and navigation data to the Melvin master controller. Includes test
 * mode for development without actual GPS hardware.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "melvin_member.h"
#include "config.h"

// GPS module instance
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

/**
 * GPS Controller Class
 * Handles GPS data collection, processing, and transmission to master controller
 */
class GPSController : public MelvinMemberController {
private:
    // GPS data variables
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double speed_kmh = 0.0;
    double course = 0.0;
    int satellites = 0;
    bool has_fix = false;
    bool gps_fix = false;
    
    // Test mode variables
    bool test_mode_active = false;
    int test_sentence_index = 0;
    unsigned long last_gps_update = 0;
    
    // Test NMEA sentences for development
    const char* test_nmea_sentences[6] = {
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
        "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39",
        "$GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71",
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48",
        "$GPGLL,4807.038,N,01131.000,E,123519,A,*21"
    };

public:
    GPSController() : MelvinMemberController("GPS_001", "GPS Interface Controller", DEVICE_SENSOR_INPUT) {
        Serial.println("GPS Controller initializing...");
    }

    /**
     * Initialize GPS hardware and status LEDs
     */
    void setup_hardware() override {
        Serial.println("Setting up GPS hardware...");
        
        // Initialize GPS serial communication
        gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        
        // Initialize GPIO pins for status LEDs
        pinMode(PIN_GPS_FIX_LED, OUTPUT);
        pinMode(PIN_DATA_LED, OUTPUT);
        pinMode(PIN_ERROR_LED, OUTPUT);
        pinMode(PIN_STATUS_LED, OUTPUT);
        
        // Initialize test mode switch
        pinMode(PIN_TEST_MODE, INPUT_PULLUP);
        
        // Turn off all LEDs initially
        digitalWrite(PIN_GPS_FIX_LED, LOW);
        digitalWrite(PIN_DATA_LED, LOW);
        digitalWrite(PIN_ERROR_LED, LOW);
        digitalWrite(PIN_STATUS_LED, LOW);
        
        // Register GPS controls with master controller
        add_control("Latitude", 0, CONTROL_CONTINUOUS);
        add_control("Longitude", 1, CONTROL_CONTINUOUS);
        add_control("Altitude", 2, CONTROL_CONTINUOUS);
        add_control("Speed", 3, CONTROL_CONTINUOUS);
        add_control("Course", 4, CONTROL_CONTINUOUS);
        add_control("Satellites", 5, CONTROL_CONTINUOUS);
        add_control("GPS Fix", 6, CONTROL_BOOLEAN);
        
        Serial.println("GPS hardware setup complete");
    }

    /**
     * Read GPS data and update control values
     */
    void read_inputs() override {
        static unsigned long last_test_update = 0;
        unsigned long current_time = millis();
        
        // Check test mode switch
        test_mode_active = (digitalRead(PIN_TEST_MODE) == LOW);
        
        if (test_mode_active) {
            // Generate test data every TEST_DATA_INTERVAL ms
            if (current_time - last_test_update >= TEST_DATA_INTERVAL) {
                generate_test_data();
                last_test_update = current_time;
            }
        } else {
            // Read real GPS data
            read_real_gps_data();
        }
        
        // Update control values based on GPS data
        update_control_values();
        
        // Update status LEDs
        update_status_leds();
        
        // Check for GPS timeout
        if (current_time - last_gps_update > GPS_TIMEOUT) {
            has_fix = false;
            gps_fix = false;
        }
    }

    /**
     * Write outputs (GPS controller is input-only)
     */
    void write_outputs() override {
        // GPS controller is input-only, no outputs to write
    }

    /**
     * Handle errors by flashing error LED
     */
    void handle_error(const String& error) override {
        Serial.println("GPS Controller Error: " + error);
        
        // Flash error LED
        for (int i = 0; i < 10; i++) {
            digitalWrite(PIN_ERROR_LED, HIGH);
            delay(100);
            digitalWrite(PIN_ERROR_LED, LOW);
            delay(100);
        }
    }

private:
    /**
     * Generate test NMEA data for development
     */
    void generate_test_data() {
        // Cycle through test NMEA sentences
        const char* sentence = test_nmea_sentences[test_sentence_index];
        test_sentence_index = (test_sentence_index + 1) % 6;
        
        // Feed test sentence to GPS parser
        for (int i = 0; i < strlen(sentence); i++) {
            if (gps.encode(sentence[i])) {
                // GPS parser has new data
                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    has_fix = true;
                    gps_fix = true;
                    last_gps_update = millis();
                }
                
                if (gps.altitude.isValid()) {
                    altitude = gps.altitude.meters();
                }
                
                if (gps.speed.isValid()) {
                    speed_kmh = gps.speed.kmph();
                }
                
                if (gps.course.isValid()) {
                    course = gps.course.deg();
                }
                
                if (gps.satellites.isValid()) {
                    satellites = gps.satellites.value();
                }
            }
        }
        
        Serial.println("Generated test GPS data");
    }

    /**
     * Read real GPS data from hardware
     */
    void read_real_gps_data() {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                // GPS parser has new data
                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    has_fix = true;
                    gps_fix = true;
                    last_gps_update = millis();
                    
                    // Blink data LED to show activity
                    digitalWrite(PIN_DATA_LED, HIGH);
                    delay(10);
                    digitalWrite(PIN_DATA_LED, LOW);
                }
                
                if (gps.altitude.isValid()) {
                    altitude = gps.altitude.meters();
                }
                
                if (gps.speed.isValid()) {
                    speed_kmh = gps.speed.kmph();
                }
                
                if (gps.course.isValid()) {
                    course = gps.course.deg();
                }
                
                if (gps.satellites.isValid()) {
                    satellites = gps.satellites.value();
                }
            }
        }
    }

    /**
     * Update control values based on GPS data
     */
    void update_control_values() {
        // Map GPS coordinates to 0-32768 range
        uint16_t lat_value = map_coordinate_to_control(latitude, -90.0, 90.0);
        uint16_t lon_value = map_coordinate_to_control(longitude, -180.0, 180.0);
        uint16_t alt_value = map_range_to_control(altitude, 0.0, 8000.0);
        uint16_t speed_value = map_range_to_control(speed_kmh, 0.0, 200.0);
        uint16_t course_value = map_range_to_control(course, 0.0, 360.0);
        uint16_t sat_value = map_range_to_control(satellites, 0.0, 20.0);
        uint16_t fix_value = gps_fix ? 32768 : 0;
        
        // Update control values
        set_control_value("Latitude", lat_value);
        set_control_value("Longitude", lon_value);
        set_control_value("Altitude", alt_value);
        set_control_value("Speed", speed_value);
        set_control_value("Course", course_value);
        set_control_value("Satellites", sat_value);
        set_control_value("GPS Fix", fix_value);
    }

    /**
     * Update status LEDs based on GPS state
     */
    void update_status_leds() {
        // GPS Fix LED
        digitalWrite(PIN_GPS_FIX_LED, has_fix ? HIGH : LOW);
        
        // Error LED (blink if no GPS data for too long)
        unsigned long current_time = millis();
        if (current_time - last_gps_update > GPS_TIMEOUT) {
            digitalWrite(PIN_ERROR_LED, (current_time / 500) % 2);
        } else {
            digitalWrite(PIN_ERROR_LED, LOW);
        }
        
        // Status LED shows network status
        digitalWrite(PIN_STATUS_LED, is_registered() ? HIGH : LOW);
    }

    /**
     * Map coordinate value to control range (0-32768)
     */
    uint16_t map_coordinate_to_control(double value, double min_val, double max_val) {
        if (value < min_val) value = min_val;
        if (value > max_val) value = max_val;
        
        return (uint16_t)((value - min_val) / (max_val - min_val) * 32768.0);
    }

    /**
     * Map range value to control range (0-32768)
     */
    uint16_t map_range_to_control(double value, double min_val, double max_val) {
        if (value < min_val) value = min_val;
        if (value > max_val) value = max_val;
        
        return (uint16_t)(value / max_val * 32768.0);
    }
};

// Global controller instance
GPSController* controller = nullptr;

/**
 * Arduino setup function
 */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("GPS Interface Controller Starting...");
    
    // Create and initialize controller
    controller = new GPSController();
    if (controller->initialize()) {
        Serial.println("GPS Interface Controller Ready");
    } else {
        Serial.println("GPS Interface Controller Failed to Initialize");
    }
}

/**
 * Arduino main loop
 */
void loop() {
    if (controller) {
        controller->update();
    }
    delay(100); // 100ms update interval
}

// ===================================================================
// MelvinMemberController Implementation
// ===================================================================

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
    
    // Connect to WiFi and register with master
    if (!connect_to_wifi()) {
        handle_error("WiFi connection failed");
        return false;
    }
    
    if (!register_with_master()) {
        handle_error("Master registration failed");
        return false;
    }
    
    debug_print("Member controller initialized successfully");
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