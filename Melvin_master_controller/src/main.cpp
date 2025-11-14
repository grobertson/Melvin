/**
 * Melvin Master Controller - ESP32 Vehicle Control System
 * 
 * This is the master controller that operates in WiFi AP mode and coordinates
 * all member controllers in the Melvin distributed vehicle control system.
 * 
 * Features:
 * - WiFi Access Point for member controller network
 * - REST API endpoints for device registration and control
 * - Device management and tracking
 * - Control value routing and distribution
 * - Flash pattern support (wig-wag, etc.)
 * 
 * Author: F. G. Robertson (@grobertson)
 * Framework: Arduino C++ using (PlatformIO)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PicoMQTT.h>
#include <map>
#include <vector>
#include <string>
#include <memory>
#include "melvin_config.h"
#include "melvin_version.h"
#include "melvin_errors.h"

/**
 * Global Data Structures
 */
static std::map<uint16_t, Control> g_controls;              // All controls in the system
static std::map<std::string, Device> g_devices;             // All registered devices
static uint16_t g_next_control_id = 1;                      // Next available control ID
static hw_timer_t* g_flash_timer = nullptr;                 // Timer for flash patterns
static AsyncWebServer g_server(80);                         // HTTP server

// MQTT Broker instance
static WiFiServer g_mqtt_server(MQTT_PORT);                 // MQTT server
static PicoMQTT::Server g_mqtt(g_mqtt_server);              // MQTT broker

/**
 * Generate unique control ID
 */
uint16_t generate_control_id() {
    return g_next_control_id++;
}

/**
 * MQTT Helper Functions
 */

/**
 * Publish an event to MQTT when a control value changes
 */
void publish_control_event(const std::string& device_id, uint16_t control_id, 
                          const std::string& control_name, uint16_t old_value, uint16_t new_value) {
    DynamicJsonDocument doc(512);
    doc["device_id"] = device_id;
    doc["control_id"] = control_id;
    doc["control_name"] = control_name;
    doc["old_value"] = old_value;
    doc["new_value"] = new_value;
    doc["timestamp"] = millis();
    
    String json_string;
    serializeJson(doc, json_string);
    
    // Publish to general events topic
    g_mqtt.publish(MQTT_TOPIC_EVENTS, json_string.c_str(), MQTT_RETAIN_EVENTS, MQTT_QOS_EVENTS);
    
    // Publish to device-specific topic for targeted listening
    String device_topic = String(MQTT_TOPIC_EVENTS) + "/" + device_id.c_str();
    g_mqtt.publish(device_topic.c_str(), json_string.c_str(), MQTT_RETAIN_EVENTS, MQTT_QOS_EVENTS);
    
    // Publish to control-specific topic for direct control monitoring
    String control_topic = String(MQTT_TOPIC_CONTROLS) + "/" + String(control_id);
    g_mqtt.publish(control_topic.c_str(), json_string.c_str(), MQTT_RETAIN_EVENTS, MQTT_QOS_EVENTS);
    
    Serial.printf("MQTT: Published control event - Device: %s, Control: %s, Value: %u -> %u\n", 
                  device_id.c_str(), control_name.c_str(), old_value, new_value);
}

/**
 * Publish device status updates to MQTT
 */
void publish_device_status(const std::string& device_id, bool online) {
    DynamicJsonDocument doc(256);
    doc["device_id"] = device_id;
    doc["online"] = online;
    doc["timestamp"] = millis();
    
    String json_string;
    serializeJson(doc, json_string);
    
    String device_status_topic = String(MQTT_TOPIC_DEVICES) + "/" + device_id.c_str() + "/status";
    g_mqtt.publish(device_status_topic.c_str(), json_string.c_str(), true, MQTT_QOS_EVENTS); // Retain device status
    
    Serial.printf("MQTT: Published device status - Device: %s, Online: %s\n", 
                  device_id.c_str(), online ? "true" : "false");
}

/**
 * Handle incoming MQTT control commands
 */
void handle_mqtt_control_command(const char* topic, const char* payload) {
    DynamicJsonDocument doc(512);
    
    if (deserializeJson(doc, payload)) {
        Serial.printf("MQTT: Failed to parse control command JSON: %s\n", payload);
        return;
    }
    
    std::string target_device = doc["target_device"].as<std::string>();
    uint16_t control_id = doc["control_id"];
    uint16_t value = doc["value"];
    uint16_t value_y = doc["value_y"] | 0;
    uint16_t value_z = doc["value_z"] | 0;
    
    // Find and update the control
    auto it = g_controls.find(control_id);
    if (it != g_controls.end()) {
        Control& control = it->second;
        uint16_t old_value = control.current_value;
        
        control.current_value = value;
        control.current_value_y = value_y;
        control.current_value_z = value_z;
        
        // Publish the change event
        publish_control_event(target_device, control_id, control.name, old_value, value);
        
        Serial.printf("MQTT: Updated control %u via MQTT command: %u -> %u\n", control_id, old_value, value);
    } else {
        Serial.printf("MQTT: Control command for unknown control ID: %u\n", control_id);
    }
}

/**
 * Initialize MQTT broker
 */
void init_mqtt_broker() {
    Serial.println("Initializing MQTT broker...");
    
    // Subscribe to control command topics
    g_mqtt.subscribe(String(MQTT_TOPIC_CONTROLS) + "/+/set", [](const char* topic, const char* payload) {
        Serial.printf("MQTT: Received control command on topic: %s\n", topic);
        handle_mqtt_control_command(topic, payload);
    });
    
    // Subscribe to device command topics
    g_mqtt.subscribe(String(MQTT_TOPIC_DEVICES) + "/+/command", [](const char* topic, const char* payload) {
        Serial.printf("MQTT: Received device command on topic: %s\n", topic);
        // Device commands can be implemented here for remote device management
    });
    
    // Start the MQTT broker
    g_mqtt.begin();
    
    // Publish broker status
    DynamicJsonDocument status_doc(256);
    status_doc["broker"] = "online";
    status_doc["port"] = MQTT_PORT;
    status_doc["max_clients"] = MQTT_MAX_CLIENTS;
    status_doc["timestamp"] = millis();
    
    String status_json;
    serializeJson(status_doc, status_json);
    g_mqtt.publish(MQTT_TOPIC_STATUS, status_json.c_str(), true, MQTT_QOS_EVENTS);
    
    Serial.printf("MQTT broker initialized on port %d\n", MQTT_PORT);
}

/**
 * Flash pattern timer callback - handles all flash patterns
 */
void IRAM_ATTR flash_timer_callback() {
    uint32_t current_time = millis();
    
    for (auto& control_pair : g_controls) {
        Control& control = control_pair.second;
        
        if (control.flash_pattern == FLASH_NONE) {
            continue;
        }
        
        uint32_t pattern_period = FLASH_PATTERN_PERIOD_MS;
        if (control.flash_pattern == FLASH_WIG_WAG) {
            pattern_period = WIG_WAG_PERIOD_MS;
        }
        
        if (current_time - control.last_flash_time >= pattern_period) {
            control.flash_state = !control.flash_state;
            control.last_flash_time = current_time;
            
            Serial.printf("Flash pattern update - Control ID: %d, State: %s\n", 
                         control_pair.first, control.flash_state ? "ON" : "OFF");
        }
    }
}

/**
 * Initialize flash pattern timer
 */
void init_flash_timer() {
    // Create hardware timer (timer 0, divider 80 for 1MHz, count up)
    g_flash_timer = timerBegin(0, 80, true);
    
    // Attach callback function
    timerAttachInterrupt(g_flash_timer, &flash_timer_callback, false);
    
    // Set timer interval (50ms = 50000 microseconds for smooth patterns)
    timerAlarmWrite(g_flash_timer, 50000, true);
    
    // Enable timer
    timerAlarmEnable(g_flash_timer);
    
    Serial.println("Flash pattern timer initialized");
}

/**
 * WiFi event handlers
 */
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("Station connected to AP");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("Station disconnected from AP");
            break;
        case ARDUINO_EVENT_WIFI_AP_START:
            Serial.println("AP Started");
            //Serial.print("AP SSID: ");
            //Serial.println(WIFI_SSID);
            //Serial.print("AP IP address: ");
            //Serial.println(WiFi.softAPIP());
            break;
        default:
            break;
    }
}

/**
 * Initialize WiFi in Access Point mode
 */
void wifi_init_ap() {
    WiFi.onEvent(WiFiEvent);
    
    // Configure and start Access Point
    WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL, 0, MAX_STA_CONN);
    
    // Configure IP settings (optional - uses defaults)
    IPAddress local_ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    
    Serial.printf("WiFi AP started. SSID: %s, Channel: %d\n", WIFI_SSID, WIFI_CHANNEL);
    Serial.printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
}

/**
 * Create JSON response for control data
 */
void control_to_json(JsonObject& json, const Control& control, uint16_t control_id) {
    json["id"] = control_id;
    json["base_address"] = control.base_address;
    json["control_number"] = control.control_number;
    json["type"] = control.type;
    json["value"] = control.current_value;
    
    if (control.type == CONTROL_JOYSTICK_2AXIS) {
        json["value_y"] = control.current_value_y;
    } else if (control.type == CONTROL_RGB_GROUP) {
        json["value_g"] = control.current_value_y;
        json["value_b"] = control.current_value_z;
    }
    
    json["name"] = control.name;
    json["is_group"] = control.is_group;
    json["flash_pattern"] = control.flash_pattern;
    json["flash_state"] = control.flash_state;
}

/**
 * Create JSON response for device data
 */
void device_to_json(JsonObject& json, const Device& device) {
    json["device_id"] = device.device_id;
    json["ip_address"] = device.ip_address;
    json["device_type"] = device.device_type;
    json["name"] = device.name;
    json["is_online"] = device.is_online;
    json["last_heartbeat"] = device.last_heartbeat;
    
    JsonArray controls_array = json.createNestedArray("controls");
    for (uint16_t control_id : device.controls) {
        controls_array.add(control_id);
    }
}

/**
 * Setup all REST API endpoints
 */
void setup_rest_api() {
    // Enable CORS for all routes
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
    
    // GET /controls/ - List all controls
    g_server.on("/controls/", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(8192);
        JsonArray controls_array = doc.createNestedArray("controls");
        
        for (const auto& control_pair : g_controls) {
            JsonObject control_obj = controls_array.createNestedObject();
            control_to_json(control_obj, control_pair.second, control_pair.first);
        }
        
        doc["count"] = g_controls.size();
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // POST /controls/ - Create new control
    g_server.on("/controls/", HTTP_POST, [](AsyncWebServerRequest *request) {
        // This will be handled by the body handler below
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, (char*)data);
        
        if (!doc.containsKey("base_address") || !doc.containsKey("control_number") ||
            !doc.containsKey("type") || !doc.containsKey("name")) {
            request->send(400, "application/json", "{\"error\":\"Missing required fields\"}");
            return;
        }
        
        // Create new control
        uint16_t control_id = generate_control_id();
        Control new_control;
        new_control.base_address = doc["base_address"];
        new_control.control_number = doc["control_number"];
        new_control.type = (ControlType)(int)doc["type"];
        new_control.name = doc["name"].as<std::string>();
        new_control.current_value = CONTROL_VALUE_OFF;
        
        g_controls[control_id] = new_control;
        
        Serial.printf("Created control ID: %d, Base: %d, Number: %d, Type: %d, Name: %s\n",
                     control_id, new_control.base_address, new_control.control_number,
                     new_control.type, new_control.name.c_str());
        
        // Send response
        DynamicJsonDocument response_doc(512);
        response_doc["control_id"] = control_id;
        response_doc["status"] = "created";
        
        String response;
        serializeJson(response_doc, response);
        request->send(200, "application/json", response);
    });
    
    // PUT /controls/{id} - Update control value
    g_server.on("/controls/*", HTTP_PUT, [](AsyncWebServerRequest *request) {
        // This will be handled by the body handler below
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        // Extract control ID from URL
        String url = request->url();
        int lastSlash = url.lastIndexOf('/');
        uint16_t control_id = url.substring(lastSlash + 1).toInt();
        
        if (g_controls.find(control_id) == g_controls.end()) {
            request->send(404, "application/json", "{\"error\":\"Control not found\"}");
            return;
        }
        
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, (char*)data);
        
        Control& control = g_controls[control_id];
        
        // Store old values for event publishing
        uint16_t old_value = control.current_value;
        uint16_t old_value_y = control.current_value_y;
        uint16_t old_value_z = control.current_value_z;
        
        // Find the device that owns this control for event publishing
        std::string owner_device_id = "unknown";
        for (const auto& device_pair : g_devices) {
            const Device& device = device_pair.second;
            for (uint16_t control_id_check : device.controls) {
                if (control_id_check == control_id) {
                    owner_device_id = device.device_id;
                    break;
                }
            }
            if (owner_device_id != "unknown") break;
        }
        
        // Update control values based on type
        if (doc.containsKey("value")) {
            control.current_value = doc["value"];
            
            // Clamp values to valid range
            if (control.type == CONTROL_BOOLEAN) {
                control.current_value = (control.current_value > 0) ? CONTROL_VALUE_BOOLEAN_ON : CONTROL_VALUE_OFF;
            } else {
                if (control.current_value > CONTROL_VALUE_MAX) {
                    control.current_value = CONTROL_VALUE_MAX;
                }
            }
        }
        
        // Handle multi-axis controls
        if (control.type == CONTROL_JOYSTICK_2AXIS || control.type == CONTROL_RGB_GROUP) {
            if (doc.containsKey("value_y")) {
                control.current_value_y = doc["value_y"];
                if (control.current_value_y > CONTROL_VALUE_MAX) {
                    control.current_value_y = CONTROL_VALUE_MAX;
                }
            }
            
            if (control.type == CONTROL_RGB_GROUP && doc.containsKey("value_z")) {
                control.current_value_z = doc["value_z"];
                if (control.current_value_z > CONTROL_VALUE_MAX) {
                    control.current_value_z = CONTROL_VALUE_MAX;
                }
            }
        }
        
        // Update flash pattern if specified
        if (doc.containsKey("flash_pattern")) {
            control.flash_pattern = (FlashPattern)(int)doc["flash_pattern"];
            control.last_flash_time = millis();
            control.flash_state = false;
        }
        
        Serial.printf("Updated control ID: %d, Value: %d\n", control_id, control.current_value);
        
        // Publish MQTT event for control value change
        bool value_changed = (old_value != control.current_value) || 
                            (old_value_y != control.current_value_y) || 
                            (old_value_z != control.current_value_z);
        
        if (value_changed) {
            publish_control_event(owner_device_id, control_id, control.name, old_value, control.current_value);
        }
        
        // Send updated control data
        DynamicJsonDocument response_doc(1024);
        JsonObject response_obj = response_doc.to<JsonObject>();
        control_to_json(response_obj, control, control_id);
        
        String response;
        serializeJson(response_doc, response);
        request->send(200, "application/json", response);
    });
    
    // POST /register/device/ - Register new device
    g_server.on("/register/device/", HTTP_POST, [](AsyncWebServerRequest *request) {
        // This will be handled by the body handler below
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, (char*)data);
        
        if (!doc.containsKey("device_id") || !doc.containsKey("device_type") ||
            !doc.containsKey("name")) {
            request->send(400, "application/json", "{\"error\":\"Missing required fields\"}");
            return;
        }
        
        // Create new device
        Device new_device;
        new_device.device_id = doc["device_id"].as<std::string>();
        new_device.device_type = (DeviceType)(int)doc["device_type"];
        new_device.name = doc["name"].as<std::string>();
        new_device.last_heartbeat = millis();
        new_device.is_online = true;
        new_device.ip_address = request->client()->remoteIP().toString().c_str();
        
        g_devices[new_device.device_id] = new_device;
        
        Serial.printf("Registered device: %s (%s) Type: %d IP: %s\n",
                     new_device.device_id.c_str(), new_device.name.c_str(),
                     new_device.device_type, new_device.ip_address.c_str());
        
        // Publish MQTT device status event
        publish_device_status(new_device.device_id, true);
        
        // Send response
        DynamicJsonDocument response_doc(512);
        response_doc["status"] = "registered";
        response_doc["device_id"] = new_device.device_id;
        
        String response;
        serializeJson(response_doc, response);
        request->send(200, "application/json", response);
    });
    
    // POST /deregister/device/ - Deregister device
    g_server.on("/deregister/device/", HTTP_POST, [](AsyncWebServerRequest *request) {
        // This will be handled by the body handler below
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        DynamicJsonDocument doc(512);
        deserializeJson(doc, (char*)data);
        
        if (!doc.containsKey("device_id")) {
            request->send(400, "application/json", "{\"error\":\"Missing device_id\"}");
            return;
        }
        
        std::string device_id = doc["device_id"].as<std::string>();
        
        if (g_devices.find(device_id) == g_devices.end()) {
            request->send(404, "application/json", "{\"error\":\"Device not found\"}");
            return;
        }
        
        g_devices.erase(device_id);
        
        // Publish MQTT device status event
        publish_device_status(device_id, false);
        
        Serial.printf("Deregistered device: %s\n", device_id.c_str());
        
        DynamicJsonDocument response_doc(512);
        response_doc["status"] = "deregistered";
        response_doc["device_id"] = device_id;
        
        String response;
        serializeJson(response_doc, response);
        request->send(200, "application/json", response);
    });
    
    // GET /devices/ - List all devices
    g_server.on("/devices/", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(4096);
        JsonArray devices_array = doc.createNestedArray("devices");
        
        for (const auto& device_pair : g_devices) {
            JsonObject device_obj = devices_array.createNestedObject();
            device_to_json(device_obj, device_pair.second);
        }
        
        doc["count"] = g_devices.size();
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // GET /health - System health check endpoint
    g_server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(512);
        
        doc["status"] = "ok";
        doc["version"] = MELVIN_VERSION_STRING;
        doc["build"] = MELVIN_VERSION_BUILD;
        doc["uptime_ms"] = millis();
        doc["uptime_seconds"] = millis() / 1000;
        doc["device_count"] = g_devices.size();
        doc["control_count"] = g_controls.size();
        
        // Count online devices
        int online_count = 0;
        for (const auto& device_pair : g_devices) {
            if (device_pair.second.is_online) {
                online_count++;
            }
        }
        doc["devices_online"] = online_count;
        doc["devices_offline"] = g_devices.size() - online_count;
        
        // Memory information
        doc["free_heap"] = ESP.getFreeHeap();
        doc["heap_size"] = ESP.getHeapSize();
        doc["min_free_heap"] = ESP.getMinFreeHeap();
        
        // WiFi information
        doc["wifi_clients"] = WiFi.softAPgetStationNum();
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // Handle OPTIONS requests for CORS
    g_server.onNotFound([](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(200);
        } else {
            request->send(404, "application/json", "{\"error\":\"Not found\"}");
        }
    });
    
    Serial.println("REST API endpoints configured");
}

/**
 * Device heartbeat monitoring task
 */
void device_monitor_task() {
    static uint32_t last_check = 0;
    const uint32_t CHECK_INTERVAL = 5000; // Check every 5 seconds
    
    uint32_t current_time = millis();
    if (current_time - last_check < CHECK_INTERVAL) {
        return;
    }
    
    last_check = current_time;
    
    for (auto& device_pair : g_devices) {
        Device& device = device_pair.second;
        
        if (device.is_online && 
            (current_time - device.last_heartbeat) > HEARTBEAT_TIMEOUT_MS) {
            device.is_online = false;
            Serial.printf("Device %s (%s) went offline\n", 
                         device.device_id.c_str(), device.name.c_str());
        }
    }
}

/**
 * System status logging
 */
void log_system_status() {
    static uint32_t last_status = 0;
    const uint32_t STATUS_INTERVAL = 60000; // Every 60 seconds
    
    uint32_t current_time = millis();
    if (current_time - last_status < STATUS_INTERVAL) {
        return;
    }
    
    last_status = current_time;
    Serial.printf("System Status - Devices: %d, Controls: %d\n", 
                 g_devices.size(), g_controls.size());
}

/**
 * Arduino setup function
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Starting Melvin Master Controller");
    Serial.println("==================================");
    
    // Initialize WiFi AP
    wifi_init_ap();
    
    // Initialize MQTT broker
    init_mqtt_broker();
    
    // Initialize flash pattern timer
    init_flash_timer();
    
    // Setup REST API endpoints
    setup_rest_api();
    
    // Start HTTP server
    g_server.begin();
    
    Serial.println("Melvin Master Controller initialized successfully");
    Serial.printf("WiFi AP: %s\n", WIFI_SSID);
    Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Listening for device registrations and control commands");
    Serial.println("Ready!");
}

/**
 * Arduino main loop
 */
void loop() {
    // Process MQTT broker connections and messages
    g_mqtt.loop();
    
    // Run device monitoring
    device_monitor_task();
    
    // Log system status periodically
    log_system_status();
    
    // Small delay to prevent watchdog triggers
    delay(100);

}
