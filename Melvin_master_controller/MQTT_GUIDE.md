# Melvin Master Controller MQTT Broker Guide

The Melvin Master Controller now includes an integrated MQTT broker for low-latency inter-device communication. This enables real-time event notifications and efficient routing between member controllers.

## Overview

The MQTT broker allows member controllers to:

- Receive immediate notifications when control values change
- Subscribe to specific device or control events
- Send commands directly to other devices
- Monitor device status changes in real-time

## MQTT Broker Configuration

The broker is automatically started with the following settings:

- **Port**: 1883 (standard MQTT port)
- **Max Clients**: 10 concurrent connections
- **QoS Level**: 0 (fire-and-forget for low latency)
- **Keepalive**: 60 seconds

## Topic Structure

The MQTT broker uses a hierarchical topic structure:

### Event Topics

- `melvin/events` - All control change events
- `melvin/events/{device_id}` - Events from specific device
- `melvin/controls/{control_id}` - Events for specific control

### Device Topics

- `melvin/devices/{device_id}/status` - Device online/offline status
- `melvin/devices/{device_id}/command` - Commands to specific device

### Control Topics

- `melvin/controls/{control_id}/set` - Send control commands

### System Topics

- `melvin/status` - Broker status and system information

## Event Message Format

### Control Change Events

```json
{
    "device_id": "GPS_001",
    "control_id": 3001,
    "control_name": "Latitude",
    "old_value": 16234,
    "new_value": 16384,
    "timestamp": 123456789
}
```

### Device Status Events

```json
{
    "device_id": "RELAY_001",
    "online": true,
    "timestamp": 123456789
}
```

### Control Commands

```json
{
    "target_device": "RELAY_001",
    "control_id": 2001,
    "value": 32768,
    "value_y": 0,
    "value_z": 0
}
```

## Usage Examples

### For Member Controllers

Member controllers can subscribe to MQTT topics to receive real-time notifications:

```cpp
#include <PicoMQTT.h>

// Create MQTT client
PicoMQTT::Client mqtt("192.168.4.1");  // Master controller IP

void setup() {
    // Connect to WiFi first
    WiFi.begin("Melvin_Control_Network", "MelvinController2025");
    
    // Subscribe to all events
    mqtt.subscribe("melvin/events", [](const char* topic, const char* payload) {
        Serial.printf("Event: %s\n", payload);
        // Parse JSON and react to events
    });
    
    // Subscribe to specific control
    mqtt.subscribe("melvin/controls/2001", [](const char* topic, const char* payload) {
        // React to specific control changes
        DynamicJsonDocument doc(512);
        deserializeJson(doc, payload);
        
        uint16_t new_value = doc["new_value"];
        // Update relay output based on new value
    });
    
    mqtt.begin();
}

void loop() {
    mqtt.loop();
}
```

### Publishing Control Commands

To send a command to another device via MQTT:

```cpp
void sendRelayCommand(uint16_t control_id, uint16_t value) {
    DynamicJsonDocument doc(256);
    doc["target_device"] = "RELAY_001";
    doc["control_id"] = control_id;
    doc["value"] = value;
    
    String json_string;
    serializeJson(doc, json_string);
    
    String topic = "melvin/controls/" + String(control_id) + "/set";
    mqtt.publish(topic.c_str(), json_string.c_str());
}
```

## Use Case: Switch-to-Relay Routing

Here's how to implement low-latency switch-to-relay control:

### Human Interface Controller (Switch)

```cpp
// When switch state changes, it's automatically published via REST API
// The master controller publishes the event to MQTT
// No additional code needed in the switch controller
```

### Relay Bank Controller  

```cpp
void setup() {
    // Subscribe to the specific switch control
    mqtt.subscribe("melvin/controls/1001", [](const char* topic, const char* payload) {
        DynamicJsonDocument doc(512);
        deserializeJson(doc, payload);
        
        uint16_t switch_value = doc["new_value"];
        
        // Immediately update relay output
        if (switch_value > 0) {
            digitalWrite(RELAY_PIN, HIGH);
        } else {
            digitalWrite(RELAY_PIN, LOW);
        }
        
        Serial.printf("Relay updated via MQTT: %s\n", switch_value > 0 ? "ON" : "OFF");
    });
}
```

## Benefits Over REST API Polling

- **Lower Latency**: Immediate notification vs. periodic polling
- **Reduced Network Traffic**: Events only sent when changes occur
- **Better Scalability**: Multiple devices can listen to same events
- **Efficient**: No need to poll all controls continuously

## Network Architecture

```text
[Switch Controller] ──REST──> [Master Controller] ──MQTT──> [Relay Controller]
                                     |
                               [MQTT Broker]
                                     |
                             ┌───────┴───────┐
                    [Light Controller]  [Display Controller]
```

## Implementation Notes

1. **Dual Protocol**: The system supports both REST API and MQTT
2. **Event Generation**: All REST API changes automatically generate MQTT events
3. **Topic Routing**: Multiple topic patterns allow flexible subscription strategies
4. **Low Overhead**: MQTT QoS 0 provides lowest latency for real-time applications

## Troubleshooting

### Common Issues

1. **Connection Failed**: Ensure member controller connects to Melvin WiFi network first
2. **No Events**: Check that REST API calls are being made to trigger events
3. **High Latency**: Verify QoS settings and network congestion

### Debug Topics

Subscribe to `melvin/status` to monitor broker health:

```cpp
mqtt.subscribe("melvin/status", [](const char* topic, const char* payload) {
    Serial.printf("Broker Status: %s\n", payload);
});
```

## Future Enhancements

Possible extensions to the MQTT system:

- Authentication and access control
- Encrypted connections (MQTT over TLS)
- Retained messages for control state persistence
- QoS levels for critical control paths
- MQTT-SN for ultra-low-power devices
