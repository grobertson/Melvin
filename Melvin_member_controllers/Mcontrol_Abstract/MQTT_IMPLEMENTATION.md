# MQTT Implementation Guide

## Overview

The Melvin Abstract Member Controller now includes a complete MQTT client implementation that works alongside the existing REST API. This provides real-time event-based communication between controllers.

## Architecture

### Dual Protocol Design

- **REST API**: Used for device registration, configuration queries, and administrative tasks
- **MQTT**: Used for real-time control events, state updates, and inter-device messaging

### MQTT Broker

- Hosted on the master controller at port 1883
- PicoMQTT v0.3.8 lightweight broker
- Supports standard MQTT 3.1.1 protocol

### MQTT Client

- Integrated into abstract member controller base class
- Auto-reconnection with exponential backoff
- Persistent subscriptions across reconnections
- Flexible message routing system

## Topic Hierarchy

```text
melvin/
├── devices/{device_id}/
│   ├── status              # Device online/offline/error status
│   ├── controls/{id}       # Specific control events
│   └── events/{type}       # Custom device events
├── controls/{id}/
│   ├── events              # Control state changes
│   └── commands            # Control commands
└── events/{type}           # Global event topics
```

## Key Features

### 1. Automatic Connection Management

```cpp
// Automatically connects on startup
init_mqtt_client();

// Auto-reconnects with retry logic
mqtt_auto_reconnect();  // Called in update() loop
```

### 2. Global Message Handling

All MQTT messages are routed through a central handler that:

- Checks custom subscriptions first
- Routes control commands to appropriate handlers
- Processes device status messages
- Forwards unmatched messages to custom event handler

```cpp
void handle_mqtt_message(const String& topic, const String& payload) {
    // Custom subscriptions have priority
    // Then control commands
    // Then device status
    // Finally custom events
}
```

### 3. Control Event Publishing

**Simple Control Update:**

```cpp
// Publishes to: melvin/devices/{device_id}/controls/{control_id}
publish_control_event(control_id, "control_change");
```

**Complete Control State:**

```cpp
// Includes value, control type, metadata
publish_control_state(control);
```

**Automatic Publishing:**

```cpp
// Publishes all changed controls in one call
auto_publish_changed_controls();
```

### 4. Custom Subscriptions

**Subscribe to Specific Topics:**

```cpp
subscribe_to_custom_topic("melvin/lighting/front", [](String topic, String payload) {
    // Handle front lighting messages
    Serial.println("Front lights: " + payload);
});
```

**Subscribe to Control Events:**

```cpp
// Subscribe to all control events
subscribe_to_control_events(0);

// Subscribe to specific control
subscribe_to_control_events(1234);
```

**Subscribe to Device Controls:**

```cpp
// Subscribe to all device controls
subscribe_to_device_controls("");

// Subscribe to specific device
subscribe_to_device_controls("RELAY_001");
```

### 5. Control Routing Modes

**MQTT_ROUTE_NONE** (default)

- Receive commands but don't auto-update controls
- Application handles routing manually

**MQTT_ROUTE_DIRECT**

- Incoming control commands directly update local control values
- Useful for output devices (lights, relays)

**MQTT_ROUTE_MAPPED**

- Map incoming control IDs to different local IDs
- Useful for control translation (HID button → relay output)

```cpp
// Enable direct routing
set_mqtt_routing_mode(MQTT_ROUTE_DIRECT);

// Enable mapped routing
set_mqtt_routing_mode(MQTT_ROUTE_MAPPED);
add_control_id_mapping(1001, 2001);  // HID button → Relay output
```

### 6. Custom Routing Callback

For complex routing logic:

```cpp
set_control_routing_callback([](const MQTTControlCommand& cmd) -> bool {
    if (cmd.control_id >= 1000 && cmd.control_id < 2000) {
        // Custom handling for HID controls
        handle_hid_command(cmd);
        return true;  // Handled
    }
    return false;  // Not handled, use default routing
});
```

## Data Structures

### MQTTEvent

```cpp
struct MQTTEvent {
    String device_id;
    String event_type;
    uint16_t control_id;
    uint16_t value;
    uint16_t value_y;    // For 2D/3D controls
    uint16_t value_z;    // For 3D controls
    String metadata;      // JSON string
    unsigned long timestamp;
};
```

### MQTTControlCommand

```cpp
struct MQTTControlCommand {
    String target_device_id;  // Empty = broadcast
    uint16_t control_id;
    uint16_t value;
    uint16_t value_y;
    uint16_t value_z;
    String command_type;
    String source_device_id;
    unsigned long timestamp;
};
```

## Virtual Methods to Override

### on_mqtt_connected()

Called when MQTT connection is established

```cpp
void on_mqtt_connected() override {
    MelvinMemberController::on_mqtt_connected();
    // Subscribe to custom topics
    subscribe_to_custom_topic("melvin/emergency/+", handle_emergency);
}
```

### on_mqtt_disconnected()

Called when MQTT connection is lost

```cpp
void on_mqtt_disconnected() override {
    // Handle disconnection (optional)
}
```

### on_mqtt_control_command()

Called when a control command is received

```cpp
void on_mqtt_control_command(const MQTTControlCommand& cmd) override {
    Serial.printf("Control %d set to %d\n", cmd.control_id, cmd.value);
    // Update hardware, trigger actions, etc.
}
```

### on_mqtt_device_status()

Called when a device status message is received

```cpp
void on_mqtt_device_status(const String& device_id, 
                           const String& status, 
                           const String& details) override {
    if (status == "offline") {
        // Handle device going offline
    }
}
```

### on_mqtt_custom_event()

Called for unmatched MQTT messages

```cpp
void on_mqtt_custom_event(const String& topic, const String& payload) override {
    // Handle custom application-specific messages
}
```

## Configuration

### Enable/Disable MQTT

Set in `include/config.h`:

```cpp
#define MQTT_ENABLED true  // or false
```

### MQTT Broker Settings

```cpp
#define MQTT_BROKER_HOST "192.168.4.1"  // Master controller IP
#define MQTT_BROKER_PORT 1883
```

### Topic Prefixes

```cpp
#define MQTT_TOPIC_PREFIX "melvin"
#define MQTT_TOPIC_DEVICES "melvin/devices"
#define MQTT_TOPIC_CONTROLS "melvin/controls"
#define MQTT_TOPIC_EVENTS "melvin/events"
```

## Testing

### 1. Monitor All Messages

Use the global subscription to see all MQTT traffic:

```cpp
void on_mqtt_custom_event(const String& topic, const String& payload) override {
    Serial.println("MQTT: " + topic + " = " + payload);
}
```

### 2. Test Control Publishing

```cpp
// In your control reading code
if (button_pressed) {
    MemberControl button;
    button.control_id = 1001;
    button.current_value = 1;
    button.name = "Emergency Button";
    publish_control_state(button);
}
```

### 3. Test Control Commands

From another device or MQTT client:

```json
// Publish to: melvin/devices/HID_001/controls/1001
{
    "target_device_id": "RELAY_001",
    "control_id": 2001,
    "value": 1,
    "command_type": "set",
    "source_device_id": "HID_001",
    "timestamp": 123456
}
```

### 4. MQTT Debug Tools

Use an MQTT client like MQTT Explorer or mosquitto_sub:

```bash
# Subscribe to all Melvin messages
mosquitto_sub -h 192.168.4.1 -t "melvin/#" -v
```

## Performance Considerations

### Message Frequency

- Avoid publishing on every analog value change (noisy)
- Use deadband filtering for continuous controls
- Batch updates when possible with `auto_publish_changed_controls()`

### QoS Levels

Current implementation uses QoS 0 (at most once):

- Fast and lightweight
- Acceptable for real-time control where latest value matters
- For critical commands, consider adding application-level ACK

### Memory Usage

- Each custom subscription callback uses ~40 bytes
- MQTT client buffers use ~4KB total
- Consider memory when subscribing to many topics

## Integration with REST API

### When to Use REST vs MQTT

**Use REST for:**

- Device registration and initial handshake
- Configuration queries
- Administrative commands
- Requesting current state
- Acknowledgment of critical operations

**Use MQTT for:**

- Real-time control events (button presses, switch toggles)
- Sensor readings
- State change notifications
- Inter-device messaging
- Event broadcasting

### Typical Flow

1. Device boots → REST registration with master
2. Device connects to MQTT broker
3. Device subscribes to relevant topics
4. Device publishes control events via MQTT
5. Device receives commands via MQTT
6. Periodic REST status updates (optional)

## Troubleshooting

### MQTT Not Connecting

- Check `MQTT_BROKER_HOST` matches master controller IP
- Verify master controller MQTT broker is running
- Check network connectivity (ping test)
- Review connection retry logs

### Messages Not Received

- Verify subscription is established (check logs)
- Ensure topic syntax matches exactly
- Use wildcard subscriptions for testing (`#`)
- Check message is being published (broker logs)

### High Memory Usage

- Reduce number of custom subscriptions
- Avoid storing large payloads
- Use payload size limits in JSON parsing

### Connection Drops

- Check WiFi signal strength
- Verify master controller stability
- Review auto-reconnection logs
- Consider increasing retry intervals

## Best Practices

1. **Use Meaningful Topic Names**: Follow the hierarchy consistently
2. **Include Timestamps**: Always timestamp events for correlation
3. **Add Source Device IDs**: Track message origin for debugging
4. **Handle Disconnections Gracefully**: Don't assume MQTT is always available
5. **Log Important Events**: Use the MelvinLogger for troubleshooting
6. **Test Reconnection**: Verify behavior when master reboots
7. **Document Custom Topics**: Keep track of application-specific topics
8. **Version Your Message Formats**: Plan for future protocol changes

## Example: HID to Relay Control

```cpp
// In HID controller setup()
void setup() {
    // ... device registration ...
    
    // Enable mapped routing
    set_mqtt_routing_mode(MQTT_ROUTE_MAPPED);
    
    // Map button presses to relay controls
    add_control_id_mapping(1001, 2001);  // Button 1 → Relay 1
    add_control_id_mapping(1002, 2002);  // Button 2 → Relay 2
    
    // Subscribe to relay status
    subscribe_to_device_controls("RELAY_001");
}

void on_mqtt_device_status(const String& device_id, 
                           const String& status, 
                           const String& details) override {
    if (device_id == "RELAY_001" && status == "offline") {
        // Flash indicator LED to show relay offline
        indicate_relay_offline();
    }
}

void loop() {
    controller.update();
    
    // Read button state
    if (button1_pressed()) {
        // Publish control event
        publish_control_event(1001, "button_press");
    }
}
```

## Future Enhancements

Potential improvements for future versions:

- [ ] QoS 1 support for critical messages
- [ ] Retained messages for persistent state
- [ ] Message compression for large payloads
- [ ] Topic-based access control
- [ ] MQTT-based OTA updates
- [ ] Structured logging via MQTT
- [ ] Control group subscriptions
- [ ] Pattern-based topic matching

## Related Documentation

- `CUSTOMIZATION_GUIDE.md` - How to create custom member controllers
- `MQTT_GUIDE.md` (master controller) - MQTT broker configuration
- `CONTROL_ID_ALLOCATION.md` - Control ID ranges and assignment
- `README.md` - Member controllers overview
