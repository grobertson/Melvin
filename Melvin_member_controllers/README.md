# Melvin Member Controllers

This directory contains implementations for all member controller types in the Melvin distributed vehicle control system. Each member controller is an ESP32-based device that connects to the master controller via WiFi and provides specific functionality.

## Overview

Member controllers form the distributed nodes of the Melvin system. They:

- Connect to the master controller's WiFi network
- Register themselves and their capabilities
- Send sensor data and control inputs to the master
- Receive commands and control outputs from the master
- Communicate with other devices via MQTT for low-latency operations

## Available Member Controllers

### ✅ Implemented Controllers

| Controller | Purpose | Status | Key Features |
|------------|---------|--------|--------------|
| **Mcontrol_Abstract** | Base template for all controllers | ✅ Complete | Full REST + MQTT, example implementation |
| **Mcontrol_GPS_Interface** | GPS data module | ✅ Complete | NMEA parsing, position tracking, MQTT events |
| **Mcontrol_ODBII_Interface** | OBDII vehicle data | ✅ Partial | ELM327 integration, vehicle diagnostics |

### 🚧 Planned Controllers

| Controller | Purpose | Planned Features |
|------------|---------|------------------|
| **Mcontrol_HID** | Human interface devices | Switches, buttons, potentiometers, joysticks, rotary encoders |
| **Mcontrol_Relay_Bank** | High-current switching | Relay control for lights, motors, auxiliary power |
| **Mcontrol_Polled_Sensor_Bank** | On-demand sensors | Temperature, pressure, voltage monitoring |
| **Mcontrol_Event_Sensor_Bank** | Event-driven sensors | Door sensors, motion detectors, limit switches |
| **Mcontrol_Display** | Information display | LCD/OLED displays, status indicators |
| **Mcontrol_Camera_Interface** | Camera control | Pan/tilt control, camera selection |
| **Mcontrol_Spotlight_Interface** | Spotlight control | Pan/tilt spotlights, focus control |
| **Mcontrol_APRS_Interface** | APRS radio | Position reporting, messaging |
| **Mcontrol_Logger** | Data logging | SD card storage, telemetry recording |

## Quick Start

### Creating a New Member Controller

1. **Copy the Abstract Template**:

   ```bash
   cp -r Mcontrol_Abstract Mcontrol_YourDevice
   ```

2. **Update Configuration** (`include/config.h`):

   ```cpp
   #define DEVICE_TYPE DEVICE_YOUR_TYPE
   #define DEVICE_NAME "Your Device Name"
   #define DEVICE_ID "YOUR_DEVICE_001"
   ```

3. **Implement Virtual Methods** (`src/main.cpp`):
   - `setup_hardware()` - Initialize GPIO, ADC, I2C, etc.
   - `read_inputs()` - Read sensor values and update controls
   - `write_outputs()` - Drive outputs based on control values
   - `process_control_changes()` - Handle control value changes
   - `handle_error()` - Custom error handling

4. **Define Controls**:

   ```cpp
   void setup_controls() {
       add_control("Button 1", 1, CONTROL_BOOLEAN);
       add_control("Temperature", 2, CONTROL_CONTINUOUS);
       add_control("RGB Light", 3, CONTROL_RGB_GROUP);
   }
   ```

5. **Build and Upload**:

   ```bash
   cd Mcontrol_YourDevice
   pio run --target upload
   ```

## Architecture

### Communication Layers

```text
┌─────────────────────────────────────────────────────┐
│             Member Controller Device                │
├─────────────────────────────────────────────────────┤
│  Application Layer (Device-Specific Logic)          │
├─────────────────────────────────────────────────────┤
│  MelvinMemberController Base Class                  │
│  ├─ Network Management (WiFi)                       │
│  ├─ Device Registration & Heartbeat                 │
│  ├─ Control Management                              │
│  ├─ REST API Client                                 │
│  └─ MQTT Client (Events & Commands)                 │
├─────────────────────────────────────────────────────┤
│  Hardware Abstraction Layer                         │
│  (GPIO, ADC, I2C, SPI, UART, etc.)                  │
└─────────────────────────────────────────────────────┘
```

### Communication Protocols

#### REST API (Configuration & State)

- **Registration**: `POST /register/device/`
- **Control Creation**: `POST /controls/`
- **Control Updates**: `PUT /controls/{id}`
- **Heartbeat**: Periodic re-registration (30s timeout)

#### MQTT (Real-time Events)

- **Control Events**: `melvin/controls/{id}/events`
- **Device Status**: `melvin/devices/{device_id}/status`
- **Commands**: `melvin/controls/{id}/set`
- **Custom Topics**: Device-specific patterns

## Control Types

### Boolean Controls

On/off switches, buttons, binary sensors

- **Value Range**: 0 (off) or 1 (on)
- **Examples**: Toggle switches, door sensors, buttons

### Continuous Controls

Analog inputs, variable outputs

- **Value Range**: 0-32768
- **Examples**: Potentiometers, sliders, brightness, volume

### Rotary Controls

Rotary encoders with wrap-around

- **Value Range**: 0-32768 (wraps at limits)
- **Examples**: Rotary knobs, angle sensors

### 2-Axis Joystick

Directional controls with X and Y axes

- **Value Range**: Two values, each 0-32768
- **Examples**: Gamepad joysticks, pan/tilt controls

### RGB Groups

Color lighting controls

- **Value Range**: Three values (R, G, B), each 0-32768
- **Examples**: RGB LEDs, color displays

## Network Configuration

All member controllers connect to:

- **SSID**: `Melvin_Control_Network`
- **Password**: `MelvinController2025`
- **Master IP**: `192.168.4.1` (ESP32 AP default)
- **MQTT Broker**: `192.168.4.1:1883`

## Features

### Automatic Connection Management

- WiFi connection with retry logic
- Automatic reconnection on network loss
- Configurable connection timeout

### Device Registration

- Unique device identification
- Type-based capability reporting
- Automatic heartbeat maintenance

### Control System

- Multiple control types supported
- Automatic value change detection
- Deadband filtering for analog inputs
- Control grouping support

### MQTT Integration

- Real-time event publishing
- Control command subscription
- Device status monitoring
- Custom topic support
- Automatic reconnection

### Error Handling

- Standardized error codes
- Automatic error recovery
- Configurable logging levels
- Hardware error detection

## Hardware Requirements

### Minimum Requirements

- ESP32 or ESP32-S3 development board
- 12V automotive power supply
- 3.3V voltage regulation
- WiFi antenna (built-in sufficient)

### Optional Components

- SD card module (for logging)
- OLED/LCD display (for local status)
- External sensors (device-specific)
- Relay modules (for high-current switching)

## Development

### Building

```bash
# Build for ESP32
pio run -e esp32dev

# Build for ESP32-S3
pio run -e esp32-s3-devkitc-1

# Upload firmware
pio run --target upload

# Monitor serial output
pio device monitor
```

### Testing

```bash
# Run tests (when implemented)
pio test

# Check code style
pio check
```

### Debugging

```bash
# Enable debug logging
# In config.h, set:
#define LOG_LEVEL LOG_DEBUG

# View detailed serial output
pio device monitor --baud 115200
```

## API Integration

### Registration Example

```cpp
bool MelvinMemberController::register_with_master() {
    DynamicJsonDocument doc(1024);
    doc["device_id"] = device_id;
    doc["device_type"] = (int)device_type;
    doc["name"] = device_name;
    doc["protocol_version"] = MELVIN_PROTOCOL_VERSION;
    
    String json_string;
    serializeJson(doc, json_string);
    
    http_client.begin("http://192.168.4.1/register/device/");
    http_client.addHeader("Content-Type", "application/json");
    int response_code = http_client.POST(json_string);
    
    return (response_code == 200);
}
```

### MQTT Event Publishing

```cpp
bool publish_sensor_event(float temperature) {
    DynamicJsonDocument doc(256);
    doc["device_id"] = device_id;
    doc["sensor_type"] = "temperature";
    doc["value"] = temperature;
    doc["timestamp"] = millis();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "melvin/devices/" + device_id + "/sensors";
    return mqtt_client.publish(topic, payload);
}
```

## Troubleshooting

### Common Issues

**Device won't connect to WiFi**

- Verify SSID and password in `config.h`
- Check master controller is powered on and AP is active
- Confirm WiFi channel compatibility
- Check antenna connection

**Registration fails**

- Ensure unique device_id (no duplicates)
- Verify master controller is accessible at 192.168.4.1
- Check network connectivity with ping
- Review serial output for error codes

**MQTT not working**

- Confirm MQTT broker is running on master
- Check `MQTT_ENABLED` flag is set to 1
- Verify firewall/network settings
- Test with simple MQTT client first

**Control updates not received**

- Check control ID matches master controller
- Verify REST API calls return 200 OK
- Confirm JSON payload format
- Enable debug logging

**High latency**

- Use MQTT for real-time controls instead of REST
- Reduce heartbeat frequency
- Optimize control update intervals
- Check WiFi signal strength

## Best Practices

### Resource Management

- Use static JSON documents for known sizes
- Pre-allocate buffers for frequent operations
- Monitor heap usage in production
- Implement watchdog timer

### Network Efficiency

- Batch multiple control updates when possible
- Use MQTT QoS 0 for non-critical events
- Implement exponential backoff for retries
- Cache master controller responses

### Error Handling

- Always check return values
- Implement graceful degradation
- Log errors with context
- Provide user feedback (LEDs, display)

### Code Organization

- Keep hardware abstraction separate
- Use meaningful variable names
- Document device-specific behavior
- Version control configuration files

## Contributing

When creating a new member controller:

1. Base it on `Mcontrol_Abstract`
2. Follow existing naming conventions
3. Document hardware requirements
4. Add README with wiring diagrams
5. Include example configuration
6. Test with real hardware
7. Submit pull request

## Support

For issues, questions, or contributions:

- Create an issue in the repository
- Check existing documentation
- Review example implementations
- Test with `Mcontrol_Abstract` first

## License

See main project LICENSE file.

## Version History

- **v0.2.0** (2025-10-26): Added MQTT client support, improved logging
- **v0.1.0** (2025-10): Initial member controller framework
