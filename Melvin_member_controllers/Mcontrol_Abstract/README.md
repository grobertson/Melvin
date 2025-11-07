# Abstract Member Controller Template

This is a generic template for all Melvin member controllers. It provides the basic structure, communication protocols, and patterns that all member controllers should follow.

## Overview

This abstract controller demonstrates:

- WiFi connection to Melvin Master Controller
- Device registration and heartbeat management
- Control creation and value updates
- Modular design patterns
- Error handling and recovery
- Configuration management

## Key Components

### Core Features

- **Network Management**: Automatic connection to master controller
- **Device Registration**: Self-registration with unique device identification
- **Control Management**: Creation and management of various control types
- **Heartbeat System**: Periodic communication to maintain connection
- **Error Recovery**: Automatic reconnection and error handling

### Control Types Supported

- Boolean controls (switches, buttons)
- Continuous controls (potentiometers, sliders)
- Rotary controls (encoders)
- Multi-axis controls (joysticks)
- RGB color controls

## Architecture

```text
Abstract Member Controller
├── Network Layer (WiFi connection)
├── Registration Layer (Device identification)
├── Control Layer (Input/Output management)
├── Communication Layer (REST API client)
└── Hardware Abstraction Layer (GPIO, ADC, etc.)
```

## Usage

1. Copy this template to create a new member controller
2. Modify `DEVICE_TYPE`, `DEVICE_NAME`, and `DEVICE_ID` constants
3. Implement hardware-specific `setup_hardware()` and `read_inputs()` functions
4. Define your specific controls in `setup_controls()`
5. Customize the control reading logic in `update_controls()`

## File Structure

```text
Mcontrol_Abstract/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp            # Main controller implementation
├── include/
│   ├── melvin_member.h     # Member controller base class
│   └── config.h            # Configuration constants
└── README.md               # This documentation
```

## Network Configuration

The controller automatically connects to:

- **SSID**: Melvin_Control_Network
- **Password**: MelvinController2025
- **Master IP**: 192.168.4.1 (ESP32 AP default)

## Customization Points

When creating a specific member controller:

1. **Device Configuration** (in `config.h`):

   ```cpp
   #define DEVICE_TYPE DEVICE_HUMAN_INTERFACE  // Change as needed
   #define DEVICE_NAME "Custom Controller Name"
   #define DEVICE_ID "CUSTOM_001"
   ```

2. **Hardware Setup** (in `main.cpp`):

   ```cpp
   void setup_hardware() {
       // Initialize GPIO pins, ADC, I2C, etc.
   }
   ```

3. **Control Definitions** (in `main.cpp`):

   ```cpp
   void setup_controls() {
       // Create specific controls for this device
   }
   ```

4. **Input Reading** (in `main.cpp`):

   ```cpp
   void read_inputs() {
       // Read physical inputs and update control values
   }
   ```

## Building and Deployment

```bash
# Build the controller
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Integration with Master Controller

This template handles all the standard integration requirements:

- Device registration on startup
- Periodic heartbeat transmission
- Control value updates via REST API
- Automatic reconnection on network issues
- Graceful shutdown and deregistration
