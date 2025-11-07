# Melvin Master Controller

This directory contains the ESP32 master controller implementation for the Melvin vehicle control system.

## Features

- **WiFi Access Point**: Creates "Melvin_Control_Network" for member controllers to join
- **REST API Endpoints**: Complete HTTP API for device and control management
- **Device Management**: Registration, deregistration, and heartbeat monitoring
- **Control System**: Support for multiple control types (boolean, continuous, rotary, joystick, RGB)
- **Flash Patterns**: Wig-wag and other flashing patterns for emergency lighting
- **Real-time Monitoring**: Device status tracking and automatic offline detection

## Control Types Supported

1. **Boolean Controls** (0 or 1): Toggle switches, buttons
2. **Continuous Controls** (0-32768): Potentiometers, sliders  
3. **Rotary Controls** (0-32768): Rotary encoders with wrap-around
4. **2-Axis Joystick** (two 0-32768 values): Gamepad-style directional controls
5. **RGB Groups** (three 0-32768 values): Color lighting controls

## API Endpoints

### Control Management

- `GET /controls/` - List all controls
- `POST /controls/` - Create new control
- `PUT /controls/{id}` - Update control value

### Device Management

- `POST /register/device/` - Register new member controller
- `POST /deregister/device/` - Remove member controller
- `GET /devices/` - List all registered devices

## Network Configuration

- **SSID**: Melvin_Control_Network  
- **Password**: MelvinController2025
- **Channel**: 6
- **Max Connections**: 10 devices

## Building with PlatformIO

This project uses PlatformIO with Arduino framework for easier development and dependency management.

### Prerequisites

1. Install [PlatformIO](https://platformio.org/install) or use PlatformIO IDE extension for VS Code
2. No additional setup required - dependencies are managed automatically

### Building and Uploading

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Open serial monitor
pio device monitor

# Build, upload, and monitor in one command
pio run --target upload && pio device monitor
```

### Supported Boards

- ESP32 Dev Module (default)
- ESP32-S3 DevKit-C (alternative configuration)

## Hardware Requirements

- ESP32 or ESP32-S3 development board
- 12V automotive power supply with 3.3V regulation
- WiFi antenna (built-in ESP32 antenna sufficient for vehicle applications)

## Dependencies

The following libraries are automatically managed by PlatformIO:

- **ArduinoJson** (^6.21.3): JSON parsing and generation
- **ESPAsyncWebServer** (^1.2.3): High-performance async HTTP server
- **AsyncTCP** (^1.1.1): Async TCP library for ESP32

## Member Controller Integration

Member controllers connect to this master by:

1. Joining the WiFi network
2. Registering via POST to `/register/device/`  
3. Sending control updates via PUT to `/controls/{id}`
4. Maintaining heartbeat communication (30-second timeout)
