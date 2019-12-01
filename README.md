# Melvin
<<<<<<< HEAD
=======

This has been superceeded by a higher level project, encompassing the work done with Melvin, as well as my work related to a DIY fuel injection ECU, retrofitted automatic climate controls, data logging, and more. Find that repository here: [Retromod-In-Car-Systems](https://github.com/grobertson/Retromod-In-Car-Systems)
-----

Extending my Volvo with micro-controllers interconnected via serial network. This is a very early WIP, at this stage it's more like a thought experiment taken a little too far and an excuse to dive deeper into circuit design. 
>>>>>>> b1959ec (Update README.md)

Extending a 25 year old Volvo with micro-controllers.

## Project Status: v0.2.0 - Active Development

### ✅ Completed Features

- **Master Controller**: ESP32-based coordinator with WiFi AP, REST API, and MQTT broker
- **MQTT Integration**: Full pub/sub messaging system for real-time control events
- **Abstract Member Controller**: Base class with WiFi client, REST registration, and MQTT client
- **Error Handling**: Comprehensive error code system (100-999) with severity levels
- **Logging System**: 5-level logging (DEBUG, INFO, WARN, ERROR, CRITICAL) with timestamps
- **Version Tracking**: Firmware versioning with build metadata and feature flags
- **Health Monitoring**: System status endpoint with memory and device statistics
- **Connection Management**: Smart retry logic with exponential backoff
- **Control ID Allocation**: Documented ID ranges to prevent conflicts
- **Documentation**: Comprehensive guides for member controllers and MQTT usage

### 🔄 In Progress

- Hardware controller implementations (HID, Relay Bank, Sensor Bank)
- Configuration management system (web UI + SPIFFS storage)
- Testing with physical ESP32 hardware

### 📋 Planned Features

- Additional specialized controllers (GPS, Camera, Display, OBD-II)
- OTA firmware updates
- Advanced MQTT features (QoS 1, retained messages)
- Web-based configuration interface
- Data logging and analytics

## Architecture

### Communication Protocols

- **REST API**: Device registration, configuration, status queries
- **MQTT**: Real-time control events, sensor data, inter-device messaging

### Controller Types

- **Master Controller**: Central coordinator hosting WiFi network and MQTT broker
- **Member Controllers**: Specialized devices for specific functions
  - Human Interface (HID): Switches, buttons, joysticks
  - Relay Bank: Output control for lights and accessories
  - Sensor Bank: Temperature, motion, light sensors
  - GPS Interface: Location tracking and speed
  - Camera Interface: Video capture and streaming
  - Display: User interface and information display
  - OBD-II Interface: Vehicle diagnostics
  - And more...

## Getting Started

### Master Controller Setup

1. Navigate to `Melvin_master_controller/`
2. Read `GETTING_STARTED.md` for configuration
3. Build with PlatformIO: `pio run`
4. Flash to ESP32: `pio run --target upload`
5. Monitor: `pio device monitor`

### Member Controller Setup

1. Navigate to `Melvin_member_controllers/Mcontrol_Abstract/`
2. Read `README.md` and `CUSTOMIZATION_GUIDE.md`
3. Copy to create custom controller
4. Configure device ID and control IDs
5. Build and flash to ESP32

## Documentation

- **Melvin_controller_scheme.md**: Architecture and design overview
- **CONTROL_ID_ALLOCATION.md**: Control ID ranges and assignment guide
- **Melvin_master_controller/README.md**: Master controller details
- **Melvin_member_controllers/README.md**: Member controller overview
- **Mcontrol_Abstract/MQTT_IMPLEMENTATION.md**: MQTT usage guide
- **Mcontrol_Abstract/CUSTOMIZATION_GUIDE.md**: Creating custom controllers

## Technology Stack

- **Platform**: ESP32 / ESP32-S3 microcontrollers
- **Framework**: Arduino (via ESP-IDF)
- **Build System**: PlatformIO
- **Libraries**:
  - ArduinoJson (JSON serialization)
  - ESPAsyncWebServer (REST API)
  - PicoMQTT (MQTT broker/client)
  - AsyncTCP (Network layer)

## Project Structure

```text
Melvin/
├── .gitignore                          # Protect secrets and build artifacts
├── README.md                           # This file
├── LICENSE                             # Project license
├── Melvin_controller_scheme.md         # Architecture documentation
├── CONTROL_ID_ALLOCATION.md            # Control ID assignment guide
├── Melvin_master_controller/           # Master ESP32 controller
│   ├── include/
│   │   ├── melvin_config.h            # WiFi, network, MQTT config
│   │   ├── melvin_errors.h            # Error codes and logging
│   │   └── melvin_version.h           # Version tracking
│   ├── src/
│   │   └── main.cpp                   # REST API and MQTT broker
│   ├── platformio.ini                 # Build configuration
│   ├── README.md                      # Master controller docs
│   ├── GETTING_STARTED.md             # Setup instructions
│   ├── MQTT_GUIDE.md                  # MQTT broker details
│   └── BUILD_FIXES.md                 # Build troubleshooting
└── Melvin_member_controllers/          # Member ESP32 controllers
    ├── README.md                       # Member controllers overview
    ├── Mcontrol_Abstract/              # Base class template
    │   ├── include/
    │   │   ├── config.h               # Configuration constants
    │   │   └── melvin_member.h        # Class definition
    │   ├── src/
    │   │   └── main.cpp               # Base implementation
    │   ├── platformio.ini             # Build configuration
    │   ├── README.md                  # Abstract controller docs
    │   ├── CUSTOMIZATION_GUIDE.md     # Creating custom controllers
    │   ├── MQTT_IMPLEMENTATION.md     # MQTT usage guide
    │   └── MQTT_COMPLETION_SUMMARY.md # MQTT implementation status
    ├── Mcontrol_HID/                   # Human interface controller
    ├── Mcontrol_Relay_Bank/            # Relay output controller
    ├── Mcontrol_Polled_Sensor_Bank/    # Sensor input controller
    ├── Mcontrol_GPS_Interface/         # GPS controller
    ├── Mcontrol_Camera_Interface/      # Camera controller
    ├── Mcontrol_Display/               # Display controller
    ├── Mcontrol_ODBII_Interface/       # OBD-II diagnostics
    └── ... (more specialized controllers)
```

## Development Workflow

1. **Master Controller**: Always start the master controller first
2. **Member Controllers**: Build from abstract template or existing controllers
3. **Testing**: Use REST API test scripts and MQTT clients for debugging
4. **Integration**: Test communication between controllers via MQTT
5. **Iteration**: Refine control mappings and event handling

## Contributing

This is a personal project, but feel free to use ideas and code for your own projects. All code is provided as-is under the project license.

## Hardware Requirements

- ESP32 or ESP32-S3 development boards
- 12V automotive power supply (with 3.3V regulation)
- Sensors, switches, relays as needed for specific controllers
- USB cables for programming and debugging

## Safety Notice

This project involves automotive electronics. Always:

- Use proper fuses and circuit protection
- Test thoroughly before vehicle integration
- Follow automotive electrical safety practices
- Never compromise critical vehicle systems
- Consult professionals for installation

## License

See LICENSE file for details.

## See Also

- **Melvin_controller_scheme.md**: Detailed architecture and API documentation
