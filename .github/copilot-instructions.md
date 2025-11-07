# Melvin - ESP32 Vehicle Controller System

## Project Overview
Melvin is a distributed microcontroller system for extending a 25-year-old Volvo with modern electronics. The system uses ESP32 controllers in a master-member WiFi network architecture to control lighting, sensors, cameras, and auxiliary devices.

## Architecture Principles

### Controller Hierarchy
- **Master Controller**: ESP32 with WiFi in AP mode - central coordinator
- **Member Controllers**: ESP32 clients with WiFi + Bluetooth 4.2 capabilities
- **Specialized Controllers**:
  - Human Interface Controller: switches, buttons, knobs, joysticks
  - Sensor Interface Controller: sensor data collection
  - External/Cabin Light Controllers: lighting management
  - Aux Device Controller: antenna, spotlight, PA system control

### Communication Patterns
- Master controller hosts WiFi network that all members join as clients
- RESTful API endpoints for device registration and control (see `/controls/`, `/register/`, `/join/`)
- 16-bit control values: Boolean for toggles, 0-32768 for continuous controls (0=off)
- Unique device identifiers required for all controllers

## Development Guidelines

### Control Abstractions
- Map physical controls to control base addresses and numbers
- Maps of both physical controls and groups support for human-friendly names in addition to numeric addresses
- Support "group" controls containing multiple output controls
- Support for nested control groups
- Support various control types:
  - Toggle switches (boolean)
  - Potentiometers (0-32768)
  - Rotary encoders (0-32768 with wrap-around)
  - Directional joysticks (2-axis, each 0-32768)
- RGB control groups use 3 continuous controls (0-32768 each)
- Implement flash patterns (wig-wag) where input controllers observe 0/+ boolean states

### File Organization
- `Melvin_master_controller/`: Master ESP32 controller code
- `Melvin_member_controller/`: Member ESP32 controller code  
- `Melvin_controller_scheme.md`: Architecture documentation and API endpoints
- `inspiration/`: Reference images for project goals

### Technology Stack
- Platform: ESP32 microcontrollers with ESP-IDF framework
- Communication: WiFi (AP/client), Bluetooth 4.2
- Control Protocol: HTTP REST API using standard ESP32 httpd implementation
- Data Format: JSON for all REST requests and responses
- Voltage: 12V automotive with 3.3V ADC for analog inputs

### Coding Conventions
- **Modular Design**: Break functionality into logical modules with clear interfaces
- **Verbose Comments**: Document all functions, structs, and complex logic thoroughly
- **Data Structures**: Use C++ structs to organize related data and improve code clarity
- **Naming**: Use descriptive names that clearly indicate purpose and data type
- **REST Format**: All API endpoints accept and return JSON payloads

## Key Implementation Notes
- Abstract mapping between physical controls and logical control addresses
- Support for various control types: toggle switches, potentiometers, rotary encoders, directional joysticks
- Flash/strobe pattern support for emergency lighting applications
- Device registration/deregistration for dynamic network membership
- Future-proofed for non-lighting applications (antenna control, spotlights, PA systems)

## Current Status
Project is in design/planning phase. Controller directories exist but contain no implementation code yet. 

### Development Framework
- Use ESP-IDF (not Arduino framework) for all ESP32 development
- Implement REST APIs using standard ESP32 httpd component
- Structure code with clear module separation and comprehensive documentation
- Define data structures using C++ structs for parameter passing and data organization