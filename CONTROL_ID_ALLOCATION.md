# Control ID Allocation Guide

This document defines the recommended control ID ranges for different device types in the Melvin system to prevent conflicts and maintain organization.

## Overview

Each control in the Melvin system requires a unique ID. To avoid conflicts between devices, control IDs are organized into ranges based on device type.

## ID Allocation Scheme

Control IDs are 16-bit unsigned integers (0-65535). The allocation uses the following structure:

```text
Device Type Base Address Range: 1000-60000 (increments of 1000)
Controls per Device: Up to 999 controls per device instance
```

## Standard ID Ranges

| Device Type | Base Range | Max Devices | Controls per Device | Total IDs |
|-------------|------------|-------------|---------------------|-----------|
| **HID (Human Interface)** | 1000-1999 | 1 | 1000 | 1000-1999 |
| **Relay Bank** | 2000-2999 | 1 | 1000 | 2000-2999 |
| **Polled Sensor Bank** | 3000-3999 | 1 | 1000 | 3000-3999 |
| **Event Sensor Bank** | 4000-4999 | 1 | 1000 | 4000-4999 |
| **GPS Interface** | 5000-5999 | 1 | 1000 | 5000-5999 |
| **ODBII Interface** | 6000-6999 | 1 | 1000 | 6000-6999 |
| **Display** | 7000-7999 | 1 | 1000 | 7000-7999 |
| **Camera Interface** | 8000-8999 | 1 | 1000 | 8000-8999 |
| **Spotlight Interface** | 9000-9999 | 1 | 1000 | 9000-9999 |
| **APRS Interface** | 10000-10999 | 1 | 1000 | 10000-10999 |
| **Logger** | 11000-11999 | 1 | 1000 | 11000-11999 |
| **Custom/Future** | 12000-65535 | - | - | Reserved |

## Multiple Device Instances

If you need multiple instances of the same device type (e.g., two relay banks), allocate them sequentially within the same range:

### Example: Two Relay Banks

```cpp
// Relay Bank 1
#define RELAY_BANK_1_BASE 2000
// Controls: 2000-2099

// Relay Bank 2  
#define RELAY_BANK_2_BASE 2100
// Controls: 2100-2199
```

Each device instance should document its control allocation within its base range.

## Control Numbering Within Device

Controls within a device use the `base_address + control_number` scheme:

```cpp
#define CONTROL_BASE_ADDRESS 1000  // HID device

// Individual controls
#define CTRL_EMERGENCY_BUTTON  1   // ID: 1001
#define CTRL_AUX_BUTTON       2   // ID: 1002
#define CTRL_MAIN_SWITCH      3   // ID: 1003
#define CTRL_BRIGHTNESS_POT   4   // ID: 1004
```

## Example Configurations

### HID Controller Example

```cpp
// Device: Mcontrol_HID
#define CONTROL_BASE_ADDRESS 1000

// Digital controls (1001-1020)
add_control("Emergency Button", 1, CONTROL_BOOLEAN);    // ID: 1001
add_control("Horn Button", 2, CONTROL_BOOLEAN);         // ID: 1002
add_control("Light Toggle", 3, CONTROL_BOOLEAN);        // ID: 1003

// Analog controls (1021-1040)
add_control("Brightness", 21, CONTROL_CONTINUOUS);      // ID: 1021
add_control("Volume", 22, CONTROL_CONTINUOUS);          // ID: 1022

// Multi-axis controls (1041-1060)
add_control("Spotlight Joy", 41, CONTROL_JOYSTICK_2AXIS); // ID: 1041

// RGB controls (1061-1080)
add_control("Cabin Light", 61, CONTROL_RGB_GROUP);      // ID: 1061
```

### Relay Bank Example

```cpp
// Device: Mcontrol_Relay_Bank
#define CONTROL_BASE_ADDRESS 2000

// High-current relays (2001-2016)
add_control("Headlights", 1, CONTROL_BOOLEAN);          // ID: 2001
add_control("Fog Lights", 2, CONTROL_BOOLEAN);          // ID: 2002
add_control("Work Lights", 3, CONTROL_BOOLEAN);         // ID: 2003
add_control("Emergency Lights", 4, CONTROL_BOOLEAN);    // ID: 2004

// PWM-capable outputs (2017-2032)
add_control("Dimmer 1", 17, CONTROL_CONTINUOUS);        // ID: 2017
add_control("Dimmer 2", 18, CONTROL_CONTINUOUS);        // ID: 2018
```

### GPS Interface Example

```cpp
// Device: Mcontrol_GPS_Interface
#define CONTROL_BASE_ADDRESS 5000

// GPS data controls (5001-5020)
add_control("Latitude", 1, CONTROL_CONTINUOUS);         // ID: 5001
add_control("Longitude", 2, CONTROL_CONTINUOUS);        // ID: 5002
add_control("Altitude", 3, CONTROL_CONTINUOUS);         // ID: 5003
add_control("Speed", 4, CONTROL_CONTINUOUS);            // ID: 5004
add_control("Heading", 5, CONTROL_CONTINUOUS);          // ID: 5005
add_control("Satellites", 6, CONTROL_CONTINUOUS);       // ID: 5006
add_control("Fix Quality", 7, CONTROL_CONTINUOUS);      // ID: 5007
```

### Sensor Bank Example

```cpp
// Device: Mcontrol_Polled_Sensor_Bank  
#define CONTROL_BASE_ADDRESS 3000

// Environmental sensors (3001-3020)
add_control("Cabin Temp", 1, CONTROL_CONTINUOUS);       // ID: 3001
add_control("External Temp", 2, CONTROL_CONTINUOUS);    // ID: 3002
add_control("Cabin Humidity", 3, CONTROL_CONTINUOUS);   // ID: 3003
add_control("Barometric Pressure", 4, CONTROL_CONTINUOUS); // ID: 3004

// Vehicle sensors (3021-3040)
add_control("Battery Voltage", 21, CONTROL_CONTINUOUS); // ID: 3021
add_control("Alternator Current", 22, CONTROL_CONTINUOUS); // ID: 3022

// Orientation sensors (3041-3060)
add_control("Pitch", 41, CONTROL_CONTINUOUS);           // ID: 3041
add_control("Roll", 42, CONTROL_CONTINUOUS);            // ID: 3042
add_control("Yaw", 43, CONTROL_CONTINUOUS);             // ID: 3043
```

## Best Practices

### 1. Document Your Allocations

Always document control IDs in your device's README or header file:

```cpp
/**
 * Control ID Allocation for MyDevice
 * Base Address: 8000
 * 
 * 8001-8010: Digital inputs
 * 8011-8020: Analog inputs  
 * 8021-8030: Outputs
 * 8031-8040: Status indicators
 */
```

### 2. Group Related Controls

Use logical groupings within your range:

- 01-20: Digital inputs
- 21-40: Analog inputs
- 41-60: Multi-axis controls
- 61-80: RGB/color controls
- 81-99: Status/diagnostic

### 3. Leave Gaps for Future Expansion

Don't pack controls tightly - leave room for adding features:

```cpp
// Leave gaps between groups
#define LIGHTS_BASE    1   // 1001-1020
#define SENSORS_BASE   21  // 1021-1040  
#define MOTORS_BASE    41  // 1041-1060
// 1061-1099 reserved for future use
```

### 4. Use Meaningful Names

Control numbers should be memorable and related to function:

```cpp
// Good
#define CTRL_LEFT_TURN    10  // Easy to remember
#define CTRL_RIGHT_TURN   11  // Sequential and logical

// Avoid
#define CTRL_SOMETHING    73  // Random, hard to remember
```

## Control ID Assignment in Code

### Automatic Assignment

The master controller automatically assigns IDs when controls are registered:

```cpp
// Member controller registers control
POST /controls/
{
    "base_address": 1000,
    "control_number": 5,
    "type": 0,
    "name": "Button 1"
}

// Master responds with assigned ID
{
    "control_id": 1005,
    "status": "created"
}
```

### Manual Tracking

Member controllers track their control IDs after registration:

```cpp
struct MemberControl {
    uint16_t control_id;        // Assigned by master
    uint16_t base_address;      // 1000
    uint16_t control_number;    // 5
    // ... other fields
};
```

## Conflict Resolution

If you encounter ID conflicts:

1. **Check allocation table** - Ensure you're using the correct range
2. **Review device instances** - Multiple devices may be overlapping
3. **Adjust control numbers** - Use gaps within your allocated range
4. **Document exceptions** - Note any non-standard allocations

## Future Enhancements

Planned improvements to the ID allocation system:

- **Automatic validation** - Master controller validates IDs are in correct range
- **Collision detection** - Master warns when IDs conflict
- **Dynamic allocation** - Master can auto-assign IDs without device specification
- **ID remapping** - Support for changing IDs without device reconfiguration

## Summary

- Use the allocated base range for your device type
- Number controls sequentially starting from 1
- Document your control allocations
- Leave gaps for future expansion
- Test for conflicts during development

For questions or to request a new device type range, update this document and submit a pull request.
