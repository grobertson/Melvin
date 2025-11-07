# Mcontrol_ODBII_Interface

ESP32-based OBD-II interface controller for the Melvin vehicle electronics system. This member controller connects to Bluetooth OBD-II adapters to retrieve vehicle diagnostic data and proxy selected parameters to the Melvin master controller over WiFi.

## Overview

The OBD-II Interface Controller bridges between standard automotive OBD-II diagnostic systems and the Melvin distributed control network. It provides configurable data filtering, update intervals, and test modes for development and validation.

### Key Features

- **Bluetooth OBD-II Connectivity**: Interfaces with standard ELM327-compatible Bluetooth OBD-II adapters
- **Configurable Data Proxying**: Selectively forward specific OBD-II parameters to master controller
- **Adjustable Update Rates**: Configurable polling intervals for different data types
- **Test Mode**: Generate simulated OBD-II data for development and testing
- **Network Integration**: Full integration with Melvin WiFi control network
- **Error Recovery**: Automatic reconnection and error handling for both Bluetooth and WiFi

## Architecture

```text
┌─────────────────┐    Bluetooth     ┌─────────────────┐    WiFi/REST   ┌─────────────────┐
│   Vehicle ECU   │ ◄─────────────►  │   ESP32 OBD-II  │ ◄────────────► │ Melvin Master   │
│                 │                  │   Interface     │                │   Controller    │
│ - Engine Data   │                  │                 │                │                 │
│ - Transmission  │                  │ - Data Filtering│                │ - Data Logging  │
│ - Emissions     │                  │ - Rate Control  │                │ - Display       │
│ - Diagnostics   │                  │ - Test Mode     │                │ - Alerts        │
└─────────────────┘                  └─────────────────┘                └─────────────────┘
```

## Supported OBD-II Parameters

### Engine Data

- **Engine RPM** (PID 0x0C): Real-time engine speed
- **Engine Load** (PID 0x04): Calculated engine load percentage
- **Engine Coolant Temperature** (PID 0x05): Coolant temperature in °C
- **Intake Air Temperature** (PID 0x0F): Air intake temperature
- **Throttle Position** (PID 0x11): Throttle position percentage

### Fuel System

- **Fuel System Status** (PID 0x03): Fuel system operational status
- **Fuel Level** (PID 0x2F): Fuel tank level percentage
- **Fuel Rail Pressure** (PID 0x23): Fuel rail pressure (gas engines)
- **Short Term Fuel Trim** (PID 0x06): Bank 1 fuel trim percentage
- **Long Term Fuel Trim** (PID 0x07): Bank 1 long-term fuel trim

### Vehicle Speed & Performance

- **Vehicle Speed** (PID 0x0D): Current vehicle speed in km/h
- **MAF Air Flow Rate** (PID 0x10): Mass airflow sensor reading
- **Intake Manifold Pressure** (PID 0x0B): Absolute manifold pressure

### Emissions & Environment

- **Oxygen Sensor Values** (PIDs 0x14-0x1B): O2 sensor readings
- **Catalyst Temperature** (PIDs 0x3C-0x3F): Catalytic converter temperatures
- **EGR Error** (PID 0x2D): Exhaust gas recirculation error

### Diagnostic Information

- **Diagnostic Trouble Codes (DTCs)**: Active and pending fault codes
- **Freeze Frame Data**: Snapshot data when DTCs were set
- **Readiness Monitors**: Emission system readiness status

## Configuration

### Device Settings

```cpp
#define DEVICE_TYPE DEVICE_ODBII_INTERFACE
#define DEVICE_NAME "OBD-II Interface"
#define DEVICE_ID "ODBII_001"
#define CONTROL_BASE_ADDRESS 5000
```

### OBD-II Configuration

```cpp
// Bluetooth OBD-II Adapter Settings
#define ODBII_BLUETOOTH_NAME "OBDII"           // Default adapter name
#define ODBII_BLUETOOTH_PIN "1234"             // Default pairing PIN
#define ODBII_BAUD_RATE 38400                  // ELM327 default baud rate

// Data Update Intervals (milliseconds)
#define ODBII_FAST_UPDATE_INTERVAL 250         // Engine RPM, speed (4 Hz)
#define ODBII_MEDIUM_UPDATE_INTERVAL 1000      // Temperatures, pressures (1 Hz)
#define ODBII_SLOW_UPDATE_INTERVAL 5000        // Fuel level, diagnostics (0.2 Hz)

// Data Filtering
#define MAX_ODBII_PARAMETERS 32                // Maximum trackable parameters
#define ENABLE_DTC_MONITORING true             // Monitor diagnostic trouble codes
#define ENABLE_FREEZE_FRAME true               // Capture freeze frame data
```

### Parameter Selection

```cpp
// Configurable parameter list with update priorities
struct OBDIIParameter {
    uint8_t pid;                    // OBD-II Parameter ID
    const char* name;               // Human-readable name
    uint8_t update_group;           // Update frequency group (0=fast, 1=medium, 2=slow)
    bool enabled;                   // Enable/disable this parameter
    uint16_t scaling_factor;        // Scaling for Melvin 16-bit values
};
```

## Control Interface

### Exposed Controls

The controller exposes the following controls to the Melvin master:

| Control Name | Type | Address | Description |
|-------------|------|---------|-------------|
| Engine RPM | Continuous | 5001 | Engine speed (0-8000 RPM mapped to 0-32768) |
| Vehicle Speed | Continuous | 5002 | Vehicle speed (0-255 km/h mapped to 0-32768) |
| Engine Load | Continuous | 5003 | Engine load percentage (0-100% mapped to 0-32768) |
| Coolant Temperature | Continuous | 5004 | Coolant temp (-40°C to 215°C mapped to 0-32768) |
| Fuel Level | Continuous | 5005 | Fuel tank level (0-100% mapped to 0-32768) |
| Throttle Position | Continuous | 5006 | Throttle position (0-100% mapped to 0-32768) |
| OBD Connection Status | Boolean | 5007 | Bluetooth OBD-II adapter connection status |
| DTC Count | Continuous | 5008 | Number of active diagnostic trouble codes |
| Test Mode Active | Boolean | 5009 | Indicates if test mode is running |

### Configuration Controls

| Control Name | Type | Address | Description |
|-------------|------|---------|-------------|
| Fast Update Rate | Continuous | 5010 | Fast polling interval (100-2000ms mapped to 0-32768) |
| Medium Update Rate | Continuous | 5011 | Medium polling interval (500-10000ms) |
| Slow Update Rate | Continuous | 5012 | Slow polling interval (2000-30000ms) |
| Enable Test Mode | Boolean | 5013 | Activate simulated data generation |

## Implementation Details

### Core Classes

#### OBDIIController

```cpp
class OBDIIController : public AbstractMemberController {
public:
    bool initialize() override;
    void read_inputs() override;
    void write_outputs() override;
    void handle_error(const char* error_msg) override;
    
private:
    BluetoothSerial bluetooth;
    std::vector<OBDIIParameter> parameters;
    unsigned long last_fast_update;
    unsigned long last_medium_update; 
    unsigned long last_slow_update;
    bool test_mode_active;
};
```

#### Parameter Management

```cpp
class OBDIIParameterManager {
public:
    bool add_parameter(uint8_t pid, const char* name, uint8_t group);
    bool update_parameter(uint8_t pid);
    uint16_t get_scaled_value(uint8_t pid);
    bool is_parameter_stale(uint8_t pid, unsigned long max_age_ms);
    
private:
    std::map<uint8_t, OBDIIParameter> parameter_map;
    std::map<uint8_t, unsigned long> last_update_time;
};
```

### OBD-II Communication Protocol

#### ELM327 Command Interface

```cpp
// Standard ELM327 initialization sequence
const char* INIT_COMMANDS[] = {
    "ATZ",      // Reset
    "ATE0",     // Echo off
    "ATL0",     // Linefeeds off
    "ATS0",     // Spaces off
    "ATH1",     // Headers on
    "ATSP0"     // Auto protocol detection
};

// PID query format: "01 [PID]" for Mode 1 data
// Example: "01 0C" for Engine RPM
```

#### Data Parsing

```cpp
bool parse_obd_response(const String& response, uint8_t pid, uint16_t& value) {
    // Parse hex response and apply PID-specific scaling
    // Handle multi-byte responses and error conditions
    // Convert to Melvin 16-bit control value format
}
```

### Test Mode Implementation

#### Simulated Data Generation

```cpp
class OBDIITestMode {
public:
    void generate_engine_data();
    void generate_driving_scenario();
    void inject_dtc_events();
    
private:
    uint16_t simulated_rpm;
    uint16_t simulated_speed;
    bool engine_running;
    unsigned long scenario_start_time;
};
```

#### Test Scenarios

- **Idle Engine**: Steady 800 RPM, 0 speed, normal temperatures
- **City Driving**: Variable RPM 1500-3000, speed 0-60 km/h
- **Highway Cruise**: Steady 2000 RPM, 100 km/h, optimal fuel trim
- **Cold Start**: Temperature ramp-up, rich fuel mixture
- **Diagnostic Events**: Simulated DTC generation and clearing

## Error Handling

### Bluetooth Connection Issues

- **Connection Lost**: Automatic reconnection attempts every 30 seconds
- **Adapter Not Found**: Scan for available OBD-II adapters
- **Authentication Failed**: Retry with common PIN codes (1234, 0000)
- **Communication Timeout**: Reset adapter and reinitialize

### OBD-II Protocol Errors

- **No Response**: Mark parameter as stale, continue with other parameters
- **Invalid Data**: Log error, use last known good value
- **Bus Init Failed**: Vehicle may not support OBD-II or ignition off
- **Protocol Mismatch**: Try alternative protocols (CAN, ISO, etc.)

### Network Integration

- **WiFi Connection Lost**: Maintain OBD-II logging, reconnect when available
- **Master Controller Unreachable**: Buffer critical data, resume when connected
- **Registration Failed**: Retry with exponential backoff

## Installation and Setup

### Hardware Requirements

- ESP32 development board with Bluetooth support
- ELM327 Bluetooth OBD-II adapter
- Vehicle with OBD-II port (1996+ in US, 2001+ in EU)
- 12V to 3.3V power supply for ESP32

### Software Dependencies

```ini
[lib_deps]
    bblanchon/ArduinoJson@^6.19.4
    espressif/esp32@^2.0.5
    BluetoothSerial
```

### Build and Deploy

```bash
# Clone the Melvin repository
git clone [repository-url]
cd Melvin/Melvin_member_controllers/Mcontrol_ODBII_Interface

# Build with PlatformIO
pio run

# Upload to ESP32
pio run --target upload

# Monitor for debugging
pio device monitor
```

### Configuration Steps

1. **Pair OBD-II Adapter**: Use smartphone to verify adapter functionality
2. **Vehicle Connection**: Connect adapter to vehicle OBD-II port
3. **ESP32 Configuration**: Update WiFi credentials and device ID
4. **Parameter Selection**: Enable desired OBD-II parameters in configuration
5. **Testing**: Verify data flow using test mode before vehicle connection

## Usage Examples

### Basic Data Monitoring

```cpp
// Read engine RPM for display on dashboard
uint16_t engine_rpm, dummy_y, dummy_z;
if (master.get_control_value("Engine RPM", engine_rpm, dummy_y, dummy_z)) {
    float actual_rpm = map_control_to_float(engine_rpm, 0, 8000);
    display_engine_rpm(actual_rpm);
}
```

### Diagnostic Alert System

```cpp
// Monitor for diagnostic trouble codes
uint16_t dtc_count, dummy_y, dummy_z;
if (master.get_control_value("DTC Count", dtc_count, dummy_y, dummy_z)) {
    if (dtc_count > 0) {
        // Trigger warning light or notification
        set_warning_light(true);
        log_diagnostic_event();
    }
}
```

### Performance Monitoring

```cpp
// Track fuel efficiency and performance metrics
struct VehicleState {
    float speed_kmh;
    float engine_load_pct;
    float fuel_level_pct;
    bool dtc_active;
};

VehicleState get_current_vehicle_state() {
    VehicleState state;
    // Retrieve all relevant OBD-II parameters
    // Calculate derived metrics (fuel economy, etc.)
    return state;
}
```

## Troubleshooting

### Common Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| No OBD-II Data | All values show 0 or stale | Check vehicle ignition, OBD-II adapter pairing |
| Intermittent Connection | Data drops out periodically | Check Bluetooth range, adapter power |
| Wrong Values | Data seems incorrect | Verify PID definitions, check scaling factors |
| High CPU Usage | ESP32 overheating/crashing | Reduce update frequencies, optimize parsing |

### Debug Procedures

1. **Serial Monitor**: Check for Bluetooth and OBD-II communication logs
2. **Test Mode**: Verify control interface without vehicle connection
3. **Bluetooth Scan**: Ensure OBD-II adapter is discoverable
4. **Manual Commands**: Send AT commands directly to adapter
5. **Network Connectivity**: Verify registration with master controller

## Development Status

- ✅ **Architecture Design**: Complete
- ✅ **Documentation**: Complete
- ⏳ **Implementation**: Pending
- ⏳ **Testing**: Pending
- ⏳ **Integration**: Pending

## Future Enhancements

### Advanced Features

- **CAN Bus Direct Access**: Bypass OBD-II for manufacturer-specific data
- **Real-time Alerting**: Immediate notifications for critical parameters
- **Data Logging**: Local storage for diagnostic history
- **Wireless OBD-II**: Support for WiFi-based OBD-II adapters
- **Multi-Vehicle Support**: Switch between different vehicle profiles

### Integration Possibilities

- **Navigation System**: Speed and fuel data for route optimization
- **Maintenance Tracking**: Automatic service interval monitoring
- **Performance Logging**: Track acceleration, braking, fuel economy
- **Fleet Management**: Multi-vehicle monitoring dashboard

## Related Documentation

- [Melvin Controller Architecture](../../Melvin_controller_scheme.md)
- [Abstract Member Controller](../Mcontrol_Abstract/README.md)
- [Master Controller API](../../Melvin_master_controller/README.md)
- [Network Configuration Guide](../Mcontrol_Abstract/CUSTOMIZATION_GUIDE.md)

## License

This project is part of the Melvin vehicle electronics system. See the main project LICENSE file for details.
