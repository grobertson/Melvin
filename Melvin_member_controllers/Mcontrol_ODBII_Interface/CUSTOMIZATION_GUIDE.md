# OBD-II Interface Controller Customization Guide

This guide explains how to customize and configure the OBD-II Interface Controller for your specific vehicle and requirements.

## Quick Start

1. **Hardware Setup**:
   - Connect ESP32 to 12V automotive power with proper regulation to 3.3V
   - Ensure Bluetooth OBD-II adapter is paired and functional
   - Connect status LEDs to configured pins (optional but recommended)

2. **Configuration**:
   - Update WiFi credentials in `config.h` if different from defaults
   - Set unique DEVICE_ID if running multiple OBD-II controllers
   - Configure desired OBD-II parameters in `setup_obd_parameters()`

3. **Build and Deploy**:

   ```bash
   pio run --target upload
   pio device monitor
   ```

## Configuration Options

### Network Settings

```cpp
// In config.h
#define WIFI_SSID "Melvin_Control_Network"      // Change if using different network
#define WIFI_PASSWORD "MelvinController2025"    // Change if using different password
#define DEVICE_ID "ODBII_001"                   // Make unique for each controller
```

### OBD-II Adapter Settings

```cpp
// Bluetooth adapter configuration
#define ODBII_BLUETOOTH_NAME "OBDII"            // Change to match your adapter
#define ODBII_BLUETOOTH_PIN "1234"              // Change if adapter uses different PIN
#define ODBII_BAUD_RATE 38400                   // Usually 38400 for ELM327

// Connection timeouts
#define ODBII_CONNECTION_TIMEOUT_MS 10000       // Increase for slow adapters
#define ODBII_RESPONSE_TIMEOUT_MS 2000          // Increase for slow vehicles
```

### Update Frequencies

Customize how often different parameter groups are polled:

```cpp
// Fast parameters (engine RPM, speed) - affects responsiveness
#define ODBII_FAST_UPDATE_INTERVAL 250          // 4 Hz (every 250ms)

// Medium parameters (temperatures, pressures) - balance of accuracy and performance  
#define ODBII_MEDIUM_UPDATE_INTERVAL 1000       // 1 Hz (every 1000ms)

// Slow parameters (fuel level, diagnostics) - infrequent changes
#define ODBII_SLOW_UPDATE_INTERVAL 5000         // 0.2 Hz (every 5000ms)
```

## Parameter Customization

### Adding New Parameters

To monitor additional OBD-II parameters, add them to `setup_obd_parameters()`:

```cpp
void setup_obd_parameters() {
    // Existing parameters...
    
    // Add new parameter
    parameters.push_back(OBDIIParameter(
        0x46,                           // PID (Ambient Air Temperature)
        "Ambient Temperature",          // Human-readable name
        ODBII_UPDATE_GROUP_SLOW,        // Update frequency group
        true,                           // Enabled
        128                             // Scaling factor
    ));
}
```

Then add the corresponding control and parsing logic:

```cpp
// In initialize() method, add control:
if (!add_control("Ambient Temperature", 14, CONTROL_CONTINUOUS)) return false;

// In update_control_from_parameter(), add case:
case 0x46:
    set_control_value("Ambient Temperature", param.current_value);
    break;
```

### Removing Unused Parameters

To improve performance, disable parameters you don't need:

```cpp
// Set enabled = false for unused parameters
parameters.push_back(OBDIIParameter(0x2F, "Fuel Level", ODBII_UPDATE_GROUP_SLOW, false));
```

### Custom Parameter Groups

Create custom update groups for specific monitoring needs:

```cpp
// Add to config.h
#define ODBII_UPDATE_GROUP_CRITICAL 3   // Very high frequency for critical parameters

// Modify update intervals in read_inputs()
if (current_time - last_critical_update >= 100) {  // 10 Hz for critical data
    update_parameter_group(ODBII_UPDATE_GROUP_CRITICAL);
    last_critical_update = current_time;
}
```

## Vehicle-Specific Adaptations

### Ford Vehicles

```cpp
// Ford-specific PIDs (Mode 22)
parameters.push_back(OBDIIParameter(0x22F40C, "Transmission Temperature", ODBII_UPDATE_GROUP_MEDIUM));
parameters.push_back(OBDIIParameter(0x22F433, "Battery Voltage", ODBII_UPDATE_GROUP_SLOW));
```

### GM Vehicles

```cpp
// GM-specific PIDs
parameters.push_back(OBDIIParameter(0x22F403, "Fuel Pump Duty Cycle", ODBII_UPDATE_GROUP_MEDIUM));
parameters.push_back(OBDIIParameter(0x22F40E, "EGR Position", ODBII_UPDATE_GROUP_MEDIUM));
```

### Toyota/Lexus Vehicles

```cpp
// Toyota-specific PIDs
parameters.push_back(OBDIIParameter(0x2201, "Hybrid Battery SOC", ODBII_UPDATE_GROUP_FAST));
parameters.push_back(OBDIIParameter(0x2202, "Motor RPM", ODBII_UPDATE_GROUP_FAST));
```

## Advanced Features

### Custom Scaling Functions

For parameters that need complex scaling, modify `scale_obd_value()`:

```cpp
uint16_t scale_obd_value(uint16_t raw_value, const OBDIIParameter& param) {
    switch (param.pid) {
        case 0x46: // Ambient Air Temperature
            // Custom scaling: temp = raw - 40, then map to control range
            return map(raw_value - 40, -40, 85, 0, CONTROL_VALUE_MAX);
            
        case 0x22F40C: // Ford Transmission Temperature
            // Multi-byte scaling for Ford-specific PID
            return map(raw_value * 0.75 - 40, 0, 150, 0, CONTROL_VALUE_MAX);
            
        default:
            return raw_value * param.scaling_factor;
    }
}
```

### Error Handling Customization

Customize error recovery behavior:

```cpp
// In config.h
#define MAX_CONSECUTIVE_ERRORS 3        // Reduce for faster error recovery
#define BLUETOOTH_RECONNECT_INTERVAL_MS 15000  // Faster reconnection attempts
#define ODBII_MAX_RETRIES 5             // More retries for unreliable connections
```

### Test Mode Scenarios

Add custom test scenarios for development:

```cpp
class OBDIITestMode {
    // Add new scenario
    void generate_highway_scenario() {
        simulated_rpm = 2500;
        simulated_speed = 100;
        simulated_coolant_temp = 95;
        // Add other parameters...
    }
    
    // Add scenario selection
    void update_scenario() {
        unsigned long cycle_time = elapsed % 60000; // 60 second cycles
        
        if (cycle_time < 20000) {
            generate_city_scenario();
        } else if (cycle_time < 40000) {
            generate_highway_scenario();
        } else {
            generate_idle_scenario();
        }
    }
};
```

## Performance Optimization

### Memory Usage

Reduce memory footprint for resource-constrained applications:

```cpp
// In config.h
#define MAX_ODBII_PARAMETERS 16         // Reduce if monitoring fewer parameters
#define HTTP_REQUEST_TIMEOUT_MS 3000    // Reduce timeout for faster recovery
```

### CPU Usage

Optimize for better performance:

```cpp
// Increase main loop delay for less CPU usage
void loop() {
    if (g_controller) {
        g_controller->update();
    }
    delay(20); // Increase from 10ms to reduce CPU load
}

// Use fewer ADC samples for analog readings (if any)
#define ADC_SAMPLES 2  // Reduce from 4 to 2 samples
```

### Network Efficiency

Reduce network traffic:

```cpp
// Only send changes above a threshold
bool MelvinMemberController::set_control_value(const String& name, uint16_t value, uint16_t value_y, uint16_t value_z) {
    // Add minimum change threshold
    const uint16_t MIN_CHANGE_THRESHOLD = 10;
    
    if (abs((int)control.current_value - (int)value) < MIN_CHANGE_THRESHOLD) {
        return true; // Skip update if change is too small
    }
    
    // Existing implementation...
}
```

## Troubleshooting

### Common Configuration Issues

1. **Wrong Bluetooth Adapter Name**:

   ```cpp
   // Try multiple common names
   const char* adapter_names[] = {"OBDII", "OBD-II", "ELM327", "OBDLink", "BAFX", "Veepeak"};
   ```

2. **Incorrect Baud Rate**:

   ```cpp
   // Some adapters use different rates
   #define ODBII_BAUD_RATE 115200  // Try this if 38400 doesn't work
   ```

3. **Vehicle Compatibility**:

   ```cpp
   // Add protocol-specific initialization
   const char* init_commands[] = {
       "ATZ", "ATE0", "ATL0", "ATS0", "ATH1",
       "ATSP6",  // Force CAN protocol for newer vehicles
       "ATCAF0"  // Disable automatic formatting
   };
   ```

### Debug Configuration

Enable detailed debugging:

```cpp
// In config.h
#define DEBUG_VERBOSE 1
#define ODBII_DEBUG_ENABLED 1

// Add debug output to main loop
void loop() {
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 5000) {
        Serial.println("Free heap: " + String(ESP.getFreeHeap()));
        Serial.println("Network status: " + network_status_to_string(g_controller->get_network_status()));
        last_debug = millis();
    }
    
    // Existing loop code...
}
```

## Integration Examples

### Dashboard Display

```cpp
// Read OBD-II data for dashboard display
void update_dashboard() {
    uint16_t rpm, speed, temp, dummy_y, dummy_z;
    
    if (get_control_value("Engine RPM", rpm, dummy_y, dummy_z)) {
        float actual_rpm = map(rpm, 0, CONTROL_VALUE_MAX, 0, 8000);
        display_tachometer(actual_rpm);
    }
    
    if (get_control_value("Vehicle Speed", speed, dummy_y, dummy_z)) {
        float actual_speed = map(speed, 0, CONTROL_VALUE_MAX, 0, 255);
        display_speedometer(actual_speed);
    }
}
```

### Data Logging

```cpp
// Log vehicle data to SD card
void log_vehicle_data() {
    static unsigned long last_log = 0;
    if (millis() - last_log > 1000) { // Log every second
        
        String log_line = String(millis()) + ",";
        
        uint16_t value, dummy_y, dummy_z;
        if (get_control_value("Engine RPM", value, dummy_y, dummy_z)) {
            log_line += String(value) + ",";
        }
        if (get_control_value("Vehicle Speed", value, dummy_y, dummy_z)) {
            log_line += String(value) + ",";
        }
        
        // Write to SD card or serial
        Serial.println(log_line);
        last_log = millis();
    }
}
```

### Alert System

```cpp
// Monitor for diagnostic trouble codes
void check_vehicle_health() {
    uint16_t dtc_count, dummy_y, dummy_z;
    if (get_control_value("DTC Count", dtc_count, dummy_y, dummy_z)) {
        if (dtc_count > 0) {
            // Trigger warning system
            digitalWrite(PIN_WARNING_LED, HIGH);
            // Could send notification to master controller
            // Could trigger buzzer or other alert
        }
    }
    
    // Check coolant temperature
    uint16_t coolant_temp;
    if (get_control_value("Coolant Temperature", coolant_temp, dummy_y, dummy_z)) {
        float actual_temp = map(coolant_temp, 0, CONTROL_VALUE_MAX, -40, 215);
        if (actual_temp > 105) { // Overheating threshold
            // Trigger overheat warning
            digitalWrite(PIN_OVERHEAT_LED, HIGH);
        }
    }
}
```

This customization guide should help you adapt the OBD-II Interface Controller for your specific vehicle and monitoring requirements.
