# How to Create a Custom Member Controller

This guide shows how to use the Abstract Member Controller template to create a specific member controller for your application.

## Step 1: Copy the Template

```bash
# Copy the entire Abstract directory
cp -r Mcontrol_Abstract Mcontrol_YourNewController
cd Mcontrol_YourNewController
```

## Step 2: Customize Configuration

Edit `include/config.h` to match your specific controller:

```cpp
// Device Configuration - CHANGE THESE
#define DEVICE_TYPE DEVICE_HUMAN_INTERFACE  // Or other appropriate type
#define DEVICE_NAME "Dashboard Control Panel"
#define DEVICE_ID "HIC_001"  // Unique identifier

// Control Configuration  
#define CONTROL_BASE_ADDRESS 2000    // Different for each controller type

// GPIO Pin Assignments - CHANGE TO MATCH YOUR HARDWARE
#define PIN_BUTTON_1 12
#define PIN_SWITCH_1 14
#define PIN_POT_1 A0
// ... etc
```

## Step 3: Implement Hardware-Specific Functions

In `src/main.cpp`, modify the `ConcreteAbstractController` class:

### setup_hardware()

```cpp
void setup_hardware() override {
    // Initialize YOUR specific hardware
    pinMode(PIN_YOUR_BUTTON, INPUT_PULLUP);
    pinMode(PIN_YOUR_LED, OUTPUT);
    // Add I2C, SPI, sensor initialization, etc.
}
```

### read_inputs()

```cpp
void read_inputs() override {
    // Read YOUR specific inputs
    bool emergency_button = !digitalRead(PIN_EMERGENCY);
    if (emergency_button) {
        set_control_value("Emergency Stop", CONTROL_VALUE_BOOLEAN_ON);
    }
    
    // Read sensors, encoders, etc.
    int sensor_value = read_your_sensor();
    set_control_value("Sensor Reading", sensor_value);
}
```

### write_outputs()

```cpp
void write_outputs() override {
    // Control YOUR specific outputs
    uint16_t light_brightness, dummy_y, dummy_z;
    if (get_control_value("Light Brightness", light_brightness, dummy_y, dummy_z)) {
        analogWrite(PIN_LIGHT_PWM, map(light_brightness, 0, 32768, 0, 255));
    }
}
```

## Step 4: Define Your Controls

In the `initialize()` function, add your specific controls:

```cpp
// Remove the default template controls and add yours:
if (!add_control("Emergency Stop", 1, CONTROL_BOOLEAN)) return false;
if (!add_control("Engine RPM", 2, CONTROL_CONTINUOUS)) return false;
if (!add_control("Steering Angle", 3, CONTROL_ROTARY)) return false;
if (!add_control("Camera Control", 4, CONTROL_JOYSTICK_2AXIS)) return false;
if (!add_control("Interior Lighting", 5, CONTROL_RGB_GROUP)) return false;
```

## Step 5: Test and Build

```bash
# Build your custom controller
pio run

# Upload to ESP32
pio run --target upload

# Monitor for debugging
pio device monitor
```

## Example: Human Interface Controller

Here's a complete example for a dashboard control panel:

### config.h changes

```cpp
#define DEVICE_TYPE DEVICE_HUMAN_INTERFACE
#define DEVICE_NAME "Dashboard Control Panel"
#define DEVICE_ID "HIC_DASH_001"
#define CONTROL_BASE_ADDRESS 1000

// Physical connections
#define PIN_EMERGENCY_BUTTON 12
#define PIN_LIGHT_SWITCH 13
#define PIN_BRIGHTNESS_POT A0
#define PIN_VOLUME_POT A1
#define PIN_STATUS_LED 2
```

### main.cpp changes

```cpp
void setup_hardware() override {
    pinMode(PIN_EMERGENCY_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LIGHT_SWITCH, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    analogReadResolution(12);
}

void read_inputs() override {
    // Emergency button
    static bool last_emergency_state = false;
    bool emergency_state = !digitalRead(PIN_EMERGENCY_BUTTON);
    if (emergency_state != last_emergency_state) {
        set_control_value("Emergency Stop", emergency_state ? 1 : 0);
        last_emergency_state = emergency_state;
    }
    
    // Light switch
    static bool last_light_state = false;
    bool light_state = !digitalRead(PIN_LIGHT_SWITCH);
    if (light_state != last_light_state) {
        set_control_value("Main Lights", light_state ? 1 : 0);
        last_light_state = light_state;
    }
    
    // Potentiometers
    uint16_t brightness = map_analog_to_control(analogRead(PIN_BRIGHTNESS_POT));
    uint16_t volume = map_analog_to_control(analogRead(PIN_VOLUME_POT));
    
    set_control_value("Brightness", brightness);
    set_control_value("Volume", volume);
}

// In initialize():
add_control("Emergency Stop", 1, CONTROL_BOOLEAN);
add_control("Main Lights", 2, CONTROL_BOOLEAN);  
add_control("Brightness", 3, CONTROL_CONTINUOUS);
add_control("Volume", 4, CONTROL_CONTINUOUS);
```

## Controller Type Examples

### Sensor Interface Controller

- Read temperature, pressure, GPS, accelerometer
- Send periodic sensor data to master
- Handle calibration and filtering

### External Light Controller  

- Receive commands from master for light patterns
- Control LED strips, emergency lights, work lights
- Implement flash patterns locally

### Relay Bank Controller

- Control multiple relays for various systems
- Provide feedback on relay states
- Handle overcurrent protection

## Best Practices

1. **Unique Identifiers**: Always use unique DEVICE_ID values
2. **Base Addresses**: Use different CONTROL_BASE_ADDRESS for each controller type  
3. **Error Handling**: Implement proper error handling in handle_error()
4. **Debouncing**: Always debounce mechanical inputs
5. **Deadbands**: Use deadbands on analog inputs to prevent noise
6. **Status Indication**: Use LEDs to show connection and error status
7. **Documentation**: Comment your specific hardware connections and logic

## Troubleshooting

- Check serial monitor for debug messages
- Verify GPIO pin assignments match your hardware
- Ensure WiFi credentials are correct
- Test individual components before integration
- Use multimeter to verify hardware connections
