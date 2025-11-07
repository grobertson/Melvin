# Getting Started with Melvin Master Controller

This guide will help you build, deploy, and test the Melvin Master Controller using PlatformIO.

## Prerequisites

### Software Requirements

1. **VS Code** with PlatformIO extension installed
   - Or PlatformIO Core CLI
2. **Python 3.7+** for testing scripts
3. **Git** (for version control)

### Hardware Requirements

- ESP32 or ESP32-S3 development board
- USB cable for programming
- 12V automotive power supply (for final deployment)

## Quick Start

### 1. Install PlatformIO

If using VS Code:

1. Install the PlatformIO IDE extension
2. Restart VS Code

If using command line:

```bash
# Install PlatformIO Core
pip install platformio
```

### 2. Build the Project

```bash
# Navigate to the project directory
cd Melvin_master_controller

# Build the firmware
pio run

# Or in VS Code: Ctrl+Shift+P -> "PlatformIO: Build"
```

### 3. Upload to ESP32

```bash
# Upload firmware to connected ESP32
pio run --target upload

# Or in VS Code: Click the "Upload" button in PlatformIO toolbar
```

### 4. Monitor Serial Output

```bash
# Open serial monitor
pio device monitor

# Or in VS Code: Click the "Serial Monitor" button
```

## Testing the System

### 1. Connect to WiFi Network

After uploading the firmware:

1. The ESP32 will create a WiFi access point: `Melvin_Control_Network`
2. Password: `MelvinController2025`
3. Connect your computer to this network
4. The ESP32 IP will be: `192.168.4.1`

### 2. Test the API

```bash
# Install Python dependencies
pip install requests

# Run the API test script
python api_test.py

# Or specify custom IP
python api_test.py 192.168.4.1
```

### 3. Run Member Controller Simulation

```bash
# Start a simulated member controller
python member_controller_example.py

# This will:
# - Register as a device
# - Create various controls
# - Simulate input changes
# - Send periodic heartbeats
```

## Project Structure

```text
Melvin_master_controller/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp            # Main controller code
├── include/
│   └── melvin_config.h     # Configuration and data structures
├── api_test.py             # API testing script
├── member_controller_example.py  # Example member controller
└── README.md               # Documentation
```

## API Endpoints

### Device Management

- `POST /register/device/` - Register a new member controller
- `POST /deregister/device/` - Remove a member controller  
- `GET /devices/` - List all registered devices

### Control Management

- `GET /controls/` - List all controls
- `POST /controls/` - Create a new control
- `PUT /controls/{id}` - Update control value

## Control Types

| Type | Value Range | Description | Example |
|------|-------------|-------------|---------|
| Boolean (0) | 0 or 1 | Toggle switches, buttons | Emergency lights |
| Continuous (1) | 0-32768 | Potentiometers, sliders | Brightness control |
| Rotary (2) | 0-32768 | Rotary encoders | Frequency selector |
| Joystick (3) | Two 0-32768 values | 2-axis joystick | Spotlight direction |
| RGB (4) | Three 0-32768 values | Color controls | Cabin lighting |

## Flash Patterns

| Pattern | ID | Description |
|---------|----| -------------|
| None | 0 | No flashing |
| Wig-wag | 1 | Alternating on/off (1 second period) |
| Strobe | 2 | Fast flashing |
| Pulse | 3 | Smooth fade in/out |
| Alternating | 4 | Custom alternating pattern |

## Troubleshooting

### Build Issues

```bash
# Clean build cache
pio run --target clean

# Update PlatformIO platforms
pio platform update
```

### Upload Issues

```bash
# Check connected devices
pio device list

# Force upload port
pio run --target upload --upload-port COM3  # Windows
pio run --target upload --upload-port /dev/ttyUSB0  # Linux
```

### WiFi Issues

1. Check that ESP32 AP is broadcasting
2. Verify network name: `Melvin_Control_Network`
3. Check serial monitor for IP address
4. Try connecting manually to 192.168.4.1

### API Issues

1. Ensure computer is connected to ESP32 WiFi network
2. Test basic connectivity: `ping 192.168.4.1`
3. Check serial monitor for error messages
4. Verify JSON format in requests

## Development Tips

### Serial Monitor

- Baud rate: 115200
- Shows device registrations, control updates, and system status
- Use for debugging API calls and device communication

### Adding New Control Types

1. Add enum value to `ControlType` in `melvin_config.h`
2. Update control creation and update handlers in `main.cpp`
3. Test with API or member controller simulation

### Customizing Network Settings

Edit these constants in `melvin_config.h`:

```cpp
#define WIFI_SSID "Your_Network_Name"
#define WIFI_PASS "Your_Password" 
#define WIFI_CHANNEL 6
```

## Next Steps

1. **Create Member Controllers**: Use `member_controller_example.py` as a template to create specific device controllers
2. **Add Physical Controls**: Connect switches, potentiometers, and other inputs to ESP32 GPIO pins
3. **Implement Output Controls**: Add relay drivers, LED controllers, and motor controllers
4. **Vehicle Integration**: Design automotive-grade power supplies and mounting systems

## Support

For issues or questions:

1. Check the serial monitor output
2. Verify network connectivity
3. Test with the provided example scripts
4. Review the API documentation above
