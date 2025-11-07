# GPS Interface Controller

This member controller interfaces with GPS hardware to provide position, speed, and navigation data to the Melvin master controller. It includes a test mode that generates simulated NMEA data for development and testing.

## Overview

The GPS Interface Controller demonstrates:

- GPS hardware interfacing using TinyGPS++ library
- NMEA sentence parsing and processing
- Test mode with simulated GPS data for development
- Real-time GPS data collection and transmission
- Status monitoring and error handling
- LED indicators for GPS fix and data activity

## Key Features

### Core Functionality

- **Real GPS Mode**: Interfaces with actual GPS hardware via UART
- **Test Mode**: Generates simulated NMEA sentences for testing
- **Data Processing**: Converts GPS coordinates to Melvin control values
- **Status Monitoring**: Tracks GPS fix status and data activity
- **Error Handling**: Timeout detection and error reporting

### Control Types Supported

- Latitude (continuous 0-32768)
- Longitude (continuous 0-32768)
- Altitude (continuous 0-32768)
- Speed (continuous 0-32768)
- Course/Heading (continuous 0-32768)
- Satellite Count (continuous 0-32768)
- GPS Fix Status (boolean)

## Hardware Requirements

### GPS Module

- **Communication**: UART serial at 9600 baud
- **Protocol**: NMEA 0183 standard sentences
- **Recommended**: Any GPS module with UART output (u-blox, etc.)

### ESP32 Connections

```text
GPS Module    ESP32 Pin
----------    ---------
VCC          3.3V
GND          GND
TX           GPIO 16 (GPS_RX_PIN)
RX           GPIO 17 (GPS_TX_PIN)
```

### Status LEDs

```text
Function           ESP32 Pin
--------           ---------
GPS Fix Status     GPIO 19
Data Activity      GPIO 21
Error Indicator    GPIO 4
General Status     GPIO 2
```

### Test Mode Control

```text
Control            ESP32 Pin
-------            ---------
Test Mode Switch   GPIO 22 (LOW = test mode)
```

## Test Mode

The controller includes a comprehensive test mode that:

- Generates 6 different NMEA sentence types
- Simulates realistic GPS coordinates
- Cycles through sentences every 2 seconds
- Provides fake GPS fix for testing
- Enables development without actual GPS hardware

### Test NMEA Sentences

1. **$GPGGA** - Global Positioning System Fix Data
2. **$GPGSA** - GPS DOP and active satellites
3. **$GPGSV** - GPS Satellites in view
4. **$GPRMC** - Recommended minimum specific GPS/Transit data
5. **$GPVTG** - Track made good and ground speed
6. **$GPGLL** - Geographic position, latitude/longitude

## Data Mapping

GPS data is mapped to Melvin control values (0-32768 range):

| GPS Parameter | Range | Control Mapping |
|---------------|-------|-----------------|
| Latitude | -90° to +90° | 0 to 32768 |
| Longitude | -180° to +180° | 0 to 32768 |
| Altitude | 0 to 8000m | 0 to 32768 |
| Speed | 0 to 200 km/h | 0 to 32768 |
| Course | 0° to 360° | 0 to 32768 |
| Satellites | 0 to 20 | 0 to 32768 |
| GPS Fix | No/Yes | 0/32768 |

## Configuration

Key configuration options in `config.h`:

```cpp
// Device identification
#define DEVICE_TYPE 2                    // sensor_input
#define DEVICE_NAME "GPS Interface Controller" 
#define DEVICE_ID "GPS_001"

// GPS hardware settings
#define GPS_RX_PIN 16                    // ESP32 RX pin
#define GPS_TX_PIN 17                    // ESP32 TX pin  
#define GPS_BAUD_RATE 9600              // GPS module baud rate

// Timing settings
#define GPS_TIMEOUT 5000                 // GPS timeout (ms)
#define TEST_DATA_INTERVAL 2000          // Test data interval (ms)

// Control addressing
#define CONTROL_BASE_ADDRESS 3000        // Base address for GPS controls
```

## Status Indicators

### LED Patterns

- **GPS Fix LED**: Solid when GPS has valid fix
- **Data Activity LED**: Blinks when receiving GPS data
- **Error LED**: Flashes on timeout or errors
- **Status LED**: Network status (connecting/registered/error)

### Serial Debug Output

The controller provides detailed debug information:

- GPS controller initialization status
- Test mode on/off indication  
- GPS data reception confirmations
- Network registration status
- Error conditions and timeouts

## Building and Deployment

```bash
# Navigate to controller directory
cd Melvin_member_controllers/Mcontrol_GPS_Interface

# Build the controller
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Integration with Master Controller

The GPS Interface Controller:

- Registers as device type "sensor_input"
- Creates 7 GPS-specific controls automatically
- Sends position updates every 100ms
- Maintains heartbeat with master controller
- Handles network reconnection automatically

## Troubleshooting

### No GPS Fix

- Check GPS module wiring (TX/RX connections)
- Verify GPS module has clear view of sky
- Confirm GPS module is outputting NMEA sentences
- Use test mode to verify controller functionality

### No Data Activity  

- Check serial communication settings (baud rate)
- Verify GPIO pin assignments match hardware
- Monitor serial output for error messages
- Check GPS module power supply (3.3V)

### Build Issues

- Ensure TinyGPS++ library is installed
- Verify ESP32 platform is properly configured
- Check that all include files are present
- Review platformio.ini for correct dependencies

## Development Notes

This controller serves as a reference implementation for:

- UART communication with external devices
- Real-time data processing and conversion  
- Test mode implementation for offline development
- Status monitoring and error handling patterns
- Integration with the Melvin master controller architecture
