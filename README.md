# Melvin

![version](https://img.shields.io/badge/version-0.2.0-blue)
![platform](https://img.shields.io/badge/platform-ESP32-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)


Extending a 25-year-old Volvo with a network of micro-controllers.  
(Yes, your car can get smarter — and maybe rebel gently.)

## 🚀 Project Status: Active Development

**✅ Completed Features**  
- Master Controller: ESP32-based coordinator with WiFi AP, REST API, and MQTT broker  
- MQTT Integration: Full pub/sub messaging system for real-time control events  
- Abstract Member Controller: Base class with WiFi client, REST registration, and MQTT client  
- Error Handling: Comprehensive system (100-999) with severity levels  
- Logging System: 5-level logging (DEBUG, INFO, WARN, ERROR, CRITICAL) with timestamps  
- Version Tracking: Firmware versioning with build metadata and feature flags  
- Health Monitoring: System status endpoint with memory and device statistics  
- Connection Management: Retry logic with exponential back-off  
- Control ID Allocation: Documented ID ranges to prevent conflicts  
- Documentation: Guides for member controllers & MQTT usage  

**🔧 In Progress**  
- Hardware controller implementations (HID, Relay Bank, Sensor Bank)  
- Configuration management system (web UI + SPIFFS storage)  

**🎯 Planned Features**  
- Specialized controllers (GPS, Camera, Display, OBD-II)  
- OTA firmware updates  
- Advanced MQTT (QoS 1, retained messages)  
- Web-based configuration interface  
- Data logging & analytics  

## 🏗 Architecture  

### Communication Protocols  
- REST API: Device registration, configuration & status queries  
- MQTT: Real-time control events, sensor data, inter-device messaging  

### Controller Types  
- **Master Controller**: Central coordinator — WiFi, REST, MQTT broker  
- **Member Controllers**: Specialized devices for functions like:  
  - Human Interface (HID) → switches/buttons  
  - Relay Bank → lights/accessories  
  - Sensor Bank → temperature/motion/light sensors  
  - GPS Interface, Camera, Display, OBD-II Diagnostics … and more  

## 🛠 Getting Started  

### Master Controller Setup  
1. `cd Melvin_master_controller/`  
2. Read `GETTING_STARTED.md` for configuration  
3. Build with PlatformIO: `pio run`  
4. Flash to ESP32: `pio run --target upload`  
5. Monitor output: `pio device monitor`  

### Member Controller Setup  
1. `cd Melvin_member_controllers/Mcontrol_Abstract/`  
2. Read `README.md` & `CUSTOMIZATION_GUIDE.md`  
3. Copy template to create a custom controller  
4. Set device ID and control IDs  
5. Build and flash to ESP32  

## 📁 Project Structure  
```text

Melvin/
├── .gitignore
├── README.md
├── LICENSE
├── Melvin_controller_scheme.md
├── CONTROL_ID_ALLOCATION.md
├── Melvin_master_controller/
│   ├── include/
│   ├── src/
│   ├── platformio.ini
│   ├── README.md
│   ├── GETTING_STARTED.md
│   └── ...
└── Melvin_member_controllers/
├── README.md
├── Mcontrol_Abstract/
├── Mcontrol_HID/
├── Mcontrol_Relay_Bank/
└── ...
```

## 🔩 Hardware Requirements  
- ESP32 or ESP32-S3 development board  
- 12 V automotive power supply (with 3.3 V regulation)  
- Additional electronics: sensors, switches, relays per use-case  
- USB cable for programming/debugging  

## ⚠️ Safety Notice  
This is automotive electronics territory:  
- Use proper fuses and circuit protection  
- Test thoroughly before integrating  
- Never compromise critical vehicle systems  
- If unsure, consult a professional  

## 🤝 Contributing  
This is a personal project, but feel free to reuse ideas and code. All code is provided as-is under the MIT license. PRs welcome though, I would love to see what you're working on, or how you're using Melvins distributed modules to reduce the need for long wire runs.

## See Also  
- `Melvin_controller_scheme.md`: Architecture and API overview  
- `CONTROL_ID_ALLOCATION.md`: Control ID assignment guide  
- Master & Member controller docs located in their respective directories  

---


