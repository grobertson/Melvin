/**
 * Melvin Version Information
 * 
 * Centralized version tracking for firmware releases
 */

#ifndef MELVIN_VERSION_H
#define MELVIN_VERSION_H

// Version information
#define MELVIN_VERSION_MAJOR 0
#define MELVIN_VERSION_MINOR 2
#define MELVIN_VERSION_PATCH 0
#define MELVIN_VERSION_BUILD "20251026"

// Version string
#define MELVIN_VERSION_STRING "v0.2.0"
#define MELVIN_VERSION_FULL "Melvin v0.2.0 (Build 20251026)"

// Build information
#define MELVIN_BUILD_DATE __DATE__
#define MELVIN_BUILD_TIME __TIME__

// Feature flags for this version
#define MELVIN_FEATURE_MQTT_BROKER 1
#define MELVIN_FEATURE_REST_API 1
#define MELVIN_FEATURE_FLASH_PATTERNS 1
#define MELVIN_FEATURE_DEVICE_MANAGEMENT 1
#define MELVIN_FEATURE_CONTROL_GROUPS 1

// Compatibility version - increment when breaking changes occur
#define MELVIN_PROTOCOL_VERSION 1

namespace MelvinVersion {
    struct VersionInfo {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
        const char* build;
        const char* version_string;
        const char* build_date;
        const char* build_time;
        uint8_t protocol_version;
    };
    
    inline VersionInfo get_version() {
        return {
            MELVIN_VERSION_MAJOR,
            MELVIN_VERSION_MINOR,
            MELVIN_VERSION_PATCH,
            MELVIN_VERSION_BUILD,
            MELVIN_VERSION_STRING,
            MELVIN_BUILD_DATE,
            MELVIN_BUILD_TIME,
            MELVIN_PROTOCOL_VERSION
        };
    }
    
    inline const char* get_version_string() {
        return MELVIN_VERSION_FULL;
    }
    
    inline bool is_compatible(uint8_t protocol_version) {
        return protocol_version == MELVIN_PROTOCOL_VERSION;
    }
}

#endif // MELVIN_VERSION_H
