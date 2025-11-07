/**
 * Melvin Error Codes and Logging System
 * 
 * Standardized error codes and logging utilities for the Melvin system.
 */

#ifndef MELVIN_ERRORS_H
#define MELVIN_ERRORS_H

#include <Arduino.h>

/**
 * Log Levels - Standard severity levels for system messages
 */
enum LogLevel {
    LOG_DEBUG = 0,      // Detailed debug information
    LOG_INFO,           // General informational messages
    LOG_WARN,           // Warning messages - potential issues
    LOG_ERROR,          // Error messages - operation failed
    LOG_CRITICAL        // Critical errors - system stability affected
};

/**
 * Error Codes - Standardized error codes for the Melvin system
 */
enum ErrorCode {
    // Success
    ERR_SUCCESS = 0,
    
    // Network Errors (100-199)
    ERR_NETWORK_WIFI_CONNECT_FAILED = 100,
    ERR_NETWORK_WIFI_DISCONNECT = 101,
    ERR_NETWORK_AP_START_FAILED = 102,
    ERR_NETWORK_TIMEOUT = 103,
    ERR_NETWORK_DNS_FAILED = 104,
    
    // HTTP/REST API Errors (200-299)
    ERR_HTTP_REQUEST_FAILED = 200,
    ERR_HTTP_INVALID_RESPONSE = 201,
    ERR_HTTP_TIMEOUT = 202,
    ERR_HTTP_UNAUTHORIZED = 203,
    ERR_HTTP_NOT_FOUND = 204,
    ERR_HTTP_SERVER_ERROR = 205,
    
    // MQTT Errors (300-399)
    ERR_MQTT_CONNECT_FAILED = 300,
    ERR_MQTT_DISCONNECT = 301,
    ERR_MQTT_PUBLISH_FAILED = 302,
    ERR_MQTT_SUBSCRIBE_FAILED = 303,
    ERR_MQTT_BROKER_START_FAILED = 304,
    ERR_MQTT_INVALID_TOPIC = 305,
    
    // Device Management Errors (400-499)
    ERR_DEVICE_REGISTRATION_FAILED = 400,
    ERR_DEVICE_DUPLICATE_ID = 401,
    ERR_DEVICE_NOT_FOUND = 402,
    ERR_DEVICE_OFFLINE = 403,
    ERR_DEVICE_HEARTBEAT_TIMEOUT = 404,
    ERR_DEVICE_INVALID_TYPE = 405,
    
    // Control Errors (500-599)
    ERR_CONTROL_INVALID_ID = 500,
    ERR_CONTROL_INVALID_VALUE = 501,
    ERR_CONTROL_NOT_FOUND = 502,
    ERR_CONTROL_DUPLICATE_ID = 503,
    ERR_CONTROL_TYPE_MISMATCH = 504,
    ERR_CONTROL_OUT_OF_RANGE = 505,
    
    // Configuration Errors (600-699)
    ERR_CONFIG_INVALID_PARAMETER = 600,
    ERR_CONFIG_FILE_NOT_FOUND = 601,
    ERR_CONFIG_PARSE_FAILED = 602,
    ERR_CONFIG_SAVE_FAILED = 603,
    
    // Hardware Errors (700-799)
    ERR_HARDWARE_GPIO_FAILED = 700,
    ERR_HARDWARE_ADC_FAILED = 701,
    ERR_HARDWARE_I2C_FAILED = 702,
    ERR_HARDWARE_SPI_FAILED = 703,
    ERR_HARDWARE_SENSOR_FAILED = 704,
    
    // Memory Errors (800-899)
    ERR_MEMORY_ALLOCATION_FAILED = 800,
    ERR_MEMORY_OUT_OF_HEAP = 801,
    ERR_MEMORY_FRAGMENTATION = 802,
    
    // System Errors (900-999)
    ERR_SYSTEM_INITIALIZATION_FAILED = 900,
    ERR_SYSTEM_WATCHDOG_TIMEOUT = 901,
    ERR_SYSTEM_PANIC = 902,
    ERR_SYSTEM_UNKNOWN = 999
};

/**
 * Logger Class - Centralized logging with severity levels
 */
class MelvinLogger {
private:
    static LogLevel current_level;
    static bool enable_timestamps;
    static bool enable_colors;
    static const char* component_name;
    
    static const char* level_to_string(LogLevel level) {
        switch (level) {
            case LOG_DEBUG:    return "DEBUG";
            case LOG_INFO:     return "INFO";
            case LOG_WARN:     return "WARN";
            case LOG_ERROR:    return "ERROR";
            case LOG_CRITICAL: return "CRITICAL";
            default:           return "UNKNOWN";
        }
    }
    
    static const char* level_to_color(LogLevel level) {
        if (!enable_colors) return "";
        
        switch (level) {
            case LOG_DEBUG:    return "\033[36m";  // Cyan
            case LOG_INFO:     return "\033[32m";  // Green
            case LOG_WARN:     return "\033[33m";  // Yellow
            case LOG_ERROR:    return "\033[31m";  // Red
            case LOG_CRITICAL: return "\033[35m";  // Magenta
            default:           return "\033[0m";   // Reset
        }
    }
    
    static void print_prefix(LogLevel level) {
        Serial.print(level_to_color(level));
        Serial.print("[");
        Serial.print(level_to_string(level));
        Serial.print("]");
        
        if (enable_timestamps) {
            Serial.print(" [");
            Serial.print(millis());
            Serial.print("ms]");
        }
        
        if (component_name) {
            Serial.print(" [");
            Serial.print(component_name);
            Serial.print("]");
        }
        
        Serial.print(" ");
        
        if (enable_colors) {
            Serial.print("\033[0m");  // Reset color
        }
    }
    
public:
    static void init(const char* component = nullptr, LogLevel level = LOG_INFO) {
        component_name = component;
        current_level = level;
        enable_timestamps = true;
        enable_colors = false;  // Disable colors by default for serial compatibility
    }
    
    static void set_level(LogLevel level) {
        current_level = level;
    }
    
    static void enable_timestamp(bool enable) {
        enable_timestamps = enable;
    }
    
    static void enable_color(bool enable) {
        enable_colors = enable;
    }
    
    static void log(LogLevel level, const String& message) {
        if (level < current_level) return;
        
        print_prefix(level);
        Serial.println(message);
    }
    
    static void logf(LogLevel level, const char* format, ...) {
        if (level < current_level) return;
        
        print_prefix(level);
        
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        Serial.println(buffer);
    }
    
    static void debug(const String& message) { log(LOG_DEBUG, message); }
    static void info(const String& message) { log(LOG_INFO, message); }
    static void warn(const String& message) { log(LOG_WARN, message); }
    static void error(const String& message) { log(LOG_ERROR, message); }
    static void critical(const String& message) { log(LOG_CRITICAL, message); }
    
    static void debugf(const char* format, ...) {
        if (LOG_DEBUG < current_level) return;
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LOG_DEBUG, buffer);
    }
    
    static void infof(const char* format, ...) {
        if (LOG_INFO < current_level) return;
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LOG_INFO, buffer);
    }
    
    static void warnf(const char* format, ...) {
        if (LOG_WARN < current_level) return;
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LOG_WARN, buffer);
    }
    
    static void errorf(const char* format, ...) {
        if (LOG_ERROR < current_level) return;
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LOG_ERROR, buffer);
    }
    
    static void criticalf(const char* format, ...) {
        if (LOG_CRITICAL < current_level) return;
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LOG_CRITICAL, buffer);
    }
    
    static void log_error_code(ErrorCode code, const String& context = "") {
        String message = "Error " + String(code);
        if (context.length() > 0) {
            message += ": " + context;
        }
        error(message);
    }
};

// Static member initialization
LogLevel MelvinLogger::current_level = LOG_INFO;
bool MelvinLogger::enable_timestamps = true;
bool MelvinLogger::enable_colors = false;
const char* MelvinLogger::component_name = nullptr;

// Convenience macros
#define LOG_DEBUG(msg) MelvinLogger::debug(msg)
#define LOG_INFO(msg) MelvinLogger::info(msg)
#define LOG_WARN(msg) MelvinLogger::warn(msg)
#define LOG_ERROR(msg) MelvinLogger::error(msg)
#define LOG_CRITICAL(msg) MelvinLogger::critical(msg)

#define LOG_DEBUGF(...) MelvinLogger::debugf(__VA_ARGS__)
#define LOG_INFOF(...) MelvinLogger::infof(__VA_ARGS__)
#define LOG_WARNF(...) MelvinLogger::warnf(__VA_ARGS__)
#define LOG_ERRORF(...) MelvinLogger::errorf(__VA_ARGS__)
#define LOG_CRITICALF(...) MelvinLogger::criticalf(__VA_ARGS__)

#define LOG_ERROR_CODE(code, context) MelvinLogger::log_error_code(code, context)

#endif // MELVIN_ERRORS_H
