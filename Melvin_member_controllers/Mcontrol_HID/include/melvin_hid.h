#ifndef MCONTROL_HID_H
#define MCONTROL_HID_H

#include <Arduino.h>
#include <vector>
#include <map>

// Include config FIRST to get enum definitions
#include "config.h"

// Then include the base member controller library
#include <MelvinMemberController.h>

// ============================================================================
// INPUT TYPE ENUMERATIONS
// ============================================================================

enum InputType {
    INPUT_TOGGLE_SWITCH,
    INPUT_MOMENTARY_BUTTON,
    INPUT_POTENTIOMETER,
    INPUT_ROTARY_ENCODER,
    INPUT_JOYSTICK_AXIS,
    INPUT_JOYSTICK_BUTTON
};

// ============================================================================
// ROTARY ENCODER STATE
// ============================================================================

struct EncoderState {
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    gpio_num_t pin_button;
    int position;
    int last_encoded;
    unsigned long last_change_time;
    bool button_pressed;
    bool button_changed;
};

// ============================================================================
// JOYSTICK STATE
// ============================================================================

struct JoystickState {
    gpio_num_t pin_x;
    gpio_num_t pin_y;
    gpio_num_t pin_button;
    uint16_t x_value;
    uint16_t y_value;
    uint16_t x_raw;
    uint16_t y_raw;
    bool button_pressed;
    bool button_changed;
    unsigned long last_update;
};

// ============================================================================
// ANALOG INPUT STATE
// ============================================================================

struct AnalogInput {
    gpio_num_t pin;
    uint16_t current_value;
    uint16_t last_published_value;
    uint32_t raw_accumulator;
    uint8_t sample_count;
    unsigned long last_update;
};

// ============================================================================
// DIGITAL INPUT STATE
// ============================================================================

struct DigitalInput {
    gpio_num_t pin;
    bool current_state;
    bool last_state;
    unsigned long last_change_time;
    bool changed;
};

// ============================================================================
// HID CONTROLLER CLASS
// ============================================================================

class HIDController : public MelvinMemberController {
public:
    HIDController();
    
    // Setup and initialization
    void setup();  // Not override - base class doesn't have virtual setup()
    void update(); // Not override - base class doesn't have virtual update()
    
protected:
    // MQTT event handlers
    void on_mqtt_connected() override;
    void on_mqtt_control_command(const MQTTControlCommand& command) override;
    
private:
    // ========================================================================
    // INITIALIZATION METHODS
    // ========================================================================
    void init_gpio_pins();
    void init_controls();
    void init_digital_inputs();
    void init_analog_inputs();
    void init_encoders();
    void init_joysticks();
    
    // ========================================================================
    // INPUT READING METHODS
    // ========================================================================
    void scan_all_inputs();
    void scan_digital_inputs();
    void scan_analog_inputs();
    void scan_encoders();
    void scan_joysticks();
    
    // ========================================================================
    // DIGITAL INPUT PROCESSING
    // ========================================================================
    bool read_digital_with_debounce(gpio_num_t pin, unsigned long& last_time);
    void process_toggle_switch(uint16_t control_id, DigitalInput& input);
    void process_momentary_button(uint16_t control_id, DigitalInput& input);
    
    // ========================================================================
    // ANALOG INPUT PROCESSING
    // ========================================================================
    uint16_t read_analog_averaged(gpio_num_t pin);
    void process_potentiometer(uint16_t control_id, AnalogInput& input);
    uint16_t apply_analog_deadband(uint16_t current, uint16_t previous);
    
    // ========================================================================
    // ROTARY ENCODER PROCESSING
    // ========================================================================
    void process_encoder(uint16_t control_id, EncoderState& encoder);
    int read_encoder_change(EncoderState& encoder);
    void update_encoder_position(EncoderState& encoder, int change);
    
    // ========================================================================
    // JOYSTICK PROCESSING
    // ========================================================================
    void process_joystick(uint16_t control_id, JoystickState& joystick);
    uint16_t apply_joystick_deadzone(uint16_t raw_value);
    uint16_t calibrate_joystick_axis(uint16_t raw_value);
    
    // ========================================================================
    // STATUS LED CONTROL
    // ========================================================================
    void update_status_led();
    void blink_status_led(uint8_t blinks, uint16_t duration_ms);
    
    // ========================================================================
    // UTILITY METHODS
    // ========================================================================
    void publish_changed_controls();
    String input_type_to_string(InputType type);
    String get_control_name_by_id(uint16_t control_id);
    
    // ========================================================================
    // PURE VIRTUAL METHOD IMPLEMENTATIONS
    // ========================================================================
    void setup_hardware() override;
    void read_inputs() override;
    void write_outputs() override;
    void handle_error(const String& error) override;
    
    // ========================================================================
    // STATE STORAGE
    // ========================================================================
    
    // Digital inputs
    std::map<uint16_t, DigitalInput> toggle_switches;
    std::map<uint16_t, DigitalInput> momentary_buttons;
    
    // Analog inputs
    std::map<uint16_t, AnalogInput> potentiometers;
    
    // Rotary encoders
    std::map<uint16_t, EncoderState> encoders;
    
    // Joysticks
    std::map<uint16_t, JoystickState> joysticks;
    
    // Timing
    unsigned long last_control_scan;
    unsigned long last_status_update;
    unsigned long last_publish_batch;
    
    // Status LED
    uint8_t status_led_state;
    unsigned long status_led_last_toggle;
};

#endif // MCONTROL_HID_H
