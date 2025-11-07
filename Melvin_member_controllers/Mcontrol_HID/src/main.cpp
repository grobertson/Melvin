#include <Arduino.h>
#include "melvin_hid.h"
#include "config.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

HIDController::HIDController() 
    : MelvinMemberController(DEVICE_ID, DEVICE_TYPE, DEVICE_HUMAN_INTERFACE),
      last_control_scan(0),
      last_status_update(0),
      last_publish_batch(0),
      status_led_state(LOW),
      status_led_last_toggle(0) {
}

// ============================================================================
// SETUP AND INITIALIZATION
// ============================================================================

void HIDController::setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n\n========================================");
    Serial.println("Melvin HID Controller");
    Serial.println("Device ID: " + String(DEVICE_ID));
    Serial.println("========================================\n");
    
    // Initialize hardware (GPIO pins)
    setup_hardware();
    
    // Initialize control structures
    init_controls();
    
    // Initialize input state
    init_digital_inputs();
    init_analog_inputs();
    init_encoders();
    init_joysticks();
    
    // Call base class initialize (WiFi, registration, MQTT)
    if (!initialize()) {
        Serial.println("ERROR: Failed to initialize member controller!");
        handle_error("Initialization failed");
    }
    
    // Visual feedback
    blink_status_led(3, 200);
    
    Serial.println("HID Controller setup complete!");
}

// Implement pure virtual method from base class
void HIDController::setup_hardware() {
    init_gpio_pins();
}

// Implement pure virtual method from base class
void HIDController::read_inputs() {
    scan_all_inputs();
}

// Implement pure virtual method from base class (HID has no outputs)
void HIDController::write_outputs() {
    // HID controller doesn't have outputs, but could add LED indicators here
}

// Implement pure virtual method from base class
void HIDController::handle_error(const String& error) {
    Serial.println("ERROR: " + error);
    // Rapid blink pattern for errors
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
    }
}

void HIDController::init_gpio_pins() {
    Serial.println("Initializing GPIO pins...");
    
#if ENABLE_STATUS_LED
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
#endif

#if ENABLE_TOGGLE_SWITCHES
    pinMode(PIN_TOGGLE_SWITCH_1, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_SWITCH_2, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_SWITCH_3, INPUT_PULLUP);
    pinMode(PIN_TOGGLE_SWITCH_4, INPUT_PULLUP);
#endif

#if ENABLE_MOMENTARY_BUTTONS
    pinMode(PIN_MOMENTARY_BTN_1, INPUT_PULLUP);
    pinMode(PIN_MOMENTARY_BTN_2, INPUT_PULLUP);
    pinMode(PIN_MOMENTARY_BTN_3, INPUT_PULLUP);
    pinMode(PIN_MOMENTARY_BTN_4, INPUT_PULLUP);
#endif

#if ENABLE_POTENTIOMETERS
    pinMode(PIN_POTENTIOMETER_1, INPUT);
    pinMode(PIN_POTENTIOMETER_2, INPUT);
    pinMode(PIN_POTENTIOMETER_3, INPUT);
    analogSetAttenuation(ADC_11db);  // Full range: 0-3.3V
#endif

#if ENABLE_ROTARY_ENCODERS
    pinMode(PIN_ENCODER_1_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_1_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_1_BTN, INPUT_PULLUP);
    
    pinMode(PIN_ENCODER_2_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_2_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_2_BTN, INPUT_PULLUP);
#endif

#if ENABLE_JOYSTICKS
    pinMode(PIN_JOYSTICK_1_X, INPUT);
    pinMode(PIN_JOYSTICK_1_Y, INPUT);
    pinMode(PIN_JOYSTICK_1_BTN, INPUT_PULLUP);
    
    pinMode(PIN_JOYSTICK_2_X, INPUT);
    pinMode(PIN_JOYSTICK_2_Y, INPUT);
    pinMode(PIN_JOYSTICK_2_BTN, INPUT_PULLUP);
#endif
    
    Serial.println("GPIO pins initialized.");
}

void HIDController::init_controls() {
    Serial.println("Initializing control definitions...");
    
#if ENABLE_TOGGLE_SWITCHES
    add_control("Toggle Switch 1", CTRL_TOGGLE_SWITCH_1, CONTROL_BOOLEAN);
    add_control("Toggle Switch 2", CTRL_TOGGLE_SWITCH_2, CONTROL_BOOLEAN);
    add_control("Toggle Switch 3", CTRL_TOGGLE_SWITCH_3, CONTROL_BOOLEAN);
    add_control("Toggle Switch 4", CTRL_TOGGLE_SWITCH_4, CONTROL_BOOLEAN);
#endif

#if ENABLE_MOMENTARY_BUTTONS
    add_control("Button 1", CTRL_MOMENTARY_BTN_1, CONTROL_BOOLEAN);
    add_control("Button 2", CTRL_MOMENTARY_BTN_2, CONTROL_BOOLEAN);
    add_control("Button 3", CTRL_MOMENTARY_BTN_3, CONTROL_BOOLEAN);
    add_control("Button 4", CTRL_MOMENTARY_BTN_4, CONTROL_BOOLEAN);
#endif

#if ENABLE_POTENTIOMETERS
    add_control("Potentiometer 1", CTRL_POTENTIOMETER_1, CONTROL_CONTINUOUS);
    add_control("Potentiometer 2", CTRL_POTENTIOMETER_2, CONTROL_CONTINUOUS);
    add_control("Potentiometer 3", CTRL_POTENTIOMETER_3, CONTROL_CONTINUOUS);
#endif

#if ENABLE_ROTARY_ENCODERS
    add_control("Rotary Encoder 1", CTRL_ROTARY_ENCODER_1, CONTROL_ROTARY);
    add_control("Rotary Encoder 2", CTRL_ROTARY_ENCODER_2, CONTROL_ROTARY);
#endif

#if ENABLE_JOYSTICKS
    add_control("Joystick 1", CTRL_JOYSTICK_1, CONTROL_JOYSTICK_2AXIS);
    add_control("Joystick 2", CTRL_JOYSTICK_2, CONTROL_JOYSTICK_2AXIS);
#endif
    
    Serial.println("Control definitions created.");
}

void HIDController::init_digital_inputs() {
#if ENABLE_TOGGLE_SWITCHES
    toggle_switches[CTRL_TOGGLE_SWITCH_1] = {PIN_TOGGLE_SWITCH_1, false, false, 0, false};
    toggle_switches[CTRL_TOGGLE_SWITCH_2] = {PIN_TOGGLE_SWITCH_2, false, false, 0, false};
    toggle_switches[CTRL_TOGGLE_SWITCH_3] = {PIN_TOGGLE_SWITCH_3, false, false, 0, false};
    toggle_switches[CTRL_TOGGLE_SWITCH_4] = {PIN_TOGGLE_SWITCH_4, false, false, 0, false};
#endif

#if ENABLE_MOMENTARY_BUTTONS
    momentary_buttons[CTRL_MOMENTARY_BTN_1] = {PIN_MOMENTARY_BTN_1, false, false, 0, false};
    momentary_buttons[CTRL_MOMENTARY_BTN_2] = {PIN_MOMENTARY_BTN_2, false, false, 0, false};
    momentary_buttons[CTRL_MOMENTARY_BTN_3] = {PIN_MOMENTARY_BTN_3, false, false, 0, false};
    momentary_buttons[CTRL_MOMENTARY_BTN_4] = {PIN_MOMENTARY_BTN_4, false, false, 0, false};
#endif
}

void HIDController::init_analog_inputs() {
#if ENABLE_POTENTIOMETERS
    potentiometers[CTRL_POTENTIOMETER_1] = {PIN_POTENTIOMETER_1, 0, 0, 0, 0, 0};
    potentiometers[CTRL_POTENTIOMETER_2] = {PIN_POTENTIOMETER_2, 0, 0, 0, 0, 0};
    potentiometers[CTRL_POTENTIOMETER_3] = {PIN_POTENTIOMETER_3, 0, 0, 0, 0, 0};
#endif
}

void HIDController::init_encoders() {
#if ENABLE_ROTARY_ENCODERS
    encoders[CTRL_ROTARY_ENCODER_1] = {
        PIN_ENCODER_1_A, PIN_ENCODER_1_B, PIN_ENCODER_1_BTN,
        0, 0, 0, false, false
    };
    
    encoders[CTRL_ROTARY_ENCODER_2] = {
        PIN_ENCODER_2_A, PIN_ENCODER_2_B, PIN_ENCODER_2_BTN,
        0, 0, 0, false, false
    };
#endif
}

void HIDController::init_joysticks() {
#if ENABLE_JOYSTICKS
    joysticks[CTRL_JOYSTICK_1] = {
        PIN_JOYSTICK_1_X, PIN_JOYSTICK_1_Y, PIN_JOYSTICK_1_BTN,
        JOYSTICK_CENTER, JOYSTICK_CENTER, 0, 0,
        false, false, 0
    };
    
    joysticks[CTRL_JOYSTICK_2] = {
        PIN_JOYSTICK_2_X, PIN_JOYSTICK_2_Y, PIN_JOYSTICK_2_BTN,
        JOYSTICK_CENTER, JOYSTICK_CENTER, 0, 0,
        false, false, 0
    };
#endif
}

// ============================================================================
// MAIN UPDATE LOOP
// ============================================================================

void HIDController::update() {
    // Call base class update (handles WiFi, registration, MQTT)
    MelvinMemberController::update();
    
    // Update status LED
    update_status_led();
    
    // Scan inputs at configured interval
    unsigned long now = millis();
    if (now - last_control_scan >= CONTROL_SCAN_INTERVAL_MS) {
        last_control_scan = now;
        scan_all_inputs();
    }
    
    // Batch publish changed controls
    if (now - last_publish_batch >= MQTT_PUBLISH_BATCH_INTERVAL) {
        last_publish_batch = now;
        publish_changed_controls();
    }
    
    // Periodic status update
    if (now - last_status_update >= STATUS_UPDATE_INTERVAL_MS) {
        last_status_update = now;
        if (is_mqtt_connected()) {
            publish_device_status("online", "All systems operational");
        }
    }
}

// ============================================================================
// INPUT SCANNING
// ============================================================================

void HIDController::scan_all_inputs() {
#if ENABLE_TOGGLE_SWITCHES || ENABLE_MOMENTARY_BUTTONS
    scan_digital_inputs();
#endif

#if ENABLE_POTENTIOMETERS
    scan_analog_inputs();
#endif

#if ENABLE_ROTARY_ENCODERS
    scan_encoders();
#endif

#if ENABLE_JOYSTICKS
    scan_joysticks();
#endif
}

void HIDController::scan_digital_inputs() {
#if ENABLE_TOGGLE_SWITCHES
    for (auto& pair : toggle_switches) {
        process_toggle_switch(pair.first, pair.second);
    }
#endif

#if ENABLE_MOMENTARY_BUTTONS
    for (auto& pair : momentary_buttons) {
        process_momentary_button(pair.first, pair.second);
    }
#endif
}

void HIDController::scan_analog_inputs() {
#if ENABLE_POTENTIOMETERS
    for (auto& pair : potentiometers) {
        process_potentiometer(pair.first, pair.second);
    }
#endif
}

void HIDController::scan_encoders() {
#if ENABLE_ROTARY_ENCODERS
    for (auto& pair : encoders) {
        process_encoder(pair.first, pair.second);
    }
#endif
}

void HIDController::scan_joysticks() {
#if ENABLE_JOYSTICKS
    unsigned long now = millis();
    for (auto& pair : joysticks) {
        JoystickState& js = pair.second;
        if (now - js.last_update >= JOYSTICK_UPDATE_MS) {
            js.last_update = now;
            process_joystick(pair.first, js);
        }
    }
#endif
}

// ============================================================================
// DIGITAL INPUT PROCESSING
// ============================================================================

bool HIDController::read_digital_with_debounce(gpio_num_t pin, unsigned long& last_time) {
    bool current_state = !digitalRead(pin);  // Inverted due to INPUT_PULLUP
    unsigned long now = millis();
    
    if (now - last_time >= DEBOUNCE_DELAY_MS) {
        last_time = now;
        return current_state;
    }
    
    return false;  // Still bouncing
}

void HIDController::process_toggle_switch(uint16_t control_id, DigitalInput& input) {
    bool new_state = !digitalRead(input.pin);  // Inverted for INPUT_PULLUP
    unsigned long now = millis();
    
    if (new_state != input.last_state) {
        if (now - input.last_change_time >= DEBOUNCE_DELAY_MS) {
            input.current_state = new_state;
            input.changed = true;
            input.last_change_time = now;
            
            // Update control value by name (find control name from ID)
            String control_name = get_control_name_by_id(control_id);
            if (control_name.length() > 0) {
                set_control_value(control_name, new_state ? 1 : 0);
            }
            
#if DEBUG_PRINT_INPUTS
            Serial.printf("Toggle Switch %d: %s\n", control_id, new_state ? "ON" : "OFF");
#endif
        }
    }
    input.last_state = new_state;
}

void HIDController::process_momentary_button(uint16_t control_id, DigitalInput& input) {
    bool new_state = !digitalRead(input.pin);  // Inverted for INPUT_PULLUP
    unsigned long now = millis();
    
    if (new_state != input.last_state) {
        if (now - input.last_change_time >= DEBOUNCE_DELAY_MS) {
            input.current_state = new_state;
            input.changed = true;
            input.last_change_time = now;
            
            // Update control value by name
            String control_name = get_control_name_by_id(control_id);
            if (control_name.length() > 0) {
                set_control_value(control_name, new_state ? 1 : 0);
            }
            
#if DEBUG_PRINT_INPUTS
            Serial.printf("Button %d: %s\n", control_id, new_state ? "PRESSED" : "RELEASED");
#endif
        }
    }
    input.last_state = new_state;
}

// ============================================================================
// ANALOG INPUT PROCESSING
// ============================================================================

uint16_t HIDController::read_analog_averaged(gpio_num_t pin) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(pin);
        delayMicroseconds(10);
    }
    return sum / ADC_SAMPLES;
}

void HIDController::process_potentiometer(uint16_t control_id, AnalogInput& input) {
    unsigned long now = millis();
    
    // Read and average samples
    input.raw_accumulator += analogRead(input.pin);
    input.sample_count++;
    
    if (input.sample_count >= ADC_SAMPLES) {
        uint16_t raw_value = input.raw_accumulator / ADC_SAMPLES;
        input.raw_accumulator = 0;
        input.sample_count = 0;
        
        // Apply deadband
        uint16_t new_value = apply_analog_deadband(raw_value, input.current_value);
        
        if (abs((int)new_value - (int)input.current_value) >= ADC_DEADBAND) {
            input.current_value = new_value;
            
            // Map to 0-32768 range
            uint16_t control_value = map(new_value, 0, ADC_MAX_VALUE, 0, 32768);
            
            String control_name = get_control_name_by_id(control_id);
            if (control_name.length() > 0) {
                set_control_value(control_name, control_value);
            }
            
#if DEBUG_PRINT_INPUTS
            Serial.printf("Potentiometer %d: %d (raw: %d)\n", control_id, control_value, new_value);
#endif
        }
        
        input.last_update = now;
    }
}

uint16_t HIDController::apply_analog_deadband(uint16_t current, uint16_t previous) {
    int diff = abs((int)current - (int)previous);
    if (diff < ADC_DEADBAND) {
        return previous;
    }
    return current;
}

// ============================================================================
// ROTARY ENCODER PROCESSING
// ============================================================================

void HIDController::process_encoder(uint16_t control_id, EncoderState& encoder) {
    int change = read_encoder_change(encoder);
    
    if (change != 0) {
        update_encoder_position(encoder, change);
        
        // Update control value
        String control_name = get_control_name_by_id(control_id);
        if (control_name.length() > 0) {
            set_control_value(control_name, encoder.position);
        }
        
#if DEBUG_PRINT_ENCODER
        Serial.printf("Encoder %d: %d (change: %d)\n", control_id, encoder.position, change);
#endif
    }
    
    // Check button state
    bool button_state = !digitalRead(encoder.pin_button);
    if (button_state != encoder.button_pressed) {
        unsigned long now = millis();
        if (now - encoder.last_change_time >= DEBOUNCE_DELAY_MS) {
            encoder.button_pressed = button_state;
            encoder.button_changed = true;
            encoder.last_change_time = now;
            
            // Could publish encoder button as separate control if needed
#if DEBUG_PRINT_ENCODER
            Serial.printf("Encoder %d Button: %s\n", control_id, button_state ? "PRESSED" : "RELEASED");
#endif
        }
    }
}

int HIDController::read_encoder_change(EncoderState& encoder) {
    int MSB = digitalRead(encoder.pin_a);
    int LSB = digitalRead(encoder.pin_b);
    
    int encoded = (MSB << 1) | LSB;
    int sum = (encoder.last_encoded << 2) | encoded;
    
    int change = 0;
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        change = 1;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        change = -1;
    }
    
    encoder.last_encoded = encoded;
    return change;
}

void HIDController::update_encoder_position(EncoderState& encoder, int change) {
    encoder.position += change;
    
#if ENCODER_WRAP_AROUND
    if (encoder.position > ENCODER_MAX_VALUE) {
        encoder.position = ENCODER_MIN_VALUE;
    } else if (encoder.position < ENCODER_MIN_VALUE) {
        encoder.position = ENCODER_MAX_VALUE;
    }
#else
    encoder.position = constrain(encoder.position, ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
#endif
}

// ============================================================================
// JOYSTICK PROCESSING
// ============================================================================

void HIDController::process_joystick(uint16_t control_id, JoystickState& joystick) {
    // Read raw values
    uint16_t raw_x = read_analog_averaged(joystick.pin_x);
    uint16_t raw_y = read_analog_averaged(joystick.pin_y);
    
    // Apply deadzone and calibration
    uint16_t x_value = calibrate_joystick_axis(raw_x);
    uint16_t y_value = calibrate_joystick_axis(raw_y);
    
    // Check for significant change
    bool changed = false;
    if (abs((int)x_value - (int)joystick.x_value) >= ADC_DEADBAND ||
        abs((int)y_value - (int)joystick.y_value) >= ADC_DEADBAND) {
        joystick.x_value = x_value;
        joystick.y_value = y_value;
        joystick.x_raw = raw_x;
        joystick.y_raw = raw_y;
        changed = true;
    }
    
    // Check button state
    bool button_state = !digitalRead(joystick.pin_button);
    if (button_state != joystick.button_pressed) {
        joystick.button_pressed = button_state;
        joystick.button_changed = true;
        changed = true;
    }
    
    if (changed) {
        // Update control with 2D values (using value_y for Y axis)
        String control_name = get_control_name_by_id(control_id);
        if (control_name.length() > 0) {
            set_control_value(control_name, joystick.x_value, joystick.y_value);
        }
        
#if DEBUG_PRINT_INPUTS
        Serial.printf("Joystick %d: X=%d Y=%d Btn=%d\n", 
                     control_id, joystick.x_value, joystick.y_value, joystick.button_pressed);
#endif
    }
}

uint16_t HIDController::apply_joystick_deadzone(uint16_t raw_value) {
    int deviation = (int)raw_value - JOYSTICK_CENTER;
    
    if (abs(deviation) < JOYSTICK_DEADZONE) {
        return JOYSTICK_CENTER;
    }
    
    return raw_value;
}

uint16_t HIDController::calibrate_joystick_axis(uint16_t raw_value) {
    // Apply deadzone
    uint16_t value = apply_joystick_deadzone(raw_value);
    
    // Map to 0-32768 range
    return map(value, JOYSTICK_MIN, JOYSTICK_MAX, 0, 32768);
}

// ============================================================================
// STATUS LED CONTROL
// ============================================================================

void HIDController::update_status_led() {
#if ENABLE_STATUS_LED
    unsigned long now = millis();
    
    if (!is_connected()) {
        // Fast blink when not connected
        if (now - status_led_last_toggle >= 200) {
            status_led_state = !status_led_state;
            digitalWrite(PIN_STATUS_LED, status_led_state);
            status_led_last_toggle = now;
        }
    } else if (!is_mqtt_connected()) {
        // Slow blink when WiFi connected but MQTT not connected
        if (now - status_led_last_toggle >= 1000) {
            status_led_state = !status_led_state;
            digitalWrite(PIN_STATUS_LED, status_led_state);
            status_led_last_toggle = now;
        }
    } else {
        // Solid on when fully connected
        digitalWrite(PIN_STATUS_LED, HIGH);
    }
#endif
}

void HIDController::blink_status_led(uint8_t blinks, uint16_t duration_ms) {
#if ENABLE_STATUS_LED
    for (uint8_t i = 0; i < blinks; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(duration_ms);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(duration_ms);
    }
#endif
}

// ============================================================================
// MQTT EVENT HANDLERS
// ============================================================================

void HIDController::on_mqtt_connected() {
    MelvinMemberController::on_mqtt_connected();
    
    Serial.println("HID Controller MQTT connected");
    
    // Subscribe to any topics needed for HID (e.g., LED feedback)
    // subscribe_to_custom_topic("melvin/feedback/leds", ...);
    
    blink_status_led(2, 100);
}

void HIDController::on_mqtt_control_command(const MQTTControlCommand& command) {
    Serial.printf("HID received control command: Control %d = %d\n", 
                 command.control_id, command.value);
    
    // HID is primarily an input device, but could handle commands
    // for LED feedback, haptic feedback, etc.
}

// ============================================================================
// UTILITY METHODS
// ============================================================================

void HIDController::publish_changed_controls() {
    if (!is_mqtt_connected()) return;
    
    // Publish each changed control individually
    // Base class tracks which controls have changed
    // We could implement batch publishing here if needed
}

String HIDController::input_type_to_string(InputType type) {
    switch (type) {
        case INPUT_TOGGLE_SWITCH: return "Toggle Switch";
        case INPUT_MOMENTARY_BUTTON: return "Momentary Button";
        case INPUT_POTENTIOMETER: return "Potentiometer";
        case INPUT_ROTARY_ENCODER: return "Rotary Encoder";
        case INPUT_JOYSTICK_AXIS: return "Joystick Axis";
        case INPUT_JOYSTICK_BUTTON: return "Joystick Button";
        default: return "Unknown";
    }
}

String HIDController::get_control_name_by_id(uint16_t control_id) {
    // Map control IDs to control names
    switch (control_id) {
        case CTRL_TOGGLE_SWITCH_1: return "Toggle Switch 1";
        case CTRL_TOGGLE_SWITCH_2: return "Toggle Switch 2";
        case CTRL_TOGGLE_SWITCH_3: return "Toggle Switch 3";
        case CTRL_TOGGLE_SWITCH_4: return "Toggle Switch 4";
        case CTRL_MOMENTARY_BTN_1: return "Button 1";
        case CTRL_MOMENTARY_BTN_2: return "Button 2";
        case CTRL_MOMENTARY_BTN_3: return "Button 3";
        case CTRL_MOMENTARY_BTN_4: return "Button 4";
        case CTRL_POTENTIOMETER_1: return "Potentiometer 1";
        case CTRL_POTENTIOMETER_2: return "Potentiometer 2";
        case CTRL_POTENTIOMETER_3: return "Potentiometer 3";
        case CTRL_ROTARY_ENCODER_1: return "Rotary Encoder 1";
        case CTRL_ROTARY_ENCODER_2: return "Rotary Encoder 2";
        case CTRL_JOYSTICK_1: return "Joystick 1";
        case CTRL_JOYSTICK_2: return "Joystick 2";
        default: return "";
    }
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

HIDController controller;

void setup() {
    controller.setup();
}

void loop() {
    controller.update();
}
