# Notes

## To do list for Melvin (mostly mechanical, can be ignored when considering design and development)

- liftgate reverse lights swap out for white/amber
- CD Changer fake Bluetooth, have supplies
- FM antenna hole.. what size? Possible NMO mount? Bulkhead pass for roof electronics/sensors/signals
- find bulkhead passthrough?
- Chk/RR aux power wiring, ensure properly mounted
- route heavy gauge to roof rail w/quick disconnect for rack
- route heavy gauge wire to spare carrier area
  - add small yellow wire as relay trigger
  - Small fuse panel
  - Route always on power to tailgate inner
- need clear 12v hot and 12v switched near glovebox/console
- mount DMR in console
- RR mount holes in center dash or plug
- Internal aux lighting in glovebox needs LED swap
- Measure inner diameter of steelie hubs for caps
- perm/semi-perm 2m/70cm antenna, route to cockpit
- Driver door panel trim loose
- Pass door panel needs RR, dragging on window
- Add supplemental convenience lighting (goal: cover floormats and wells )
- RR refasten rear compartment trim (missing clips?)
- Pack/Repack basic toolset. Store in spare area
- Install rear camera, route wiring to cockpit
- Install forward camera, route wiring to cockpit
- (extended) Add Blindspot lookout cams to left and right side or roof rack

## Control Architecture (Explaining the controller architecture and details of some controller types)

- A Master controller is an ESP32 with wifi in AP mode. All controller nodes join this network as clients
- Member controllers are ESP32 with wifi and bluetooth 4.2
- The Human Interface Controller handles human controls such as switches,
    buttons, latching buttons, potentiometer type knobs and slide controls,
    directional joysticks (i.e. gamepad controller), rotary knobs, etc.
- The Sensor Interface controller handles the collection of sensor values to
    be sent to the master control
- abstract mapping of physical control to control base addr and #
- abstract mapping of "group" controls, containing one or more output controls
- Main controller is responsible for keeping track of and communicating with
    input/sensor controllers or output/lightswitch controllers
- for toggle switches, a Boolean can be used
- for "sweeping" type  controls, the value can be between 0-32768. 0 = off
    this would be something like a slide or knob style potentiometer dividing voltage
    to produce 0v-3.3v ADC
- A grouping of 3 continuous controls could implement RGB values for future devices
- controllers all need some manner of unique identifier
- Should not just be for lights. Potential other uses: Antenna stow/deploy,
    driven directional spotlight, PA system
- Needs a way to implement wig-wag or other flash styles- input controllers observe the same 0/+ true false
- input controller values are 16 bit

## example/partial list of api endpoints

/controls/
/controls/button/
/controls/toggle/
/controls/rotary/
/controls/resistance/

/register/
/register/device/

/join/
/join/device/

/deregister/
/deregister/device/

Example Device types:

- DEVICE_HID = 0, // Human interface devices (switches, joysticks, etc)
- DEVICE_DISPLAY,    // Display devices (LCD, OLED, etc)
- DEVICE_VEHICLE_INPUT, // Vehicle input controller (steering, pedals, etc)
- DEVICE_POLLED_SENSOR_BANK,  // Polled sensors are read on demand (themostat, barometer, etc)
- DEVICE_EVENT_SENSOR_BANK,   // Event sensors send updates when state changes (door sensor, motion detector, etc)
- DEVICE_RELAY_BANK,          // Relay control for high-current devices (lights, motors, etc)
- DEVICE_ODBII_INTERFACE,     // ODBII vehicle data interface
- DEVICE_GPS_INTERFACE,          // GPS data module
- DEVICE_APRS_INTERFACE,       // APRS radio interface
- DEVICE_CAMERA_INTERFACE,     // Camera control interface (pan/tilt, etc)
- DEVICE_SPOTLIGHT_CONTROLLER, // Spotlight control interface (pan/tilt)
- DEVICE_LOGGER                // Telemetry logger device (data logging and storage)
