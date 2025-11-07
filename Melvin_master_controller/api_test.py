#!/usr/bin/env python3
"""
Melvin Master Controller API Test Script
Demonstrates how to interact with the REST API endpoints

This script shows examples of:
- Device registration
- Control creation
- Control value updates
- Device monitoring

Requirements:
- requests library: pip install requests

Usage: 
  python api_test.py [master_controller_ip]
  
Default IP: 192.168.4.1 (ESP32 AP mode default)

Make sure your ESP32 is running the Melvin Master Controller firmware
and your computer is connected to the 'Melvin_Control_Network' WiFi.
"""

import requests
import json
import time
import sys

# Master controller IP (ESP32 AP mode default)
MASTER_IP = "192.168.4.1"
BASE_URL = f"http://{MASTER_IP}"

def register_device(device_id, device_type, name):
    """Register a new device with the master controller"""
    url = f"{BASE_URL}/register/device/"
    data = {
        "device_id": device_id,
        "device_type": device_type,  # 0=vehicle_input, 1=human_interface, etc.
        "name": name
    }
    
    try:
        response = requests.post(url, json=data, timeout=5)
        if response.status_code == 200:
            print(f"✓ Device registered: {device_id}")
            return True
        else:
            print(f"✗ Registration failed: {response.status_code}")
            return False
    except requests.RequestException as e:
        print(f"✗ Connection error: {e}")
        return False

def create_control(base_address, control_number, control_type, name):
    """Create a new control on the master controller"""
    url = f"{BASE_URL}/controls/"
    data = {
        "base_address": base_address,
        "control_number": control_number,
        "type": control_type,  # 0=boolean, 1=continuous, 2=rotary, 3=joystick, 4=rgb
        "name": name
    }
    
    try:
        response = requests.post(url, json=data, timeout=5)
        if response.status_code == 200:
            result = response.json()
            control_id = result.get("control_id")
            print(f"✓ Control created: {name} (ID: {control_id})")
            return control_id
        else:
            print(f"✗ Control creation failed: {response.status_code}")
            return None
    except requests.RequestException as e:
        print(f"✗ Connection error: {e}")
        return None

def update_control(control_id, value, value_y=None, value_z=None, flash_pattern=None):
    """Update a control's value"""
    url = f"{BASE_URL}/controls/{control_id}"
    data = {"value": value}
    
    if value_y is not None:
        data["value_y"] = value_y
    if value_z is not None:
        data["value_z"] = value_z
    if flash_pattern is not None:
        data["flash_pattern"] = flash_pattern
    
    try:
        response = requests.put(url, json=data, timeout=5)
        if response.status_code == 200:
            print(f"✓ Control {control_id} updated: {value}")
            return True
        else:
            print(f"✗ Update failed: {response.status_code}")
            return False
    except requests.RequestException as e:
        print(f"✗ Connection error: {e}")
        return False

def list_controls():
    """List all controls"""
    url = f"{BASE_URL}/controls/"
    
    try:
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            data = response.json()
            controls = data.get("controls", [])
            print(f"\\n📋 Controls ({len(controls)} total):")
            for control in controls:
                print(f"  ID: {control['id']}, Name: {control['name']}, "
                      f"Value: {control['value']}, Type: {control['type']}")
            return controls
        else:
            print(f"✗ Failed to list controls: {response.status_code}")
            return []
    except requests.RequestException as e:
        print(f"✗ Connection error: {e}")
        return []

def list_devices():
    """List all registered devices"""
    url = f"{BASE_URL}/devices/"
    
    try:
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            data = response.json()
            devices = data.get("devices", [])
            print(f"\\n🔗 Devices ({len(devices)} total):")
            for device in devices:
                status = "🟢 Online" if device['is_online'] else "🔴 Offline"
                print(f"  {device['device_id']}: {device['name']} - {status}")
                print(f"    Type: {device['device_type']}, IP: {device['ip_address']}")
            return devices
        else:
            print(f"✗ Failed to list devices: {response.status_code}")
            return []
    except requests.RequestException as e:
        print(f"✗ Connection error: {e}")
        return []

def demo_sequence():
    """Run a demonstration sequence"""
    print("🚗 Melvin Master Controller API Demo")
    print("=" * 40)
    
    # Register a human interface controller
    print("\\n1. Registering human interface controller...")
    register_device("HIC_001", 1, "Dashboard Control Panel")
    
    # Register an external light controller  
    print("\\n2. Registering external light controller...")
    register_device("ELC_001", 3, "Emergency Light Bar")
    
    # Create some controls
    print("\\n3. Creating controls...")
    
    # Emergency lights toggle
    emergency_id = create_control(100, 1, 0, "Emergency Lights")
    
    # Light bar brightness
    brightness_id = create_control(100, 2, 1, "Light Bar Brightness")
    
    # RGB underglow
    rgb_id = create_control(200, 1, 4, "Underglow RGB")
    
    # Joystick for spotlight
    joystick_id = create_control(300, 1, 3, "Spotlight Direction")
    
    # List current state
    print("\\n4. Current system state:")
    list_devices()
    list_controls()
    
    # Demonstrate control updates
    print("\\n5. Demonstrating control updates...")
    
    if emergency_id:
        print("  • Turning on emergency lights...")
        update_control(emergency_id, 1)  # Boolean ON
        time.sleep(1)
        
        print("  • Setting wig-wag flash pattern...")
        update_control(emergency_id, 1, flash_pattern=1)  # Wig-wag pattern
        time.sleep(2)
    
    if brightness_id:
        print("  • Setting brightness to 50%...")
        update_control(brightness_id, 16384)  # 50% of 32768
        time.sleep(1)
        
        print("  • Setting brightness to 100%...")
        update_control(brightness_id, 32768)  # Maximum brightness
        time.sleep(1)
    
    if rgb_id:
        print("  • Setting RGB to red...")
        update_control(rgb_id, 32768, value_y=0, value_z=0)  # Full red
        time.sleep(1)
        
        print("  • Setting RGB to blue...")
        update_control(rgb_id, 0, value_y=0, value_z=32768)  # Full blue
        time.sleep(1)
    
    if joystick_id:
        print("  • Moving spotlight to center...")
        update_control(joystick_id, 16384, value_y=16384)  # Center position
        time.sleep(1)
        
        print("  • Moving spotlight up-right...")
        update_control(joystick_id, 24576, value_y=24576)  # Up-right
        time.sleep(1)
    
    print("\\n6. Final system state:")
    list_controls()
    
    print("\\n✅ Demo completed!")
    print("The master controller is now managing multiple devices and controls.")

def main():
    global MASTER_IP, BASE_URL
    
    if len(sys.argv) > 1:
        MASTER_IP = sys.argv[1]
        BASE_URL = f"http://{MASTER_IP}"
        print(f"Using master controller IP: {MASTER_IP}")
    
    try:
        demo_sequence()
    except KeyboardInterrupt:
        print("\\n\\n⏹️  Demo interrupted by user")
    except Exception as e:
        print(f"\\n❌ Demo failed: {e}")

if __name__ == "__main__":
    main()