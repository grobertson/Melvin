#!/usr/bin/env python3
"""
Example Member Controller - Demonstrates how to implement a member device
that connects to the Melvin Master Controller

This simulates a Human Interface Controller with various input types:
- Toggle switches
- Potentiometer sliders  
- Rotary encoders
- Joystick controls

Usage: python member_controller_example.py [master_ip]
"""

import requests
import json
import time
import random
import sys
import threading
from datetime import datetime

class MelvinMemberController:
    def __init__(self, master_ip="192.168.4.1", device_id="HIC_DEMO", device_name="Demo Human Interface"):
        self.master_ip = master_ip
        self.base_url = f"http://{master_ip}"
        self.device_id = device_id
        self.device_name = device_name
        self.device_type = 1  # DEVICE_HUMAN_INTERFACE
        self.controls = {}  # Store created control IDs
        self.running = False
        
    def register_device(self):
        """Register this device with the master controller"""
        url = f"{self.base_url}/register/device/"
        data = {
            "device_id": self.device_id,
            "device_type": self.device_type,
            "name": self.device_name
        }
        
        try:
            response = requests.post(url, json=data, timeout=5)
            if response.status_code == 200:
                print(f"✅ Device registered: {self.device_id}")
                return True
            else:
                print(f"❌ Registration failed: {response.status_code} - {response.text}")
                return False
        except requests.RequestException as e:
            print(f"❌ Connection error during registration: {e}")
            return False
    
    def create_control(self, base_address, control_number, control_type, name):
        """Create a control on the master controller"""
        url = f"{self.base_url}/controls/"
        data = {
            "base_address": base_address,
            "control_number": control_number,
            "type": control_type,
            "name": name
        }
        
        try:
            response = requests.post(url, json=data, timeout=5)
            if response.status_code == 200:
                result = response.json()
                control_id = result.get("control_id")
                self.controls[name] = control_id
                print(f"✅ Control created: {name} (ID: {control_id})")
                return control_id
            else:
                print(f"❌ Control creation failed: {response.status_code}")
                return None
        except requests.RequestException as e:
            print(f"❌ Connection error creating control: {e}")
            return None
    
    def update_control(self, control_id, value, value_y=None, value_z=None, flash_pattern=None):
        """Update a control's value"""
        url = f"{self.base_url}/controls/{control_id}"
        data = {"value": value}
        
        if value_y is not None:
            data["value_y"] = value_y
        if value_z is not None:
            data["value_z"] = value_z
        if flash_pattern is not None:
            data["flash_pattern"] = flash_pattern
        
        try:
            response = requests.put(url, json=data, timeout=5)
            return response.status_code == 200
        except requests.RequestException:
            return False
    
    def setup_controls(self):
        """Create all the controls this device will manage"""
        print("\\n🎛️  Setting up controls...")
        
        # Toggle switches (Boolean controls)
        self.create_control(100, 1, 0, "Emergency Lights")
        self.create_control(100, 2, 0, "Strobe Lights") 
        self.create_control(100, 3, 0, "Work Lights")
        self.create_control(100, 4, 0, "Auxiliary Power")
        
        # Continuous controls (Potentiometers/Sliders)
        self.create_control(200, 1, 1, "Light Bar Brightness")
        self.create_control(200, 2, 1, "Volume Control")
        self.create_control(200, 3, 1, "Display Brightness")
        
        # Rotary encoder
        self.create_control(300, 1, 2, "Frequency Selector")
        
        # 2-axis joystick
        self.create_control(400, 1, 3, "Spotlight Direction")
        
        # RGB control group
        self.create_control(500, 1, 4, "Cabin RGB Lighting")
        
        print(f"✅ Created {len(self.controls)} controls")
    
    def simulate_inputs(self):
        """Simulate realistic input changes"""
        while self.running:
            try:
                # Simulate toggle switch changes (less frequent)
                if random.random() < 0.1:  # 10% chance
                    switch_controls = ["Emergency Lights", "Strobe Lights", "Work Lights", "Auxiliary Power"]
                    control_name = random.choice(switch_controls)
                    if control_name in self.controls:
                        # Toggle the current state
                        new_value = random.choice([0, 1])
                        control_id = self.controls[control_name]
                        if self.update_control(control_id, new_value):
                            state = "ON" if new_value else "OFF"
                            print(f"🔘 {control_name}: {state}")
                
                # Simulate continuous control changes (potentiometers)
                if random.random() < 0.3:  # 30% chance
                    continuous_controls = ["Light Bar Brightness", "Volume Control", "Display Brightness"]
                    control_name = random.choice(continuous_controls)
                    if control_name in self.controls:
                        # Random value between 0-32768
                        new_value = random.randint(0, 32768)
                        control_id = self.controls[control_name]
                        if self.update_control(control_id, new_value):
                            percentage = int((new_value / 32768) * 100)
                            print(f"🎚️  {control_name}: {percentage}%")
                
                # Simulate rotary encoder changes
                if random.random() < 0.2:  # 20% chance
                    if "Frequency Selector" in self.controls:
                        # Rotary encoders often change by small increments
                        current_pos = random.randint(0, 32768)
                        control_id = self.controls["Frequency Selector"]
                        if self.update_control(control_id, current_pos):
                            print(f"🔄 Frequency Selector: Position {current_pos}")
                
                # Simulate joystick movement
                if random.random() < 0.2:  # 20% chance
                    if "Spotlight Direction" in self.controls:
                        x_pos = random.randint(0, 32768)
                        y_pos = random.randint(0, 32768)
                        control_id = self.controls["Spotlight Direction"]
                        if self.update_control(control_id, x_pos, value_y=y_pos):
                            x_pct = int((x_pos / 32768) * 100)
                            y_pct = int((y_pos / 32768) * 100)
                            print(f"🕹️  Spotlight Direction: X={x_pct}%, Y={y_pct}%")
                
                # Simulate RGB color changes
                if random.random() < 0.15:  # 15% chance
                    if "Cabin RGB Lighting" in self.controls:
                        r = random.randint(0, 32768)
                        g = random.randint(0, 32768) 
                        b = random.randint(0, 32768)
                        control_id = self.controls["Cabin RGB Lighting"]
                        if self.update_control(control_id, r, value_y=g, value_z=b):
                            r_pct = int((r / 32768) * 100)
                            g_pct = int((g / 32768) * 100)
                            b_pct = int((b / 32768) * 100)
                            print(f"🌈 RGB Lighting: R={r_pct}%, G={g_pct}%, B={b_pct}%")
                
                # Simulate emergency flash patterns
                if random.random() < 0.05:  # 5% chance  
                    if "Emergency Lights" in self.controls:
                        flash_pattern = random.choice([0, 1, 2])  # None, Wig-wag, Strobe
                        control_id = self.controls["Emergency Lights"]
                        patterns = ["OFF", "WIG-WAG", "STROBE"]
                        if self.update_control(control_id, 1, flash_pattern=flash_pattern):
                            print(f"🚨 Emergency Lights Flash Pattern: {patterns[flash_pattern]}")
                
                time.sleep(2)  # Update every 2 seconds
                
            except Exception as e:
                print(f"❌ Error in simulation: {e}")
                time.sleep(5)
    
    def heartbeat(self):
        """Send periodic heartbeat to maintain device registration"""
        while self.running:
            try:
                # Re-register to maintain heartbeat
                self.register_device()
                time.sleep(25)  # Send heartbeat every 25 seconds (timeout is 30)
            except Exception as e:
                print(f"❌ Heartbeat error: {e}")
                time.sleep(10)
    
    def start(self):
        """Start the member controller simulation"""
        print(f"🚗 Starting Melvin Member Controller: {self.device_name}")
        print(f"📡 Connecting to master at {self.master_ip}")
        print("=" * 60)
        
        # Register with master
        if not self.register_device():
            print("❌ Failed to register device. Exiting.")
            return
        
        # Set up controls
        self.setup_controls()
        
        # Start background threads
        self.running = True
        
        # Start heartbeat thread
        heartbeat_thread = threading.Thread(target=self.heartbeat, daemon=True)
        heartbeat_thread.start()
        
        # Start input simulation thread
        simulation_thread = threading.Thread(target=self.simulate_inputs, daemon=True)
        simulation_thread.start()
        
        print("\\n✅ Member controller started!")
        print("🎛️  Simulating control inputs...")
        print("💓 Sending heartbeat every 25 seconds")
        print("\\n📊 Control Updates:")
        print("-" * 40)
        
        try:
            # Keep main thread alive
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\\n\\n⏹️  Shutting down member controller...")
            self.running = False
            
            # Deregister device
            try:
                url = f"{self.base_url}/deregister/device/"
                data = {"device_id": self.device_id}
                requests.post(url, json=data, timeout=5)
                print(f"✅ Device {self.device_id} deregistered")
            except:
                print("⚠️  Could not deregister device")
            
            print("👋 Goodbye!")

def main():
    master_ip = "192.168.4.1"
    
    if len(sys.argv) > 1:
        master_ip = sys.argv[1]
    
    # Create and start member controller
    controller = MelvinMemberController(
        master_ip=master_ip,
        device_id=f"HIC_DEMO_{random.randint(1000, 9999)}",
        device_name="Demo Human Interface Controller"
    )
    
    controller.start()

if __name__ == "__main__":
    main()