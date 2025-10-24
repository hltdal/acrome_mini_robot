# eğer motorları ilk seferde döndüremezsen bu kodu çalıştırın motorları döndürün ardından get_rpm ile ikisini bir çalıştırırsın
# id yi 0 ve 1 yaparak 2 kere çalıştır. 2 motor da tune olsun

from smd.red import *  # Import the SMD Red motor control library
from serial.tools.list_ports import comports  # Import serial communication tools
from platform import system  # Import system information module
import math
import os

# Function to detect and return the correct USB port for communication
def USB_Port():
    ports = list(comports())  # Get a list of available serial ports
    usb_names = {
        "Windows": ["USB Serial Port"],  # Windows-specific port names
        "Linux": ["/dev/ttyUSB"],  # Linux-specific port names
        "Darwin": [  # macOS-specific port names
            "/dev/tty.usbserial",
            "/dev/tty.usbmodem",
            "/dev/tty.SLAB_USBtoUART",
            "/dev/tty.wchusbserial",
            "/dev/cu.usbserial",
            "/dev/cu.usbmodem",
            "/dev/cu.SLAB_USBtoUART",
            "/dev/cu.wchusbserial",
        ]
    }
    
    os_name = system()  # Detect the operating system
    if ports:  # If ports are available
        for port, desc, hwid in sorted(ports):  # Iterate through detected ports
            # Check if the port name or description matches the expected USB names
            if any(name in port or name in desc for name in usb_names.get(os_name, [])):
                return port  # Return the detected port
        
        # If no matching port was found, print the available ports
        print("Current ports:")
        for port, desc, hwid in ports:
            print(f"Port: {port}, Description: {desc}, Hardware ID: {hwid}")
    else:
        print("No port found")  # Print message if no ports are detected
    return None  # Return None if no suitable port is found

# Get the detected USB port
port = USB_Port()

# Initialize the motor controller using the detected port
m = Master(port)

# Define motor ID
ID = 0

# Attach the motor to the master controller
m.attach(Red(ID))

# Configure motor settings
m.set_shaft_cpr(ID, 6533)  # Set encoder counts per revolution (CPR)
m.set_shaft_rpm(ID, 100)  # Set shaft rotation speed in RPM
m.set_operation_mode(ID, OperationMode.Velocity)  # Set motor to velocity control mode
m.set_control_parameters_velocity(ID, 30.0, 5.0, 0.0)  # Set PID control parameters (P, I, D)
m.enable_torque(ID, True)  # Enable motor torque

# Get user input for speed
speed = input("Speed: ")

# Set the velocity of the motor based on user input
m.set_velocity(ID, float(speed))

# Print the motor's current speed
print("The motor rotates at speed " + speed + ".")