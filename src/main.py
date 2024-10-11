import time
import os
import evdev
from core.robo_rasp_zero_w import Robo_Rasp_Zero_W
from threading import Thread

robo = Robo_Rasp_Zero_W()
robo.iniciar()

def mostrar_status():
    while True:
        #os.system("clear")
        robo.mostrar_tensao_bateria()
        robo.mostrar_velocidade()
        robo.mostrar_sensor_ultra_distancia()
        robo.mostrar_sensor_obstaculo()
        time.sleep(0.2)

# Get a list of available input devices
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

# Find the joystick device by name
joystick = None
for device in devices:
    print(device)
    if 'Controller' in device.name:
        joystick = device
        break

if joystick is None:
    print("Joystick not found!")
    exit()

print(f"Joystick found: {joystick.name}")

Thread(target = mostrar_status, args = ()).start()

# Event loop
for event in joystick.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        # Handle button press/release
        button_code = event.code
        button_value = event.value
        print(button_code)
        print(button_value)
    elif event.type == evdev.ecodes.EV_ABS:
        # Handle axis movement
        axis_code = event.code
        axis_value = event.value
        if axis_code == 0:
            robo.set_x(axis_value)
        elif axis_code == 1:
            robo.set_y(axis_value)
        print(axis_code)
        print(axis_value)

# Do something with the joystick data, for example:
# - Control a game character
# - Send commands to a robot
# - Trigger events in a software application
