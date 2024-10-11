import evdev

# Get a list of available input devices
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

# Find the joystick device by name
joystick = None
for device in devices:
    print(device)
    if 'Microsoft X-Box 360 pad' in device.name:
        joystick = device
        break

if joystick is None:
    print("Joystick not found!")
    exit()

print(f"Joystick found: {joystick.name}")

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
        print(axis_code)
        print(axis_value)

# Do something with the joystick data, for example:
# - Control a game character
# - Send commands to a robot
# - Trigger events in a software application