from adafruit_bus_device.i2c_device import I2CDevice
import board
import argparse
import time

# I2C bus and slave address
i2c = board.I2C()
SLAVE_ADDRESS = 4

# Function to send a byte and an integer to the slave
def send_data(motor, setpoint):
    # Create I2C device
    with I2CDevice(i2c, SLAVE_ADDRESS) as device:
        try:
            # Combine byte and integer into a bytearray
            direction = 1 if setpoint >= 0 else 0  # 1 for positive, 0 for negative
            magnitude = abs(setpoint) 
            data_to_send = bytearray([motor, direction, *magnitude.to_bytes(2, byteorder='little')])
            # Write data to slave
            device.write(data_to_send)
            print(f"Sent data for motor {motor} with setpoint {setpoint}")
        except Exception as e:
            print(f"Error sending data: {e}")

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Send setpoints to I2C slave.')
    # Add arguments for setpoints (allowing negative values)
    parser.add_argument('setpoint_motor1', type=int, help='Setpoint for motor 1')
    parser.add_argument('setpoint_motor2', type=int, help='Setpoint for motor 2')

    # Parse the arguments
    args = parser.parse_args()

    # Send setpoint for motor 1
    send_data(1, args.setpoint_motor1)

    # Send setpoint for motor 2
    send_data(2, args.setpoint_motor2)

    # Wait for a short time
    time.sleep(1)