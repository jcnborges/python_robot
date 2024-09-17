from adafruit_bus_device.i2c_device import I2CDevice
import board
import argparse
import time

class Motor_Controller:

    def __init__(self, slave_address: int):
        # I2C bus and slave address
        self.i2c = board.I2C()
        self.slave_address = 4

    # Function to send a byte and an integer to the slave
    def send_data(self, motor, setpoint):
        # Create I2C device
        with I2CDevice(self.i2c, self.slave_address) as device:
            try:
                # Combine byte and integer into a bytearray
                direction = 1 if setpoint >= 0 else 0  # 1 for positive, 0 for negative
                magnitude = abs(setpoint) 
                data_to_send = bytearray([motor, direction, *magnitude.to_bytes(2, byteorder='little')])
                # Write data to slave
                device.write(data_to_send)
                #print(f"Sent data for motor {motor} with setpoint {setpoint}")
            except Exception as e:
                print(f"Error sending data: {e}")
