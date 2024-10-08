import board
from adafruit_bus_device.i2c_device import I2CDevice
import struct


class Motor_Controller:

    def __init__(self, slave_address: int):
        # I2C bus and slave address
        self.i2c = board.I2C()
        self.slave_address = slave_address

    # Function to send linear and angular velocity to the slave
    def send_data(self, linear_velocity, angular_velocity):
        # Create I2C device
        with I2CDevice(self.i2c, self.slave_address) as device:
            try:
                # Combine bytes into a bytearray
                data_to_send = Motor_Controller.send_float(linear_velocity) + Motor_Controller.send_float(angular_velocity)
                # Write data to slave
                device.write(data_to_send)
                print(f"Sent linear velocity: {linear_velocity} and angular velocity: {angular_velocity}")
            except Exception as e:
                print(f"Error sending data: {e}")

    # Function to send a float to the slave
    @staticmethod
    def send_float(value):
        # Pack the float into a bytearray using struct
        packed_data = struct.pack('f', value)
        return bytearray(packed_data)