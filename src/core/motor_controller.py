import board
from adafruit_bus_device.i2c_device import I2CDevice
import struct


class Motor_Controller:

    def __init__(self, slave_address: int):
        # I2C bus and slave address
        self.i2c = board.I2C()
        self.slave_address = slave_address

    # Function to send linear and angular velocity to the slave
    def set_velocity(self, linear_velocity: float, angular_velocity: float):
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

    # Function to read a ADC channel from the slave
    def read_adc_level(self) -> int:
        # Create I2C device
        with I2CDevice(self.i2c, self.slave_address) as device:
            try:
                data = bytearray(2)
                device.readinto(data)
            except Exception as e:
                print(f"Error reading data: {e}")                        
        # Combine the two bytes into a single integer value
        adc_value = (data[0] << 8) | data[1]
        return adc_value                

    # Function to send a float to the slave
    @staticmethod
    def send_float(value: float):
        # Pack the float into a bytearray using struct
        packed_data = struct.pack('f', value)
        return bytearray(packed_data)