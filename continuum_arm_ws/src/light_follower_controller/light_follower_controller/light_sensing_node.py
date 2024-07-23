#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import board
import busio
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
import adafruit_tca9548a

class LightSensingNode(Node):
    def __init__(self):
        super().__init__('light_sensing_node')
        
        # Create I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        
        
        # Create the TCA9548A object and give it the I2C bus
        self.tca = adafruit_tca9548a.TCA9548A(i2c)
        
        # Create the APDS9960 sensor objects
        self.light_sensors = []
        multiplexer_ports = [0, 1, 2, 3]
        for port in multiplexer_ports:
            light_sensor = APDS9960(self.tca[port])
            light_sensor.enable_proximity = False
            light_sensor.enable_color = True
            light_sensor.enable_gesture = False
            self.light_sensors.append(light_sensor)
        
        # Create a publisher
        self.publisher_ = self.create_publisher(Int32MultiArray, 'ambient_luminosity', 10)
        
        # Create a timer to frequently check the sensor data
        self.timer = self.create_timer(0.01, self.timer_callback)  # Check every 10ms

        # Luminsoity Datay to publish
        self.luminosity_data = Int32MultiArray()
        self.luminosity_data.data = [0] * len(self.light_sensors)  # Initialize with zeros


    def timer_callback(self):
        for i in range(len(self.light_sensors)):
             if self.light_sensors[i].color_data_ready:
                r, g, b, c = self.light_sensors[i].color_data
                self.luminosity_data.data[i] = c

        self.publisher_.publish(self.luminosity_data)

        self.get_logger().info(f'Published luminosity data: {self.luminosity_data.data}')

        x_diff = self.luminosity_data.data[3] - self.luminosity_data.data[1]
        y_diff = self.luminosity_data.data[2] - self.luminosity_data.data[0]

        self.get_logger().info(f'X Diff: {x_diff}')
        self.get_logger().info(f'Y Diff: {y_diff}')



def main(args=None):
    rclpy.init(args=args)
    light_sensing_node = LightSensingNode()
    rclpy.spin(light_sensing_node)
    light_sensing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
