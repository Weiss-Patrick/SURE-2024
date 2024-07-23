#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int32MultiArray
from dynamixel_sdk import PortHandler, PacketHandler

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_SPEED = 38
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
ADDR_MX_TORQUE_CONTROL_ENABLE = 70
ADDR_MX_GOAL_TORQUE = 71

# Protocol version
PROTOCOL_VERSION = 1.0

# Default setting
DXL_IDs = [0, 1, 2, 3]  # Dynamixel IDs
BAUDRATE = 57600
DEVICENAME = '/dev/ttyDynamixel'  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

VELOCITY_LIMIT = 80  # Dynamixel velocity limit
POSITION_LIMIT = 4095  # Example position limit, adjust as needed

Kp = 0.1  # Proportional gain
DEAD_BAND = 40000  # Dead band for ignoring minor differences in luminosity
DESIRED_TORQUE = 5  # Example torque value, adjust as needed

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Initialize Dynamixel SDK
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        # Open serial port
        if self.port_handler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")
            quit()

        # Set port baudrate
        if self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")
            quit()

        # Enable torque for each Dynamixel motor
        self.init_motors()

        # Run initial torque control
        self.initial_torque_control()
        
        # Create a subscriber
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ambient_luminosity',
            self.listener_callback,
            10
        )

    def init_motors(self):
        for motor_id in DXL_IDs:
            # setting motors to "wheel mode"
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_CCW_ANGLE_LIMIT, 0)
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, ADDR_CW_ANGLE_LIMIT, 0)

            # enabling torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != 0:
                self.get_logger().error(f"Failed to enable torque for motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Error enabling torque for motor {motor_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"ID {motor_id}: Dynamixel torque enabled")

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

    def initial_torque_control(self):
        # Set torque mode and command desired torque for each motor
        for dxl_id in DXL_IDs:
            # Enable torque control mode
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_CONTROL_ENABLE, 1)
            
            # Command torque in CW direction
            self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_GOAL_TORQUE, DESIRED_TORQUE + 1024)  # CW
        
        # Wait for the motors to reach the desired torque (adjust the sleep time as necessary)
        self.get_logger().info('Applying initial torque...')
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5))

        # Disable torque control mode
        for dxl_id in DXL_IDs:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_CONTROL_ENABLE, 0)

    def listener_callback(self, msg):
        lum_data = msg.data
        if len(lum_data) != 4:
            self.get_logger().error('Received incorrect number of luminosity data points')
            return
        
        # Calculate the differences between opposite sensors
        x_diff = lum_data[3] - lum_data[1]
        y_diff = lum_data[2] - lum_data[0]

        x_velocity = int(0 if abs(x_diff) < DEAD_BAND else Kp * x_diff)
        y_velocity = int(0 if abs(y_diff) < DEAD_BAND else Kp * y_diff)

        # Set motor velocities within limits
        m3_velocity = int(max(min(-x_velocity, VELOCITY_LIMIT), -VELOCITY_LIMIT))
        m1_velocity = int(max(min(x_velocity, VELOCITY_LIMIT), -VELOCITY_LIMIT))

        m2_velocity = int(max(min(-y_velocity, VELOCITY_LIMIT), -VELOCITY_LIMIT))
        m0_velocity = int(max(min(y_velocity, VELOCITY_LIMIT), -VELOCITY_LIMIT))

        motor_velocities = [m0_velocity, m1_velocity, m2_velocity, m3_velocity]

        for i in range(len(motor_velocities)):
            if motor_velocities[i] < 0:
                motor_velocities[i] = -motor_velocities[i]  # Make velocity positive
                motor_velocities[i] |= 1024  # Set the direction bit for CW (clockwise)
            else:
                motor_velocities[i] &= 1023  # Ensure the direction bit for CCW (counter-clockwise) is not set

        # Send velocities to motors
        self.set_motor_velocity(0, motor_velocities[0])
        self.set_motor_velocity(1, motor_velocities[1])
        self.set_motor_velocity(2, motor_velocities[2])
        self.set_motor_velocity(3, motor_velocities[3])
        
        self.get_logger().info(f'Set velocities: m0={motor_velocities[0]}, m1={motor_velocities[1]}, m2={motor_velocities[2]}, m3={motor_velocities[3]}')

    def set_motor_velocity(self, dxl_id, velocity):
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_MOVING_SPEED, velocity)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
