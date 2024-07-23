import rclpy
import time
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
from sensor_msgs.msg import Joy

class ArmController(Node):
    # Control table address
    ADDR_MX_TORQUE_ENABLE = 24  
    ADDR_MX_MOVING_SPEED = 32
    ADDR_MX_PRESENT_SPEED = 38
    ADDR_CW_ANGLE_LIMIT = 6
    ADDR_CCW_ANGLE_LIMIT = 8

    # Default setting
    DXL_IDS = [0, 1, 2, 3]                
    BAUDRATE = 57600  # Dynamixel default baudrate : 57600
    DEVICENAME = '/dev/ttyDynamixel'  # Check which port is being used on your controller

    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    # Protocol Version
    PROTOCOL_VERSION = 1.0

    def __init__(self):
        super().__init__('arm_controller')
        self.declare_parameter('device_name', self.DEVICENAME)
        self.declare_parameter('baud_rate', self.BAUDRATE)

        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Open serial port
        if self.port_handler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")
            quit()

        # Set port baudrate
        if self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")
            quit()

        self.motor_ids = self.DXL_IDS
        self.init_motors()

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def init_motors(self):

        for motor_id in self.motor_ids:
            # setting motors to "wheel mode"
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.ADDR_CCW_ANGLE_LIMIT, 0)
            self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, self.ADDR_CW_ANGLE_LIMIT, 0)

            # enabling torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != 0:
                self.get_logger().error(f"Failed to enable torque for motor {motor_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Error enabling torque for motor {motor_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"ID {motor_id}: Dynamixel torque enabled")

            time.sleep(0.5)

    def joy_callback(self, msg):
        button_to_motor = {
            0: 1,  # A -> motor 1
            2: 2,  # X -> motor 2
            3: 3,  # Y -> motor 3
            1: 4   # B -> motor 4
        }
        
        right_stick_y = msg.axes[4]  # Right stick vertical axis (assuming index 4)
        selected_motor = None
        
        for button_index, motor_id in button_to_motor.items():
            if msg.buttons[button_index] == 1:
                selected_motor = motor_id
                break

        if selected_motor is not None:
            # Calculate velocity from right stick position
            velocity = int(right_stick_y * 1023)  # Dynamixel speed range for speed is 0-1023
            self.get_logger().info(f"Velocity: {velocity}")
            if velocity < 0:
                velocity = -velocity  # Make velocity positive
                velocity |= 1024  # Set the direction bit for CW (clockwise)
            else:
                velocity &= 1023  # Ensure the direction bit for CCW (counter-clockwise) is not set
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, selected_motor, self.ADDR_MX_MOVING_SPEED, velocity)
            if dxl_comm_result != 0:
                self.get_logger().error(f"Failed to set velocity for motor {selected_motor}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                self.get_logger().error(f"Error setting velocity for motor {selected_motor}: {self.packet_handler.getRxPacketError(dxl_error)}")
            else:
                self.get_logger().info(f"Motor {selected_motor} set to velocity {velocity}")

    def destroy_node(self):
        for motor in self.motor_ids:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.port_handler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
