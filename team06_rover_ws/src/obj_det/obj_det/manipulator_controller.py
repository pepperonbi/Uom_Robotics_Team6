import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import sys

class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')
        # Initialize the robot
        self.bot = InterbotixManipulatorXS("px150", "arm", "gripper")
        if (self.bot.arm.group_info.num_joints < 5):
            self.get_logger().info('This demo requires the robot to have at least 5 joints!')
            sys.exit()

        # Initialize the subscriber to the 'goal_pose' topic
        self.subscription = self.create_subscription(
            Pose,
            '/pose_topic',
            self.goal_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def goal_pose_callback(self, msg):
        position = msg.position
        orientation = msg.orientation
        self.get_logger().info(f'Moving to position: x={position.x}, y={position.y}, z={position.z}')
        self.get_logger().info(f'Moving to orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')
        self.move_to_position(position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w)

    def move_to_position(self, x, y, z, or_x, or_y, or_z, or_w):
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, z=z, moving_time = 6.0)
        self.bot.gripper.grasp()
        self.bot.arm.go_to_sleep_pose()

def main(args=None):
    rclpy.init(args=args)
    manipulator_controller = ManipulatorController()
    rclpy.spin(manipulator_controller)

    # Cleanup before shutting down
    manipulator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
