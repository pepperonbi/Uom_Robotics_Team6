import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from object_detection import PoseEstimation
import cv2
import tf2_py as tf

class DesiredPositionPublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        # Correct the attribute name to match the initialization
        self.publisher = self.create_publisher(
            Pose,
            '/pose_topic',
            1)  # Using 1 for the QoS profile for simplicity

        # Immediately send the desired position once after creation
        self.publish_desired_position()

    def publish_desired_position(self):
        # Correct attribute reference to 'self.publisher'
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscribers...')
            # Use a small timeout for spin_once to allow other operations to proceed
            rclpy.spin_once(self, timeout_sec=0.5)
        
        pose = PoseEstimation()
        msg = Pose()
        msg.position = Point(x=0.3, y=0.0, z=0.055)
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # pose = PoseEstimation()
        # while True:
        #     rotation, translation = pose.get_pose()
        #     if rotation is not None and translation is not None:
        #         msg.position = Point(x=translation[0], y=translation[1], z=translation[2])
        #         quaternion = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        #         msg.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break

        # Publishing the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent desired position: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = DesiredPositionPublisher()
    # Allow some time for the message to be sent
    rclpy.spin_once(node, timeout_sec=1)
    # Shutdown properly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


