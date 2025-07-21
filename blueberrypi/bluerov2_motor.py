import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np

class Publisher(Node):

    def __init__(self):
        super().__init__("motor_control")    # names the node when running

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.sub = self.create_subscription(
            ManualControl,
            "/motor_commands",
            self.manual_control_callback,
            10
        )
        # self.manual_control_callback(ManualControl())
        self.get_logger().info("initialized publisher node")

    def manual_control_callback(self, msg: ManualControl):
        """
        Send manual control message

        See https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """

        # msg.x = 50.0
        # msg.y = 0.0
        # msg.z = 0.0
        # msg.r = 0.0
        # self.get_logger().info(f"z motor: + {msg.z}")

        self.pub.publish(msg)

    def manual_control_callback_STOP(self, msg: ManualControl):

        msg.x = 0,0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0

        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    # node.manual_control_callback(ManualControl)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()