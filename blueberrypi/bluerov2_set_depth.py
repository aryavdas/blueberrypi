import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
import numpy as np
from mavros_msgs.msg import OverrideRCIn, ManualControl
from std_msgs.msg import Float32

class Pub_Depth(Node):
    def __init__(self):

        super().__init__("Depth_Target_Pub")    # names the node when running

        self.pub_depth = self.create_publisher(
            Float32,        # the message type
            "/depth_target",    # the topic name
            10              # QOS (will be covered later)
        )

        self.pub_callback()

    def pub_callback(self):
        target_depth = Float32()
        target_depth.data = float(input("Target Depth(float):"))
        self.pub_depth.publish(target_depth)

    

def main(args=None):
    rclpy.init(args=args)
    node = Pub_Depth()


    try:
        # rclpy.spin_once(node)
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()