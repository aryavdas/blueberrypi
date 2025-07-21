import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
import numpy as np
from mavros_msgs.msg import OverrideRCIn, ManualControl
from std_msgs.msg import Float32

class Depth_Publisher(Node):
    def __init__(self):

        self.flag = False
        self.pressure_initial: float = None
        self.pressure_current: float = None
        self.pressure_last: float = None

        super().__init__("depth_publisher")    # names the node when running

        self.sub = self.create_subscription(
            FluidPressure,
            "/pressure",
            self.pressure_callback,
            10
        )

        self.pub_depth = self.create_publisher(
            Float32,        # the message type
            "/depth",    # the topic name
            10              # QOS (will be covered later)
        )

    def pressure_callback(self, msg : FluidPressure):

        if self.flag == False:
            self.flag = True
            self.pressure_initial = msg.fluid_pressure
            self.pressure_last = msg.fluid_pressure
            self.pressure_current = msg.fluid_pressure
            self.get_logger().info(f"STARTING PRESSURE: + {self.pressure_initial}")
            return

        self.pressure_last = self.pressure_current
        self.pressure_current = msg.fluid_pressure
        # self.get_logger().info(f"current pressure: + {self.pressure_initial.fluid_pressure}")

        self.rel_pressure_current = self.pressure_current - self.pressure_initial
        self.rel_pressure_last = self.pressure_last - self.pressure_initial

        self.depth_last = self.pressure_to_depth(self. rel_pressure_current)
        self.depth_current = self.pressure_to_depth(self.rel_pressure_last)
        # self.get_logger().info(f"current depth: + {self.depth_current}")
        # self.get_logger().info(f"last depth: + {self.depth_last}")

        msg_float32 = Float32()
        msg_float32.data = self.depth_current

        self.pub_depth.publish(msg_float32)


    def pressure_to_depth(self, rel_pressure, rho=1000, g=9.81):
        return rel_pressure / (rho * g)

def main(args=None):
    rclpy.init(args=args)
    node = Depth_Publisher()


    try:
        # rclpy.spin_once(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()