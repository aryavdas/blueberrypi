import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
import numpy as np
from mavros_msgs.msg import OverrideRCIn, ManualControl

class PID_Subscriber(Node):
    def __init__(self):

        self.flag = False
        self.pressure_initial: float = None
        self.pressure_current: float = None
        self.pressure_last: float = None
        self.time_current : float = None
        self.time_last : float = None
        self.depth_target : float = 2 # CHANGE TARGET DEPTH FOR TARGET
        
        self.k_p : float = 15
        self.k_i : float = 2
        self.k_d : float = 5

        self.error_last : float = None
        self.error_current : float = None

        self.u : int = 0

        self.sum = 0

        super().__init__("PID_depth_subscriber")    # names the node when running

        self.sub = self.create_subscription(
            FluidPressure,
            "/pressure",
            self.pressure_callback,
            10
        )

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/motor_commands",    # the topic name
            10              # QOS (will be covered later)
        )

    def pressure_callback(self, msg : FluidPressure):
        self.time_last = self.time_current
        self.time_temp = self.get_clock().now().seconds_nanoseconds()
        self.time_current = self.time_temp[0] + (self.time_temp[1] / (10 ** 9))

        if self.flag == False:
            self.flag = True
            self.pressure_initial = msg.fluid_pressure
            self.pressure_last = msg.fluid_pressure
            self.pressure_current = msg.fluid_pressure
            self.get_logger().info(f"STARTING PRESSURE: + {self.pressure_initial}")
            return
        
        self.time_delta = self.time_current - self.time_last

        self.pressure_last = self.pressure_current
        self.pressure_current = msg.fluid_pressure
        # self.get_logger().info(f"current pressure: + {self.pressure_initial.fluid_pressure}")
        # self.get_logger().info(f"current time: + {self.time_current}")

        self.rel_pressure_current = self.pressure_current - self.pressure_initial
        self.rel_pressure_last = self.pressure_last - self.pressure_initial

        self.depth_last = self.pressure_to_depth(self. rel_pressure_current)
        self.depth_current = self.pressure_to_depth(self.rel_pressure_last)
        # self.get_logger().info(f"current depth: + {self.depth_current}")
        # self.get_logger().info(f"last depth: + {self.depth_last}")

        self.u = 0

        if self.error_last == None:
            self.error_last = self.depth_target - self.depth_current
        else:
            self.error_last = self.error_current
        self.error_current = self.depth_target - self.depth_current

        self.u += self.k_p * self.error_current # K P

        self.deriv = (self.error_current - self.error_last) / (self.time_delta)
        self.u += self.k_d * self.deriv # K D

        self.sum += (self.error_current + self.error_last) * self.time_delta / 2

        self.u += self.k_i * self.sum # K I

        if(self.u >= 100):
            self.u = 100

        self.u *= -1

        msg_motor = ManualControl()
        msg_motor.x = 0.0
        msg_motor.y = 0.0
        msg_motor.z = float(self.u)
        msg_motor.r = 0.0

        self.get_logger().info(f"depth: {self.depth_current}")
        self.get_logger().info(f"p: {float(self.error_current * self.k_p)}, i: {float(self.sum * self.k_i)}, d:{float(self.deriv * self.k_d)}")

        # self.get_logger().info(f"current depth: + {self.depth_current}")
        # self.get_logger().info(f"z force: + {float(self.u)}")

        self.pub.publish(msg_motor)


    def pressure_to_depth(self, rel_pressure, rho=1000, g=9.81):
        return rel_pressure / (rho * g)

def main(args=None):
    rclpy.init(args=args)
    node = PID_Subscriber()


    try:
        # rclpy.spin_once(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()