import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from collections import deque

from std_msgs.msg import Int16, Float32, Float64
import numpy as np

class ArudoSubTranslator(Node):
    def __init__(self):
        super().__init__("ArduSubTranslator") # type: ignore
        self.sub_cbg = ReentrantCallbackGroup()
        self.pub_cbg = ReentrantCallbackGroup()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("rate", 12)
        self.motor_publishers = [
            self.create_publisher(Int16, "/output/motor1", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor2", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor3", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor4", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor5", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor6", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor7", 10, callback_group=self.pub_cbg),
            self.create_publisher(Int16, "/output/motor8", 10, callback_group=self.pub_cbg),
        ]

        # Rotational Values
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        # Translational Values
        self.surge = 0
        self.sway = 0
        self.depth = 0

        # Rotational Vector
        self.yaw_vector     = np.array([-1, 1, 1, -1, 0, 0, 0, 0])
        self.pitch_vector   = np.array([0, 0, 0, 0, 1, 1, -1, -1])
        self.roll_vector    = np.array([0, 0, 0, 0, 1, -1, 1, -1])

        # Translational Vector
        self.surge_vector   = np.array([-1, -1, 1, 1, 0, 0, 0, 0])
        self.sway_vector    = np.array([-1, 1, -1, 1, 0, 0, 0, 0])
        self.depth_vector   = np.array([0, 0, 0, 0, -1, -1, -1, -1])
        

        self.create_timer(1/self.get_parameter("rate").get_parameter_value().integer_value, self.timer_callback, callback_group=self.timer_cbg)
        # Rotational PIDs
        self.create_subscription(Float64, "/controls/roll/control_effort", self.yaw_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/pitch/control_effort", self.yaw_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/yaw/control_effort", self.yaw_callback, 10, callback_group=self.sub_cbg)
        
        # Translational Inputs
        self.create_subscription(Float64, "/controls/surge/hotas", self.surge_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/sway/hotas", self.sway_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/depth/hotas", self.depth_callback, 10, callback_group=self.sub_cbg)

        self.get_logger().info("Doing the dishes")

    def _base_pwm_conversion(self, number):
        num = round(number)
        # Negative Limit Checking
        calculation = max(round(num), -400)
        # Positive Limit Checking
        calculation = min(round(calculation), 400)
        return num
    
    def _full_pwm_conversion(self, number):
        number = self._base_pwm_conversion(number) + 1500
        # Negative Limit Checking
        calculation = max(round(number), 1100)
        # Positive Limit Checking

        calculation = min(round(calculation), 1900)
        return calculation
    
    def timer_callback(self) -> None:
        begin = self.get_clock().now()
        value = Int16()

        yaw   = self.yaw   * self.yaw_vector
        pitch = self.pitch * self.pitch_vector
        roll  = self.pitch * self.roll_vector

        surge = self.surge * self.surge_vector
        sway  = self.sway  * self.sway_vector
        depth = self.depth * self.depth_vector
        

        for idx, motor in enumerate(self.motor_publishers):
            self.get_logger().info(f"Motor {idx}: {(yaw[idx] + surge[idx] + sway[idx] + depth[idx] +pitch[idx]) + 1500}  [ya:{yaw[idx]}, su:{surge[idx]}, sw:{sway[idx]}, dp:{depth[idx]}, pi:{pitch[idx]} ] ")
            value.data = int(self._base_pwm_conversion((surge[idx] + sway[idx] + depth[idx] + pitch[idx] + roll[idx] + yaw[idx])))
            motor.publish(value)

        end = self.get_clock().now()
        self.get_logger().debug(f"Timer callback time was: {begin - end}")

    ## Rotational Callbacks
    def yaw_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.yaw = num
    
    def roll_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.roll = num

    def pitch_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.pitch = num

    # Translational Callbacks
    def surge_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        # Motor 1 and 2
        # Motor 3 and 4 Inversed
        self.surge = num

    def sway_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        # Motor 2 and 4
        # Motor 1 and 3 Inversed
        self.sway = num

    def depth_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.depth = num

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    ardu_boi = ArudoSubTranslator()
    executor.add_node(ardu_boi)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()
