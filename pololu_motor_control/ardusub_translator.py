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

        self.motor_publishers = [
            self.create_publisher(Float32, "/output/motor1", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor2", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor3", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor4", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor5", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor6", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor7", 10, callback_group=self.pub_cbg),
            self.create_publisher(Float32, "/output/motor8", 10, callback_group=self.pub_cbg),
        ]

        # Rotational Values
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

        # Translational Values
        self.surge = 0
        self.sway = 0
        self.depth = 0

        # # Rotational Vector
        # self.yaw_vector     = np.array([-1, 1, 1, -1, 0, 0, 0, 0])
        # self.pitch_vector   = np.array([0, 0, 0, 0, 1, 1, -1, -1])
        # self.roll_vector    = np.array([0, 0, 0, 0, 1, -1, 1, -1])

        # # Translational Vector
        # self.surge_vector   = np.array([-1, -1, 1, 1, 0, 0, 0, 0])
        # self.sway_vector    = np.array([-1, 1, -1, 1, 0, 0, 0, 0])
        # self.depth_vector   = np.array([0, 0, 0, 0, -1, -1, -1, -1])
        

        self.create_timer(self.rate, self.timer_callback, callback_group=self.timer_cbg)

        # TODO Rework topic name logic
        # Rotational PIDs
        self.create_subscription(Float64, "/controls/roll/control_effort", self.roll_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/pitch/control_effort", self.pitch_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/yaw/control_effort", self.yaw_callback, 10, callback_group=self.sub_cbg)
        
        # Translational Inputs
        self.create_subscription(Float64, "/controls/surge/control_effort", self.surge_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/sway/control_effort", self.sway_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float64, "/controls/depth/control_effort", self.depth_callback, 10, callback_group=self.sub_cbg)

        self.get_logger().info("Doing the dishes")

    def _get_parameters(self):
        # Timer Rate - rate is in seconds
        self.declare_parameter("rate", 10)
        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        # Clamp min and max
        self.declare_parameter('clamp_min', 1.0)
        self.declare_parameter('clamp_max', 1900)
        self.clamp_min = self.get_parameter('clamp_min').get_parameter_value().double_value
        self.clamp_max = self.get_parameter('clamp_max').get_parameter_value().double_value

        # TODO better namings
        # Offset min and maxs
        self.declare_parameter('scale_min', -1.0)
        self.declare_parameter('scale_max', 1.0)
        self.scale_min = self.get_parameter('scale_min').get_parameter_value().double_value
        self.scale_max = self.get_parameter('scale_max').get_parameter_value().double_value

        self.declare_parameter('base_offset', 0)
        self.offset = self.get_parameter('base_offset').get_parameter_value().double_value
        
        ################
        # Topic Naming #
        ################

        # Pacakge logic will append respective Motor/ESC nunmber to end of topic name
        self.declare_parameter('motor_output_topic_base', "control/motor") 


        ####################
        # Thrust Vectoring #
        ####################

        # Translational Vectors
        self.declare_parameter('yaw_vector',   [-1.0,1.0,1.0,-1.0,0.0,0.0,0.0,0.0])
        self.declare_parameter('pitch_vector', [0.0,0.0,0.0,0.0,1.0,1.0,-1.0,-1.0])
        self.declare_parameter('roll_vector',  [0.0,0.0,0.0,0.0,1.0,1.0,-1.0,-1.0])
        self.yaw_vector   =  np.array(self.get_parameter('yaw_vector').get_parameter_value().double_array_value)
        self.pitch_vector =  np.array(self.get_parameter('pitch_vector').get_parameter_value().double_array_value)
        self.roll_vector  =  np.array(self.get_parameter('roll_vector').get_parameter_value().double_array_value)

        # Rotational Vectors
        self.declare_parameter('surge_vector', [0.0,0.0,0.0,0.0,1.0,-1.0,-1.0,1.0])
        self.declare_parameter('sway_vector',  [-1.0,1.0,-1.0,1.0,0.0,0.0,0.0,0.0])
        self.declare_parameter('depth_vector', [0.0,0.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0])
        self.surge_vector = np.array(self.get_parameter('surge_vector').get_parameter_value().double_array_value)
        self.sway_vector  = np.array(self.get_parameter('sway_vector').get_parameter_value().double_array_value)
        self.depth_vector = np.array(self.get_parameter('depth_vector').get_parameter_value().double_array_value)


    def _base_pwm_conversion(self, number):
        num = round(number)
        # Negative Limit Checking
        calculation = max(round(num), self.scale_min)
        # Positive Limit Checking
        calculation = min(round(calculation), self.scale_max)
        return calculation
    
    def _full_pwm_conversion(self, number):
        number = self._base_pwm_conversion(number) + self.offset
        # Negative Limit Checking
        calculation = max(round(number), self.clamp_min)
        # Positive Limit Checking
        calculation = min(round(calculation), self.clamp_max)
        return calculation
    
    def timer_callback(self) -> None:
        begin = self.get_clock().now()
        value = Int16()
        yaw   = self.yaw   * self.yaw_vector
        pitch = self.pitch * self.pitch_vector
        roll  = self.roll * self.roll_vector

        surge = self.surge * self.surge_vector
        sway  = self.sway  * self.sway_vector
        depth = self.depth * self.depth_vector
        
        # TODO Review Logic
        for idx, motor in enumerate(self.motor_publishers):
            self.get_logger().info(f"Motor {idx}: {(yaw[idx] + surge[idx] + sway[idx] + depth[idx] +pitch[idx]) + 1500}  [ya:{yaw[idx]}, su:{surge[idx]}, sw:{sway[idx]}, dp:{depth[idx]}, pi:{pitch[idx]} ] ")
            # self.get_logger().info(f"Motor {idx}: [ya:{yaw[idx]}, su:{surge[idx]}, sw:{sway[idx]}, dp:{depth{[idx]} ] ")
            value.data = int(self._base_pwm_conversion((yaw[idx] + surge[idx] + sway[idx] + depth[idx] + pitch[idx]+ roll[idx])) + 1500)
            motor.publish(value)

        end = self.get_clock().now()
        self.get_logger().debug(f"Timer callback time was: {begin - end}")

    ###################
    # Topic Callbacks #
    ###################

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
