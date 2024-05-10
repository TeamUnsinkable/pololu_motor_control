import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from collections import deque

from std_msgs.msg import Int16, Float32

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

        # Indexes are {0: Yaw, 1: Surge, 2: Sway}
        self.motor_values = [deque()]*8

        self.create_timer(1/self.get_parameter("rate").get_parameter_value().integer_value, self.timer_callback, callback_group=self.timer_cbg)
        self.create_subscription(Float32, "/controls/yaw/hotas", self.yaw_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float32, "/controls/surge/hotas", self.surge_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float32, "/controls/sway/hotas", self.sway_callback, 10, callback_group=self.sub_cbg)
        self.create_subscription(Float32, "/controls/depth/hotas", self.depth_callback, 10, callback_group=self.sub_cbg)
        self.get_logger().info("Doing the dishes")

    def _base_pwm_conversion(self, number):
        num = round(number)
        # Negative Limit Checking
        calculation = max(round(num), -400)
        # Positive Limit Checking
        calculation = min(round(calculation), 400)
        return num
    
    def timer_callback(self) -> None:
        begin = self.get_clock().now()
        for idx, queue in enumerate(self.motor_values):
            self.get_logger().info(f"Processing PWM for Motor: {idx+1}")
            # Create Empty Message
            msg = Int16()
            # Setup Average Parameters
            cnt = len(queue)
            tot = 0
            
            # Ensure not empty.
            # Avoids 0 Divison Error later
            if cnt == 0:
                self.get_logger().info(f"No updates for motor: {idx+1}")
                continue

            # Try for exepcted amount but be ready for failures
            try:
                for _ in range(len(queue)):
                    tot += queue.popleft()
            except IndexError:
                # No more values in array
                self.get_logger().info(f"Motors IDX: {idx+1} ran into issues computing PWM")
            
            # Avoid 0 Division Error 
            avg = tot/cnt

            if avg == 0:
                msg.data = 1500
            else:
                msg.data = self._base_pwm_conversion(avg)+1500

            if round(msg.data) == -1500:
                continue
            
            self.motor_publishers[idx].publish(msg)
        end = self.get_clock().now()
        self.get_logger().debug(f"Timer callback time was: {begin - end}")



    def yaw_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        # Motor 2 and 3 Foward
        self.motor_values[1].append(num)
        self.motor_values[2].append(num)

        # Motor 1 and 4 Inversed
        self.motor_values[0].append(-num)
        self.motor_values[3].append(-num)

        
    def surge_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        # Motor 1 and 2
        self.motor_values[0].append(num)
        self.motor_values[1].append(num)
        
        # Motor 3 and 4 Inversed
        self.motor_values[2].append(-num)
        self.motor_values[3].append(-num)



    def sway_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        # Motor 2 and 4
        self.motor_values[1].append(num)
        self.motor_values[3].append(num)
        
        # Motor 1 and 3 Inversed
        self.motor_values[0].append(-num)
        self.motor_values[2].append(-num)

     

    def pitch_callback(self, msg: Float32):
        pass

    def depth_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.motor_values[4].append(num)
        self.motor_values[5].append(num)
        self.motor_values[6].append(num)
        self.motor_values[7].append(num)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    ardu_boi = ArudoSubTranslator()
    executor.add_node(ardu_boi)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()
