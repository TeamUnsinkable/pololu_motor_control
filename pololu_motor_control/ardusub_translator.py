import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16, Float32

class ArudoSubTranslator(Node):
    def __init__(self):
        super().__init__("ArduSubTranslator")
        self.motor_1_pub = self.create_publisher(Int16, "/output/motor1", 5)
        self.motor_2_pub = self.create_publisher(Int16, "/output/motor2", 5)
        self.motor_3_pub = self.create_publisher(Int16, "/output/motor3", 5)
        self.motor_4_pub = self.create_publisher(Int16, "/output/motor4", 5)
        self.motor_5_pub = self.create_publisher(Int16, "/output/motor5", 5)
        self.motor_6_pub = self.create_publisher(Int16, "/output/motor6", 5)
        self.motor_7_pub = self.create_publisher(Int16, "/output/motor7", 5)
        self.motor_8_pub = self.create_publisher(Int16, "/output/motor8", 5)

        self.create_subscription(Float32, "/controls/yaw/hotas", self.yaw_callback, 5)
        self.create_subscription(Float32, "/controls/surge/hotas", self.surge_callback, 5)
        self.create_subscription(Float32, "/controls/sway/hotas", self.sway_callback, 5)
        self.create_subscription(Float32, "/controls/depth/hotas", self.depth_callback, 5)
        self.get_logger().info("Doing the dishes")

    def _base_pwm_conversion(self, number):
        num = Int16()
        # Negative Limit Checking
        calculation = max(number+1500, 1100)
        # Positive Limit Checking
        calculation = min(calculation, 1900)
        num.data = int(round(calculation))
        return num

    def yaw_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        
        self.motor_2_pub.publish(num)
        self.motor_4_pub.publish(num)

        num.data *= -1
        self.motor_1_pub.publish(num)
        self.motor_3_pub.publish(num)
        
    def surge_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.motor_2_pub.publish(num)
        self.motor_1_pub.publish(num)
        
        num.data *= -1
        self.motor_4_pub.publish(num)
        self.motor_3_pub.publish(num)

    def sway_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.motor_2_pub.publish(num)
        self.motor_1_pub.publish(num)
        
        num.data *= -1
        self.motor_4_pub.publish(num)
        self.motor_3_pub.publish(num)
        

    def pitch_callback(self, msg: Float32):
        pass

    def depth_callback(self, msg: Float32):
        num = self._base_pwm_conversion(msg.data)
        self.motor_2_pub.publish(num)
        self.motor_1_pub.publish(num)
        self.motor_4_pub.publish(num)
        self.motor_3_pub.publish(num)

def main():
    rclpy.init()
    ardu_boi = ArudoSubTranslator()
    rclpy.spin(ardu_boi)
    rclpy.shutdown()