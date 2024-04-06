from pololu_motor_control.maestro import Controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool
from time import sleep

# Converts ardusub thruster mappings to pololu
# ardusub: pololu
motor_mapping = {
    1: 0,
    2: 1,
    3: 2,
    4: 3,
    5: 4,
    6: 5,
    7: 6,
    8: 7
}

class MaestroWritter(Node):
    def __init__(self):
        super().__init__('maestro_chief')
                
        # Declare Parameter
        self.declare_parameter("port", "/dev/ttyACM0")
        self.arm_status = True

        #Subscribe to inputs
        self.arm_sub = self.create_subscription(Bool, "/arm", self.armer, 5)
        self.motor_1_sub = self.create_subscription(Int16, "/motor1/output", self.updateMotor1, 5)
        self.motor_2_sub = self.create_subscription(Int16, "/motor2/output", self.updateMotor2, 5)
        self.motor_3_sub = self.create_subscription(Int16, "/motor3/output", self.updateMotor3, 5)
        self.motor_4_sub = self.create_subscription(Int16, "/motor4/output", self.updateMotor4, 5)
        self.motor_5_sub = self.create_subscription(Int16, "/motor5/output", self.updateMotor5, 5)
        self.motor_6_sub = self.create_subscription(Int16, "/motor6/output", self.updateMotor6, 5)
        self.motor_7_sub = self.create_subscription(Int16, "/motor7/output", self.updateMotor7, 5)
        self.motor_8_sub = self.create_subscription(Int16, "/motor8/output", self.updateMotor8, 5)

        self.polo = Controller(ttyStr=self.get_parameter("port").get_parameter_value().string_value)
        self.get_logger().warning("The spinny boys be spinning")


    def _translate_pwm(self, pwm: int):
        return pwm*4
    
    def armer(self, message: Bool):
        if message.data == True:
            # Re-enable the channel
            self.arm()
            
            sleep(1)

            # Re-enable
            self.arm()
            self.arm_status = True
            
        else:
            self.disarm()
            self.arm_status = False


    def arm(self):
        self.get_logger().warning("Arming spinny things")
        for thruster in range(8):
            self.update_motor(thruster+1, 1500)

    
    def disarm(self):
        self.get_logger().warning("diarming spinny things")
        for thruster in range(8):
            self.update_motor(thruster+1, 0)

    
    def update_motor(self, channel, value):
        if self.arm_status:
            # Set translated value
            self.polo.setTarget(motor_mapping[channel], self._translate_pwm(value))
    
    def updateMotor1(self, message: Int16):
        # Check if armed
        self.update_motor(1, message.data)

    def updateMotor2(self, message: Int16):
        # Check if armed
        self.update_motor(2, message.data)
    
    def updateMessage: Int16):
        # Check if armed
    def updateMotor4(self, message: Int16):
        # Check if armed
        self.update_motor(4, message.data)
    
    def updateMotor5(self, message: Int16):
        # Check if armed
        self.update_motor(5, message.data)

    def updateMotor6(self, message: Int16):
        # Check if armed
        self.update_motor(6, message.data)

    def updateMotor7(self, message: Int16):
        # Check if armed
        self.update_motor(7, message.data)

    def updateMotor8(self, message: Int16):
        # Check if armed
        self.update_motor(8, message.data)
    
        
def main():
    rclpy.init()
    marco_polo = MaestroWritter()
    while rclpy.ok():
        rclpy.spin(marco_polo)
    marco_polo.get_logger().warning("Shutting down")
    marco_polo.disarm()

    
if __name__ == '__main__':
    main()
