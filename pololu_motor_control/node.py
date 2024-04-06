from maestro import Controller
import rclpy
from rclpy import Node
from std_msgs.msg import Int16, Bool

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
        super.__init__('maestro_chief')
        
        # Declare Parameter
        self.declare_parameter("port", "/dev/ttyACM0", "The port which the maestro is connected to")
        self.arm_status = True

        #Subscribe to inputs
        self.arm_sub = self.create_subscription(Bool, "/arm", self.armer)
        self.motor_1_sub = self.create_subscription(Int16, "/motor/1/output", self.updateMotor1)
        self.motor_2_sub = self.create_subscription(Int16, "/motor/2/output", self.updateMotor2)
        self.motor_3_sub = self.create_subscription(Int16, "/motor/3/output", self.updateMotor3)
        self.motor_4_sub = self.create_subscription(Int16, "/motor/4/output", self.updateMotor4)
        self.motor_5_sub = self.create_subscription(Int16, "/motor/5/output", self.updateMotor5)
        self.motor_6_sub = self.create_subscription(Int16, "/motor/6/output", self.updateMotor6)
        self.motor_7_sub = self.create_subscription(Int16, "/motor/7/output", self.updateMotor7)
        self.motor_8_sub = self.create_subscription(Int16, "/motor/8/output", self.updateMotor8)

        self.polo = Controller(ttyStr=self.get_parameter("port"))

    def _translate_pwm(self, pwm: int):
        return pwm*4
    
    def armer(self, message: Bool):
        if message.data == True:
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
    
    def updateMotor3(self, message: Int16):
        # Check if armed
        self.update_motor(3, message.data)
    
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
    marco_polo = MaestroWritter()
    while rclpy.ok():
        rclpy.spin(marco_polo)

    marco_polo.get_logger().warning("Shutting down")
    
if __name__ == '__main__':
    main()
