from pololu_motor_control.maestro import Controller
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool
from time import sleep

# Converts ardusub thruster mappings to pololu
# ardusub: pololu
motor_mapping = {
    1: 3,
    2: 2,
    3: 1,
    4: 0,
    5: 7,
    6: 6,
    7: 5,
    8: 4
}

class MaestroWritter(Node):
    def __init__(self):
        super().__init__('maestro_chief')
                
        # Declare Parameter
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("rate", 10)
        self.arm_status = True

        self.motor_out = []
        for _ in range(8):
            self.motor_out.append(1500)
        
        self._update_rate = 1/self.get_parameter("rate").get_parameter_value().integer_value

        #Subscribe to inputs
        self.arm_sub = self.create_subscription(Bool, "/arm", self.armer, 5)
        self.motor_1_sub = self.create_subscription(Int16, "/output/motor1", self.updateMotor1, 7)
        self.motor_2_sub = self.create_subscription(Int16, "/output/motor2", self.updateMotor2, 7)
        self.motor_3_sub = self.create_subscription(Int16, "/output/motor3", self.updateMotor3, 7)
        self.motor_4_sub = self.create_subscription(Int16, "/output/motor4", self.updateMotor4, 7)
        self.motor_5_sub = self.create_subscription(Int16, "/output/motor5", self.updateMotor5, 7)
        self.motor_6_sub = self.create_subscription(Int16, "/output/motor6", self.updateMotor6, 7)
        self.motor_7_sub = self.create_subscription(Int16, "/output/motor7", self.updateMotor7, 7)
        self.motor_8_sub = self.create_subscription(Int16, "/output/motor8", self.updateMotor8, 7)
        self.polo_update = self.create_timer(self._update_rate, self.timer_callback)
        
        self.polo = Controller(ttyStr=self.get_parameter("port").get_parameter_value().string_value)
        self.get_logger().info(f'Ardusub Translation Table\n{motor_mapping}')
        self.get_logger().info("The spinny boys be spinning")

        # Start ESC in off state
        self.disarm()


    def _translate_pwm(self, pwm: int):
        return pwm*4
    
    def armer(self, message: Bool):
        if message.data == True:
            # Re-enable the channel
            self.arm()
            self.arm_status = True
            
        else:
            self.disarm()
            self.arm_status = False


    def arm(self):
        self.get_logger().warning("arming spinny things")
        for thruster in range(8):
            self.polo.setTarget(thruster, 1500)

    
    def disarm(self):
        self.get_logger().warning("diarming spinny things")
        for thruster in range(8):
            self.polo.setTarget(thruster, 0)

    def timer_callback(self):
        for idx, value in enumerate(self.motor_out):
            self._updateMotor(motor_mapping[idx+1], value)


    def _updateMotor(self, channel, value):
        if self.arm_status == True and value != 0: 
            # Set translated value
            self.polo.setTarget(motor_mapping[channel], self._translate_pwm(value))
    
    def updateMotor1(self, message: Int16):
        # Check if armed
        self.motor_out[1-1] = message.data

    def updateMotor2(self, message: Int16):
        # Check if armed
        self.motor_out[2-1] = message.data
    
    def updateMotor3(self, message: Int16):
        # Check if armed
        self.motor_out[3-1] = message.data

    def updateMotor4(self, message: Int16):
        # Check if armed
        self.motor_out[4-1] = message.data
    
    def updateMotor5(self, message: Int16):
        # Check if armed
        self.motor_out[5-1] = message.data

    def updateMotor6(self, message: Int16):
        # Check if armed
        self.motor_out[6-1] = message.data

    def updateMotor7(self, message: Int16):
        # Check if armed
        self.motor_out[7-1] = message.data

    def updateMotor8(self, message: Int16):
        # Check if armed
        self.motor_out[8-1] = message.data
    
        
def main():
    rclpy.init()
    marco_polo = MaestroWritter()
    while rclpy.ok():
        rclpy.spin(marco_polo)
    marco_polo.get_logger().warning("Shutting down")
    marco_polo.disarm

    
if __name__ == '__main__':
    main()
