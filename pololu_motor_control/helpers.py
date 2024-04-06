


class Motor():
    def __init__(self) -> None:
        self.arming_speed = 1500
        pass

    def arm(self):
        pass
    
    def disarm(self):
        pass
    
    def updateSpeed(self, speed):
        pass

    def convertPWM(self, PWM: int):
        return PWM*4
    