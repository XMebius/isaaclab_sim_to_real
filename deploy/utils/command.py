class NoneVelCommand():
    def __init__(self, command):

        self.R1_pressed = False
        self.R1 = None
        self.R2_pressed = False
        self.R2 = None
        self.L1_pressed = False
        self.L1 = None
        self.L2_pressed = False
        self.L2 = None
        
    def update_command(self, msg) -> None:
        # lock rising edge
        self.R1_pressed = ((msg.R1 and not self.R1) or self.R1_pressed)
        self.R2_pressed = ((msg.R2 and not self.R2) or self.R2_pressed)
        self.L1_pressed = ((msg.L1 and not self.L1) or self.L1_pressed)
        self.L2_pressed = ((msg.L2 and not self.L2) or self.L2_pressed)

        # store value
        self.R1 = msg.R1
        self.R2 = msg.R2
        self.L1 = msg.L1
        self.L2 = msg.L2

    def pre_check_calibration(self) -> bool:
        return self.R2_pressed
    
    def post_check_calibration(self) -> bool:
        return self.L2_pressed

    def change_passive(self) -> bool:
        return self.R2_pressed
    
    def reset_R1_pressed(self) -> None:
        self.R1_pressed = False
    
    def reset_R2_pressed(self) -> None:
        self.R2_pressed = False
    
    def reset_L1_pressed(self) -> None:
        self.L1_pressed = False

    def reset_L2_pressed(self) -> None:
        self.L2_pressed = False