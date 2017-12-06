class ControllerState(object):
    """
    ControllerState represents the state of an entire Xbox controller at a given moment in time.
    """

    def __init__(self):
        # A, B, X, and Y buttons. 0 is off and 1 is on.
        self.cn_a = 0
        self.cn_b = 0
        self.cn_x = 0
        self.cn_y = 0

        # Left and right bumpers. 0 is off and 1 is on.
        self.cn_left_bumper = 0
        self.cn_right_bumper = 0

        # Middle, back, and start buttons, whatever those are. 0 is off and 1 is on.
        self.cn_middle = 0
        self.cn_back = 0
        self.cn_start = 0

        # Left joystick x and y axes. Signed int.
        self.cn_left_stick_x = 0
        self.cn_left_stick_y = 0

        # Right joystick x and y axes. Signed int.
        self.cn_right_stick_x = 0
        self.cn_right_stick_y = 0

        #  Positive in from 0 to 255
        self.cn_left_trigger = 0
        self.cn_right_trigger = 0

        # Up, down, left, and right on the D-Pad. 0 is off and 1 is on.
        self.cn_dpad_up = 0
        self.cn_dpad_down = 0
        self.cn_dpad_left = 0
        self.cn_dpad_right = 0