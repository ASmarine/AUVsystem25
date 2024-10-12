from math import sqrt
from rospy import sleep
"""
PS5 Controller Button Mapping (sensor_msgs/Joy)
-----------------------------------------------
buttons[0]  : ✖️ Button
buttons[1]  : 🔴 Button
buttons[2]  : 🔺 Button
buttons[3]  : 🟪 Button
buttons[4]  : Left Bumper (L1)
buttons[5]  : Right Bumper (R2)
buttons[8]  : Back Button
buttons[9]  : Start Button
buttons[10]  : Power/Guide Button
buttons[12]  : Left Stick Button (L3)
buttons[13] : Right Stick Button (R3)

PS5 Controller Axes Mapping (sensor_msgs/Joy)
----------------------------------------------
axes[0]  : Left Stick Horizontal (1.0: left, -1.0: right)
axes[1]  : Left Stick Vertical (1.0: up, -1.0: down)
axes[2]  : Left Trigger (1.0: unpressed, -1.0: fully pressed)
axes[3]  : Right Stick Horizontal (1.0: left, -1.0: right)
axes[4]  : Right Stick Vertical (1.0: up, -1.0: down)
axes[5]  : Right Trigger (1.0: unpressed, -1.0: fully pressed)
axes[6]  : D-pad Horizontal (-1.0: left, 1.0: right)
axes[7]  : D-pad Vertical (-1.0: up, 1.0: down)
"""
class joyMapClass:

    def __init__(self):
        self.left_analog_x = 0.0
        self.left_analog_y = 0.0 
        self.right_analog_x = 0.0
        self.right_analog_y = 0.0 
        self.left_trigger = 0.0 
        self.right_trigger = 0.0 
        self.arrow_up_down = 0.0
        self.arrow_left_right = 0.0
        self.right_bumper = 0.0
        self.left_bumper = 0.0 
        self.start_button = 0.0
        self.select_button = 0.0
        self.x_button = 0.0
        self.circle_button = 0.0
        self.triangle_button = 0.0 
        self.square_button = 0.0

        self.max_wrench_val = 5
        self.x_wrench_mapped = 0.0
        self.y_wrench_mapped = 0.0
        self.z_wrench_mapped = 0.0
        self.roll_wrench_mapped = 0.0
        self.pitch_wrench_mapped = 0.0
        self.yaw_wrench_mapped = 0.0

    def joy_cb(self,msg):
        self.left_analog_x = -1*msg.axes[0]          #The left analog stick X axis is inverted
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = -1*msg.axes[3]         #The right analog stick X axis is inverted
        self.right_analog_y = msg.axes[4]
        self.left_trigger = -1*msg.axes[2] +1        #The triggers are -1 when pressed, 1 when not pressed \
        self.right_trigger = -1*msg.axes[5] +1       #So we add 1 to make the reading between 0 and 2
        self.arrow_up_down = msg.axes[7]             #The D-pad 1 for up and -1 for down 
        self.arrow_left_right = -1*msg.axes[6]       #The D-pad -1 for left and 1 for right 
        self.right_bumper = msg.buttons[5]           #The right bumper is 1 when pressed and 0 when not pressed
        self.left_bumper = msg.buttons[4]            #The left bumper is 1 when pressed and 0 when not pressed
        self.start_button = msg.buttons[9]           #The start button is 1 when pressed and 0 when not pressed
        self.select_button = msg.buttons[8]          #The select button is 1 when pressed and 0 when not pressed
        self.x_button = msg.buttons[0]               #The x button is 1 when pressed and 0 when not pressed
        self.circle_button = msg.buttons[1]          #The circle button is 1 when pressed and 0 when not pressed
        self.triangle_button = msg.buttons[2]        #The triangle button is 1 when pressed and 0 when not pressed
        self.square_button = msg.buttons[3]          #The square button is 1 when pressed and 0 when not pressed


    def map_wrench_val(self):
        """
        Maps the joystick values to the wrench max values
        """    

        if(self.start_button):
            if(self.max_wrench_val == 20):
                return
            self.max_wrench_val += 1
            sleep(0.15)
        if(self.select_button):
            if(self.max_wrench_val == 0):
                return
            self.max_wrench_val -= 1
            sleep(0.15)

    def map_XY(self):
        """
        Maps the left analog stick to the 4 horizontal motors,
        which are responsible for the forward, backward, left and right motion
        and yaw rotation
        """

        if (self.arrow_up_down != 0 or self.arrow_left_right != 0):
            self.x_wrench_mapped = self.arrow_up_down * self.max_wrench_val
            self.y_wrench_mapped = self.arrow_left_right * self.max_wrench_val
            return 

        self.x_wrench_mapped =  self.max_wrench_val * self.left_analog_y 
        self.y_wrench_mapped =  self.max_wrench_val * self.left_analog_x 

    def map_RollPitch(self):
        """
        Maps the right analog stick to the 4 vertical motors,
        which are responsible for the roll and pitch motion.
        Only works if L1 is not pressed.
        """

        if (not self.left_bumper):
            self.pitch_wrench_mapped = 0
            self.roll_wrench_mapped = 0
            return

        self.pitch_wrench_mapped =  self.max_wrench_val * self.right_analog_y 
        self.roll_wrench_mapped =  self.max_wrench_val * self.right_analog_x 

    def map_Z(self):
        """
        Maps the left and right triggers to the 4 vertical motors,
        which are responsible for the altitude control
        """

        if (self.triangle_button != 0 or self.x_button != 0):
            self.z_wrench_mapped = (self.triangle_button + -1*self.x_button ) * self.max_wrench_val
            return
        
        if (self.left_bumper):
            self.z_wrench_mapped = 0
            return

        if(self.left_trigger > 0 and self.right_trigger > 0):
            self.z_wrench_mapped = 0
            return

        self.left_trigger = -1*self.left_trigger
        self.z_wrench_mapped = self.max_wrench_val * (self.left_trigger + self.right_trigger) / 2

    def map_yaw(self):
        """
        Maps the left and right triggers to the 4 vertical motors,
        which are responsible for the altitude control
        """

        if (self.circle_button != 0 or self.square_button != 0):
            self.yaw_wrench_mapped = ( self.circle_button + -1*self.square_button ) * self.max_wrench_val
            return
        
        if (not self.left_bumper):
            self.yaw_wrench_mapped = self.max_wrench_val * self.right_analog_x
            return

        if(self.left_trigger > 0 and self.right_trigger > 0):
            self.yaw_wrench_mapped = 0
            return

        self.left_trigger = -1*self.left_trigger
        self.yaw_wrench_mapped = self.max_wrench_val * (self.left_trigger + self.right_trigger) / 2

    def doMapping(self):
        self.map_wrench_val()
        self.map_XY()
        self.map_RollPitch()
        self.map_Z()
        self.map_yaw()

    def slamMode(self):
        """
        This function is used to map the joystick values to the wrench values
        suitable for the SLAM testing
        """
        self.map_wrench_val()

        if (self.square_button == 0 and self.circle_button == 0 ):                                   # there is no YAW direction motion
            self.x_wrench_mapped = self.arrow_up_down * self.max_wrench_val                          # maybe there is X direction motion
            self.y_wrench_mapped = self.arrow_left_right * self.max_wrench_val                       # maybe there is no Y direction motion
            self.z_wrench_mapped = (self.triangle_button + -1*self.x_button ) * self.max_wrench_val  # maybe there is no Z direction motion
            self.yaw_wrench_mapped = 0
            return
        
        elif ((self.square_button != 0 or self.circle_button != 0) and  # there is a YAW direction motion
            self.arrow_left_right == 0 and                              # there is no X direction motion
            self.arrow_up_down == 0 and                                 # there is no Y direction motion
            self.triangle_button == 0 and self.x_button == 0):          # there is no Z direction motion
            self.yaw_wrench_mapped = ( self.circle_button + -1*self.square_button ) * self.max_wrench_val / 2 
            self.x_wrench_mapped = 0
            self.y_wrench_mapped = 0
            self.z_wrench_mapped = 0
            return

        
