#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Joy
import pygame
from asmarine24.joy_map import joyMapClass

joy = joyMapClass()

pygame.init()

# Constants for screen dimensions
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 400

# Initialize the screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("AUV Keyboard Control")

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 36)

    def tprint(self, screen: pygame.Surface, text: str, red=0) -> None:
        text_bitmap = self.font.render(text, True, (red, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self) -> None:
        self.x = 20
        self.y = 10
        self.line_height = 40

    def indent(self) -> None:
        self.x += 10

    def unindent(self) -> None:
        self.x -= 10
    
    def new_line(self) -> None:
        self.y += 20

text_print = TextPrint()

def main() -> None:
    done = False
    
    rospy.init_node('joy_Node', anonymous=True)
    pub = rospy.Publisher('/Wrench', WrenchStamped, queue_size=1)
    rospy.Subscriber("/joy", Joy, joy.joy_cb, queue_size=1)
    rate = rospy.Rate(20)  # 20 Hz
    rospy.loginfo("JoyStick Node Started")
    
    # Main joystick loop
    while not done and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        # Set wrench values
        joy.slamMode()

        # Clear the screen
        screen.fill((255, 255, 255))
        text_print.reset()
        text_print.tprint(screen, f"Wrench", 250)
        text_print.new_line()
        
        # Print the wrench values
        text_print.tprint(screen, f"X: {joy.x_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Y: {joy.y_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Z: {joy.z_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Roll: {joy.roll_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Pitch: {joy.pitch_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Yaw: {joy.yaw_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Wrench Range: {joy.max_wrench_val}")
        text_print.new_line()
        
        # Publish the wrench values
        vector_msg = WrenchStamped()
        vector_msg.wrench.force.x = joy.x_wrench_mapped
        vector_msg.wrench.force.y = joy.y_wrench_mapped
        vector_msg.wrench.force.z = joy.z_wrench_mapped
        vector_msg.wrench.torque.x = joy.roll_wrench_mapped
        vector_msg.wrench.torque.y = joy.pitch_wrench_mapped
        vector_msg.wrench.torque.z = joy.yaw_wrench_mapped
        vector_msg.header.stamp = rospy.Time.now()
        pub.publish(vector_msg)
        

        #rospy.loginfo(f"Published: {vector_msg}")
        rate.sleep()
        
        # Update the screen
        pygame.display.flip()

        # Limit to 30 frames per second
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()