#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
import pygame

pygame.init()

# Constants for screen dimensions
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 400

max_range = 5

x_wrench_mapped = 0
y_wrench_mapped = 0
z_wrench_mapped = 0
roll_wrench_mapped = 0
pitch_wrench_mapped = 0
yaw_wrench_mapped = 0

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

def map_range(value: float, max_range: float) -> float:
    return value * max_range

def main() -> None:
    global x_wrench_mapped, y_wrench_mapped, z_wrench_mapped
    global pitch_wrench_mapped, yaw_wrench_mapped, roll_wrench_mapped
    global max_range
    done = False
    
    rospy.init_node('Keyboard_Node', anonymous=True)
    pub = rospy.Publisher('/Wrench', WrenchStamped, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz
    rospy.loginfo("Keyboard Node Started")
    
    # Main keyboard loop
    while not done and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        keys = pygame.key.get_pressed()

        # Reset wrench values
        x_wrench_mapped = 0
        y_wrench_mapped = 0
        z_wrench_mapped = 0
        roll_wrench_mapped = 0
        pitch_wrench_mapped = 0
        yaw_wrench_mapped = 0

        # Map keyboard inputs to wrench values
        if keys[pygame.K_LEFT]:
            y_wrench_mapped = -max_range
        elif keys[pygame.K_RIGHT]:
            y_wrench_mapped = max_range

        if keys[pygame.K_DOWN]:
            x_wrench_mapped = -max_range
        elif keys[pygame.K_UP]:
            x_wrench_mapped = max_range

        if keys[pygame.K_SPACE]:
            z_wrench_mapped = max_range
        elif keys[pygame.K_b]:
            z_wrench_mapped = -max_range

        if keys[pygame.K_q]:
            roll_wrench_mapped = max_range
        elif keys[pygame.K_e]:
            roll_wrench_mapped = -max_range

        if keys[pygame.K_s]:
            pitch_wrench_mapped = max_range
        elif keys[pygame.K_w]:
            pitch_wrench_mapped = -max_range

        if keys[pygame.K_d]:
            yaw_wrench_mapped = 2
        elif keys[pygame.K_a]:
            yaw_wrench_mapped = -2

        # Adjust max range with keyboard input
        if keys[pygame.K_1]:
            max_range += 1
            if max_range > 10:
                max_range = 10
        elif keys[pygame.K_0]:
            max_range -= 1
            if max_range < 0:
                max_range = 0

        # Clear the screen
        screen.fill((255, 255, 255))
        text_print.reset()
        text_print.tprint(screen, f"Wrench", 250)
        text_print.new_line()
        
        # Print the wrench values
        text_print.tprint(screen, f"X: {x_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Y: {y_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Z: {z_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Roll: {roll_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Pitch: {pitch_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Yaw: {yaw_wrench_mapped:.3f}")
        text_print.tprint(screen, f"Wrench Range: {max_range}")
        text_print.new_line()
        
        # Publish the wrench values
        vector_msg = WrenchStamped()
        vector_msg.wrench.force.x = x_wrench_mapped
        vector_msg.wrench.force.y = y_wrench_mapped
        vector_msg.wrench.force.z = z_wrench_mapped
        vector_msg.wrench.torque.x = roll_wrench_mapped
        vector_msg.wrench.torque.y = pitch_wrench_mapped
        vector_msg.wrench.torque.z = yaw_wrench_mapped
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

