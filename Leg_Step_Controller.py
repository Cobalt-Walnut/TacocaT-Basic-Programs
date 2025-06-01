import pygame
import can
import time
import math
import struct
from Leg_Movement_Functions import leg_step, initialize_can_bus, enable_motor_mode, disable_motor_mode, set_zero_position, send_motor_command

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Leg Control")

# Initialize CAN bus
can_bus = initialize_can_bus('can1')

# Set up motors
motor1_can_id = 0x001
motor2_can_id = 0x002
motor3_can_id = 0x003

# Motor setup function
def setup_motors():
    disable_motor_mode(can_bus, motor1_can_id)
    disable_motor_mode(can_bus, motor2_can_id)
    disable_motor_mode(can_bus, motor3_can_id)
    time.sleep(1)

    set_zero_position(can_bus, motor1_can_id)
    set_zero_position(can_bus, motor2_can_id)
    set_zero_position(can_bus, motor3_can_id)
    print("Motors Zeroed")
    time.sleep(1)

    enable_motor_mode(can_bus, motor1_can_id)
    enable_motor_mode(can_bus, motor2_can_id)
    enable_motor_mode(can_bus, motor3_can_id)
    print("Motors On")
    time.sleep(1)

    # Calibrate motors
    send_motor_command(can_bus, motor1_can_id, -30)
    send_motor_command(can_bus, motor2_can_id, 0)
    send_motor_command(can_bus, motor3_can_id, 55)
    time.sleep(1)

    #set_zero_position(can_bus, motor1_can_id)
    set_zero_position(can_bus, motor2_can_id)
    set_zero_position(can_bus, motor3_can_id)
    print("Motors Calibrated")
    time.sleep(2)

    print("Ready")
    time.sleep(1)

# Set up motors
setup_motors()

# Create button class
class Button:
    def __init__(self, x, y, width, height, text, color, text_color):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.text_color = text_color

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)
        font = pygame.font.Font(None, 36)
        text = font.render(self.text, True, self.text_color)
        text_rect = text.get_rect(center=self.rect.center)
        surface.blit(text, text_rect)

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

# Create buttons
stop_button = Button(10, 200, 100, 50, "STOP", (255, 0, 0), (255, 255, 255))
close_button = Button(290, 200, 100, 50, "CLOSE", (0, 0, 255), (255, 255, 255))

# Main game loop
running = True
font = pygame.font.Font(None, 36)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                print("Forward Step")
                leg_step(thigh_can_id=motor1_can_id, knee_can_id=motor2_can_id, canbus=can_bus, duration=2.0, direction=1, center_y=-268)
            elif event.key == pygame.K_DOWN:
                print("Backward Step")
                leg_step(thigh_can_id=motor1_can_id, knee_can_id=motor2_can_id, canbus=can_bus, duration=2.0, direction=-1, center_y=-268)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if stop_button.is_clicked(event.pos):
                print("STOP button pressed. Stopping Motors.")
                send_motor_command(can_bus, motor1_can_id, 0)
                send_motor_command(can_bus, motor2_can_id, -55)
                time.sleep(3)
            elif close_button.is_clicked(event.pos):
                running = False

    # Clear the screen
    screen.fill((255, 255, 255))

    # Display instructions
    text = font.render("Up Arrow: Forward Step", True, (0, 0, 0))
    screen.blit(text, (10, 50))
    text = font.render("Down Arrow: Backward Step", True, (0, 0, 0))
    screen.blit(text, (10, 100))

    # Draw buttons
    stop_button.draw(screen)
    close_button.draw(screen)

    pygame.display.flip()

# Clean up
send_motor_command(can_bus, motor1_can_id, 0)
send_motor_command(can_bus, motor2_can_id, 0)   # Stop motor on interrupt
send_motor_command(can_bus, motor3_can_id, -55)   # Stop motor on interrupt
time.sleep(3)
disable_motor_mode(can_bus, motor1_can_id)
disable_motor_mode(can_bus, motor2_can_id)
disable_motor_mode(can_bus, motor3_can_id)
can_bus.shutdown()
pygame.quit()
