import pygame

# Initialize Pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()

# Check how many joysticks are connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    exit()

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to: {joystick.get_name()}")

# Main loop to read input
running = True
while running:
    pygame.event.pump()  # Process events

    # Read left joystick axes
    left_x = joystick.get_axis(0)  # Left stick X-axis
    left_y = joystick.get_axis(1)  # Left stick Y-axis (inverted)

    print(f"Left Stick: ({left_x:.2f}, {left_y:.2f})")

    # Exit if the 'back' button (usually button index 6) is pressed
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.JOYBUTTONDOWN and event.button == 6):
            running = False

# Cleanup
joystick.quit()
pygame.quit()