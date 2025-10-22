import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pygame

class ArmKeyboardController(Node):
    def __init__(self):
        super().__init__('arm_keyboard_controller')

        # Servo publishers
        self.shoulder_pub = self.create_publisher(Float32, 'servo_shoulder', 10)
        self.elbow_pub = self.create_publisher(Float32, 'servo_elbow', 10)
        self.wrist_pub = self.create_publisher(Float32, 'servo_wrist', 10)

        # DC motor publishers
        self.axis_pub = self.create_publisher(Float32, 'motor_axis', 10)
        self.claw_pub = self.create_publisher(Float32, 'motor_claw', 10)

        # Initialize pygame
        pygame.init()
        pygame.display.set_mode((400, 120))
        pygame.display.set_caption("Arm Keyboard Control")

        # Servo angles
        self.shoulder_angle = 90.0
        self.elbow_angle = 90.0
        self.wrist_angle = 90.0

        # Motor speeds
        self.axis_speed = 0.0
        self.claw_speed = 0.0

        # Step size
        self.step = 1.0           # degrees per tick for servos
        self.motor_step = 30.0    # speed step for motors (out of 255)

        # Timer (30 Hz)
        self.timer = self.create_timer(1/30.0, self.update)

    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting keyboard control")
                rclpy.shutdown()
                return

        keys = pygame.key.get_pressed()

        # ----- Servo control -----
        if keys[pygame.K_w]:
            self.shoulder_angle += self.step
        elif keys[pygame.K_s]:
            self.shoulder_angle -= self.step

        if keys[pygame.K_e]:
            self.elbow_angle += self.step
        elif keys[pygame.K_d]:
            self.elbow_angle -= self.step

        if keys[pygame.K_r]:
            self.wrist_angle += self.step
        elif keys[pygame.K_f]:
            self.wrist_angle -= self.step

        # Clamp servo ranges
        self.shoulder_angle = max(0.0, min(180.0, self.shoulder_angle))
        self.elbow_angle = max(0.0, min(180.0, self.elbow_angle))
        self.wrist_angle = max(0.0, min(180.0, self.wrist_angle))

        # ----- Motor control -----
        if keys[pygame.K_LEFT]:
            self.axis_speed = -150.0  # Rotate base left
        elif keys[pygame.K_RIGHT]:
            self.axis_speed = 150.0   # Rotate base right
        else:
            self.axis_speed = 0.0

        if keys[pygame.K_UP]:
            self.claw_speed = 150.0   # Open claw
        elif keys[pygame.K_DOWN]:
            self.claw_speed = -150.0  # Close claw
        else:
            self.claw_speed = 0.0

        # Publish servo angles
        self.shoulder_pub.publish(Float32(data=self.shoulder_angle))
        self.elbow_pub.publish(Float32(data=self.elbow_angle))
        self.wrist_pub.publish(Float32(data=self.wrist_angle))

        # Publish motor speeds
        self.axis_pub.publish(Float32(data=self.axis_speed))
        self.claw_pub.publish(Float32(data=self.claw_speed))

        # Optional logging
        self.get_logger().info(
            f"S:{self.shoulder_angle:.1f} E:{self.elbow_angle:.1f} W:{self.wrist_angle:.1f} "
            f"A:{self.axis_speed:.0f} C:{self.claw_speed:.0f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardController()
    rclpy.spin(node)
    pygame.quit()

if __name__ == '__main__':
    main()
