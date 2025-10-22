import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pygame

class ArmKeyboardController(Node):
    def __init__(self):
        super().__init__('arm_keyboard_controller')

        # Publishers for each servo topic
        self.shoulder_pub = self.create_publisher(Float32, 'servo_shoulder', 10)
        self.elbow_pub = self.create_publisher(Float32, 'servo_elbow', 10)
        self.wrist_pub = self.create_publisher(Float32, 'servo_wrist', 10)

        # Initialize pygame window
        pygame.init()
        pygame.display.set_mode((300, 100))
        pygame.display.set_caption("Arm Keyboard Control")

        # Initial angles (degrees)
        self.shoulder_angle = 90.0
        self.elbow_angle = 90.0
        self.wrist_angle = 90.0

        # Step per tick (degrees)
        self.step = 1.0

        # Timer loop at 30 Hz
        self.timer = self.create_timer(1/30.0, self.update)

    def update(self):
        # Keep window responsive
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting keyboard control")
                rclpy.shutdown()
                return

        keys = pygame.key.get_pressed()

        # Shoulder control: W/S
        if keys[pygame.K_w]:
            self.shoulder_angle += self.step
        elif keys[pygame.K_s]:
            self.shoulder_angle -= self.step

        # Elbow control: E/D
        if keys[pygame.K_e]:
            self.elbow_angle += self.step
        elif keys[pygame.K_d]:
            self.elbow_angle -= self.step

        # Wrist control: R/F
        if keys[pygame.K_r]:
            self.wrist_angle += self.step
        elif keys[pygame.K_f]:
            self.wrist_angle -= self.step

        # Clamp to servo range
        self.shoulder_angle = max(0.0, min(180.0, self.shoulder_angle))
        self.elbow_angle = max(0.0, min(180.0, self.elbow_angle))
        self.wrist_angle = max(0.0, min(180.0, self.wrist_angle))

        # Publish each joint
        self.shoulder_pub.publish(Float32(data=self.shoulder_angle))
        self.elbow_pub.publish(Float32(data=self.elbow_angle))
        self.wrist_pub.publish(Float32(data=self.wrist_angle))

        # Optional logging every few frames
        self.get_logger().info(
            f"Shoulder: {self.shoulder_angle:.1f}, Elbow: {self.elbow_angle:.1f}, Wrist: {self.wrist_angle:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardController()
    rclpy.spin(node)
    pygame.quit()

if __name__ == '__main__':
    main()
