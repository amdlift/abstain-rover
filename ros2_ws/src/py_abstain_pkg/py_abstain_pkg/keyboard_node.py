import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pygame
import math

class ArmKeyboardController(Node):
    def __init__(self):
        super().__init__('arm_keyboard_controller')

        # --- ROS2 Publishers ---
        self.shoulder_pub = self.create_publisher(Float32, 'servo_shoulder', 10)
        self.elbow_pub = self.create_publisher(Float32, 'servo_elbow', 10)
        self.wrist_pub = self.create_publisher(Float32, 'servo_wrist', 10)
        self.axis_pub = self.create_publisher(Float32, 'motor_axis', 10)
        self.claw_pub = self.create_publisher(Float32, 'motor_claw', 10)

        # --- Pygame Setup ---
        pygame.init()
        pygame.display.set_mode((400, 120))
        pygame.display.set_caption("Arm Keyboard Control")

        # --- Servo Angles ---
        self.shoulder_angle = 135.0   # Center position for 270° servo
        self.elbow_angle = 90.0       # Neutral
        self.wrist_angle = 90.0       # Neutral

        # --- Motor Speeds ---
        self.axis_speed = 0.0
        self.claw_speed = 0.0

        # --- Movement Step Sizes ---
        self.step = 1.0           # degrees per tick for manual servo control
        self.motor_step = 30.0    # speed step for motors
        self.cartesian_step = 1.0 # cm or arbitrary units for IK control

        # --- Arm Dimensions (for IK) ---
        self.brachium = 10.0
        self.forearm = 10.0

        # --- Initial Cartesian Position (from current angles) ---
        self.x, self.y = self.forward_kinematics(
            self.shoulder_angle, self.elbow_angle, self.brachium, self.forearm
        )

        # --- ROS2 Update Timer (30 Hz) ---
        self.timer = self.create_timer(1/30.0, self.update)

    # -------------------------------------------------------------
    # MAIN UPDATE LOOP
    # -------------------------------------------------------------
    def update(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Exiting keyboard control")
                rclpy.shutdown()
                return

        keys = pygame.key.get_pressed()

        # ---------------------------
        # MANUAL SERVO CONTROL (W/S/E/D/R/F)
        # ---------------------------
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

        # Clamp angles
        self.shoulder_angle = max(0.0, min(270.0, self.shoulder_angle))
        self.elbow_angle = max(0.0, min(180.0, self.elbow_angle))
        self.wrist_angle = max(0.0, min(180.0, self.wrist_angle))

        # ---------------------------
        # INVERSE KINEMATICS CONTROL (I/J/K/L)
        # ---------------------------
        moved = False
        if keys[pygame.K_i]:
            self.y += self.cartesian_step
            moved = True
        elif keys[pygame.K_k]:
            self.y -= self.cartesian_step
            moved = True

        if keys[pygame.K_j]:
            self.x -= self.cartesian_step
            moved = True
        elif keys[pygame.K_l]:
            self.x += self.cartesian_step
            moved = True

        if moved:
            try:
                theta1, theta2 = self.inverse_kinematics(
                    self.x, self.y, self.brachium, self.forearm
                )

                # Shoulder is a 270° servo: neutral = 135°, +θ1 = up, -θ1 = down
                self.shoulder_angle = 135.0 + theta1
                self.elbow_angle = 90.0 + theta2

                # Clamp to safe servo limits
                self.shoulder_angle = max(0.0, min(270.0, self.shoulder_angle))
                self.elbow_angle = max(0.0, min(180.0, self.elbow_angle))

            except ValueError as e:
                self.get_logger().warn(f"IK out of reach: {e}")

        # ---------------------------
        # MOTOR CONTROL (Arrow Keys)
        # ---------------------------
        if keys[pygame.K_LEFT]:
            self.axis_speed = -150.0
        elif keys[pygame.K_RIGHT]:
            self.axis_speed = 150.0
        else:
            self.axis_speed = 0.0

        if keys[pygame.K_UP]:
            self.claw_speed = 150.0
        elif keys[pygame.K_DOWN]:
            self.claw_speed = -150.0
        else:
            self.claw_speed = 0.0

        # ---------------------------
        # PUBLISH COMMANDS
        # ---------------------------
        self.shoulder_pub.publish(Float32(data=self.shoulder_angle))
        self.elbow_pub.publish(Float32(data=self.elbow_angle))
        self.wrist_pub.publish(Float32(data=self.wrist_angle))
        self.axis_pub.publish(Float32(data=self.axis_speed))
        self.claw_pub.publish(Float32(data=self.claw_speed))

        # Update Cartesian position from FK for display/log
        self.x, self.y = self.forward_kinematics(
            self.shoulder_angle, self.elbow_angle,
            self.brachium, self.forearm
        )

        self.get_logger().info(
            f"X:{self.x:.1f} Y:{self.y:.1f} | "
            f"S:{self.shoulder_angle:.1f}° E:{self.elbow_angle:.1f}° W:{self.wrist_angle:.1f}° | "
            f"A:{self.axis_speed:.0f} C:{self.claw_speed:.0f}"
        )

    # -------------------------------------------------------------
    # INVERSE KINEMATICS FUNCTION
    # -------------------------------------------------------------
    def inverse_kinematics(self, x, y, L1, L2):
        r = math.sqrt(x**2 + y**2)
        if r > L1 + L2 or r < abs(L1 - L2):
            raise ValueError("Target point out of reach")

        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = math.acos(cos_theta2)
        theta1 = math.atan2(y, x) - math.atan2(
            L2 * math.sin(theta2), L1 + L2 * math.cos(theta2)
        )

        return math.degrees(theta1), math.degrees(theta2)

    # -------------------------------------------------------------
    # FORWARD KINEMATICS FUNCTION
    # -------------------------------------------------------------
    def forward_kinematics(self, theta1_deg, theta2_deg, L1, L2):
        # Convert servo angles back into math-space
        # Shoulder: 135° = 0° math, elbow: 90° = 0° math
        theta1 = math.radians(theta1_deg - 135.0)
        theta2 = math.radians(theta2_deg - 90.0)

        x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        return x, y


# -------------------------------------------------------------
# MAIN ENTRY POINT
# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardController()
    rclpy.spin(node)
    pygame.quit()

if __name__ == '__main__':
    main()
