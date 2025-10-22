import sys
import json
import serial
import threading
import pygame
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
)
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BaseStationNode(Node):
    def __init__(self, serial_port="/dev/ttyACM0", baud=115200):
        super().__init__("base_station_node")

        # --- Servo Command Publisher ---
        self.servo_pub = self.create_publisher(String, "servo_cmds", 10)

        # --- Sensor Data Subscriber ---
        self.sensor_sub = self.create_subscription(
            String, "sensor_data", self.sensor_callback, 10
        )

        # --- Serial connection ---
        self.serial = serial.Serial(serial_port, baud, timeout=1)

        # --- Sensor data buffer ---
        self.latest_data = {"ax": 0, "ay": 0, "az": 0, "temp": 0, "alt": 0, "pres": 0}

        # --- Start serial thread ---
        threading.Thread(target=self.read_serial_loop, daemon=True).start()

        # --- Start Pygame thread for keyboard control ---
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        # --- Servo angles ---
        self.shoulder = 90
        self.elbow = 90
        self.wrist = 90

    # ========== SERIAL READER ==========
    def read_serial_loop(self):
        while True:
            try:
                line = self.serial.readline().decode().strip()
                if not line:
                    continue
                # Expect JSON from Pico like: {"ax":0.1,"ay":0.2,"az":9.8,"temp":23.4,"alt":150.2,"pres":1008.1}
                data = json.loads(line)
                self.latest_data.update(data)

                # Publish to ROS2
                msg = String()
                msg.data = json.dumps(data)
                self.sensor_sub.callback(msg)  # update GUI directly too
                self.get_logger().info(f"Sensor data: {msg.data}")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    # ========== SENSOR DATA CALLBACK ==========
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_data.update(data)
        except Exception as e:
            self.get_logger().error(f"Sensor parse error: {e}")

    # ========== KEYBOARD INPUT ==========
    def keyboard_loop(self):
        pygame.init()
        pygame.display.set_mode((200, 200))
        clock = pygame.time.Clock()

        while True:
            keys = pygame.key.get_pressed()
            changed = False

            # Shoulder control: W/S
            if keys[pygame.K_w]:
                self.shoulder = min(self.shoulder + 1, 180)
                changed = True
            elif keys[pygame.K_s]:
                self.shoulder = max(self.shoulder - 1, 0)
                changed = True

            # Elbow control: E/D
            if keys[pygame.K_e]:
                self.elbow = min(self.elbow + 1, 180)
                changed = True
            elif keys[pygame.K_d]:
                self.elbow = max(self.elbow - 1, 0)
                changed = True

            # Wrist control: R/F
            if keys[pygame.K_r]:
                self.wrist = min(self.wrist + 1, 180)
                changed = True
            elif keys[pygame.K_f]:
                self.wrist = max(self.wrist - 1, 0)
                changed = True

            if changed:
                cmd = f"W:{self.wrist},S:{self.shoulder},E:{self.elbow}"
                self.serial.write((cmd + "\n").encode())
                msg = String()
                msg.data = cmd
                self.servo_pub.publish(msg)
                self.get_logger().info(f"Sent servo command: {cmd}")

            pygame.event.pump()
            clock.tick(30)


# ===================== GUI =====================

class SensorGUI(QMainWindow):
    def __init__(self, node: BaseStationNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("ABSTAIN Base Station")
        self.setGeometry(200, 200, 800, 600)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.label = QLabel("Sensor Data", alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label)

        self.plot = pg.PlotWidget(title="Acceleration (m/s²)")
        self.plot.addLegend()
        self.ax_curve = self.plot.plot(pen="r", name="ax")
        self.ay_curve = self.plot.plot(pen="g", name="ay")
        self.az_curve = self.plot.plot(pen="b", name="az")
        layout.addWidget(self.plot)

        self.ax_data, self.ay_data, self.az_data = [], [], []

        # Timer to refresh plots
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        d = self.node.latest_data
        self.label.setText(
            f"Accel: ({d['ax']:.2f}, {d['ay']:.2f}, {d['az']:.2f}) | "
            f"Temp: {d['temp']:.2f}°C | Alt: {d['alt']:.2f}m | Pres: {d['pres']:.2f}hPa"
        )

        self.ax_data.append(d["ax"])
        self.ay_data.append(d["ay"])
        self.az_data.append(d["az"])
        if len(self.ax_data) > 200:
            self.ax_data.pop(0)
            self.ay_data.pop(0)
            self.az_data.pop(0)

        self.ax_curve.setData(self.ax_data)
        self.ay_curve.setData(self.ay_data)
        self.az_curve.setData(self.az_data)


# ===================== MAIN =====================

def main():
    rclpy.init()
    node = BaseStationNode(serial_port="/dev/ttyACM0")

    app = QApplication(sys.argv)
    gui = SensorGUI(node)
    gui.show()

    # ROS2 spin thread
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
