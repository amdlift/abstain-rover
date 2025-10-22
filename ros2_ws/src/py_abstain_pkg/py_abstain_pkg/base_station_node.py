import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from collections import deque

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QSlider, QHBoxLayout, QTabWidget
)
from PyQt6.QtCore import Qt, QTimer
import pyqtgraph as pg


class BaseStationNode(Node):
    def __init__(self):
        super().__init__('base_station_node')
        self.servo_pub = self.create_publisher(String, 'servo_cmds', 10)
        self.sensor_sub = self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)

        # Current servo angles
        self.servo_angles = {'shoulder': 90, 'elbow': 90, 'wrist': 90}
        self.gui = None  # Will be set by GUI

    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if self.gui:
                self.gui.update_sensor_display(data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Failed to decode JSON: {msg.data}")

    def publish_servo_angles(self):
        msg = String()
        msg.data = f"W:{self.servo_angles['wrist']},S:{self.servo_angles['shoulder']},E:{self.servo_angles['elbow']}"
        self.servo_pub.publish(msg)


class BaseStationGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        node.gui = self

        self.setWindowTitle("Rover Base Station")

        # Sensor display label
        self.sensor_label = QLabel("Waiting for sensor data...")

        # Sliders
        self.sliders = {}
        slider_layout = QVBoxLayout()
        for joint in ['shoulder', 'elbow', 'wrist']:
            layout = QHBoxLayout()
            label = QLabel(joint.capitalize())
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(node.servo_angles[joint])
            slider.valueChanged.connect(self.make_slider_callback(joint))
            layout.addWidget(label)
            layout.addWidget(slider)
            slider_layout.addLayout(layout)
            self.sliders[joint] = slider

        # Graphs
        self.graph_tabs = QTabWidget()
        self.graph_widgets = {}

        # Acceleration graph
        accel_widget = pg.PlotWidget(title="Acceleration")
        accel_widget.addLegend()
        accel_widget.setLabel('left', 'm/s²')
        accel_widget.setLabel('bottom', 'Samples')
        self.accel_curves = {
            'ax': accel_widget.plot(pen='r', name='ax'),
            'ay': accel_widget.plot(pen='g', name='ay'),
            'az': accel_widget.plot(pen='b', name='az')
        }
        self.accel_data = {k: deque(maxlen=200) for k in ['ax','ay','az']}
        self.graph_tabs.addTab(accel_widget, "Acceleration")

        # Altitude graph
        alt_widget = pg.PlotWidget(title="Altitude")
        alt_widget.setLabel('left', 'm')
        alt_widget.setLabel('bottom', 'Samples')
        self.alt_curve = alt_widget.plot(pen='y')
        self.alt_data = deque(maxlen=200)
        self.graph_tabs.addTab(alt_widget, "Altitude")

        # Temperature graph
        temp_widget = pg.PlotWidget(title="Temperature")
        temp_widget.setLabel('left', '°C')
        temp_widget.setLabel('bottom', 'Samples')
        self.temp_curve = temp_widget.plot(pen='c')
        self.temp_data = deque(maxlen=200)
        self.graph_tabs.addTab(temp_widget, "Temperature")

        # Pressure graph
        pres_widget = pg.PlotWidget(title="Pressure")
        pres_widget.setLabel('left', 'hPa')
        pres_widget.setLabel('bottom', 'Samples')
        self.pres_curve = pres_widget.plot(pen='m')
        self.pres_data = deque(maxlen=200)
        self.graph_tabs.addTab(pres_widget, "Pressure")

        self.graph_widgets = {
            'accel': self.accel_curves,
            'alt': self.alt_curve,
            'temp': self.temp_curve,
            'pres': self.pres_curve
        }

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.sensor_label)
        main_layout.addLayout(slider_layout)
        main_layout.addWidget(self.graph_tabs)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Timer to refresh graphs
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.start(50)  # 20 Hz

    def make_slider_callback(self, joint):
        def callback(value):
            self.node.servo_angles[joint] = value
            self.node.publish_servo_angles()
        return callback

    def update_sensor_display(self, data):
        # Update label
        text = (
            f"Acceleration:\n"
            f"  x: {data.get('ax',0):.2f}, y: {data.get('ay',0):.2f}, z: {data.get('az',0):.2f}\n"
            f"Temperature: {data.get('temp',0):.2f} C\n"
            f"Altitude: {data.get('alt',0):.2f} m\n"
            f"Pressure: {data.get('pres',0):.2f} hPa"
        )
        self.sensor_label.setText(text)

        # Append data for graphs
        for k in ['ax','ay','az']:
            self.accel_data[k].append(data.get(k,0))
        self.alt_data.append(data.get('alt',0))
        self.temp_data.append(data.get('temp',0))
        self.pres_data.append(data.get('pres',0))

    def update_graphs(self):
        # Acceleration
        for k, curve in self.accel_curves.items():
            curve.setData(list(self.accel_data[k]))

        # Altitude
        self.alt_curve.setData(list(self.alt_data))
        # Temperature
        self.temp_curve.setData(list(self.temp_data))
        # Pressure
        self.pres_curve.setData(list(self.pres_data))


def main(args=None):
    # Inside your main() function
    rclpy.init(args=args)
    node = BaseStationNode()

    app = QApplication(sys.argv)
    gui = BaseStationGUI(node)
    gui.show()

    # QTimer to spin ROS2
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(10)  # spin every 10 ms

    sys.exit(app.exec())  # start Qt event loop



if __name__ == "__main__":
    main()
