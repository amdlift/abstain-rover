from PyQt6.QtWidgets import QApplication, QWidget, QMainWindow, QPushButton, QGridLayout, QLabel, QVBoxLayout, QHBoxLayout
from PyQt6.QtCore import QPointF, QRectF, QLineF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPainter, QColor
import pyqtgraph as pg
from enum import Enum

import sys
import time
import random

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3
    CENTER = 4  # when joystick is not moved

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Abstain Base Station")
        joystick = Joystick()
        hstick = HorizontalJoystick()
        sensorData = SensorDisplay()
        sensorGraph = SensorGraph()
        grip_control = GripControl()

        controls = QVBoxLayout()
        controls.addWidget(joystick)
        controls.addWidget(hstick)
        controls.addWidget(grip_control)
        controls_widget = QWidget()
        controls_widget.setLayout(controls)

        layout = QGridLayout()

        layout.addWidget(controls_widget, 0, 0)
        layout.addWidget(sensorData, 0, 1)
        layout.addWidget(sensorGraph, 1, 1)

        self.setMinimumSize(400,300)

        widget = QWidget()
        widget.setLayout(layout)



        # Set the central widget of the Window.
        self.setCentralWidget(widget)

class Joystick(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.__maxDistance = 50
        self.knobPos = QPointF(0, 0)
        self.dragging = False

    def paintEvent(self, event):
        painter = QPainter(self)
        center = self.center()
        # Draw base
        painter.setPen(Qt.GlobalColor.black)
        painter.setBrush(QColor(220, 220, 220))
        baseRect = QRectF(-self.__maxDistance, -self.__maxDistance,
                          self.__maxDistance * 2, self.__maxDistance * 2).translated(center)
        painter.drawEllipse(baseRect)
        # Draw knob
        painter.setBrush(Qt.GlobalColor.black)
        knobRect = QRectF(-15, -15, 30, 30).translated(center + self.knobPos)
        painter.drawEllipse(knobRect)

    def center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def mousePressEvent(self, event):
        center = self.center()
        knobRect = QRectF(-15, -15, 30, 30).translated(center + self.knobPos)
        if knobRect.contains(event.position()):
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            center = self.center()
            delta = event.position() - center
            # Limit movement to max distance
            line = QLineF(QPointF(0, 0), delta)
            if line.length() > self.__maxDistance:
                line.setLength(self.__maxDistance)
            self.knobPos = line.p2()
            self.update()
            print(self.getDirection())

    def mouseReleaseEvent(self, event):
        self.dragging = False
        self.knobPos = QPointF(0, 0)
        self.update()

    def getDirection(self):
        # Returns tuple: (Direction, normalized distance 0-1)
        if self.knobPos == QPointF(0, 0):
            return (Direction.CENTER, 0.0)
        angle = QLineF(QPointF(0, 0), self.knobPos).angle()
        distance = min(QLineF(QPointF(0, 0), self.knobPos).length() / self.__maxDistance, 1.0)
        # Angle in PyQt6: 0 is east, counter-clockwise
        if 45 <= angle < 135:
            return (Direction.UP, distance)
        elif 135 <= angle < 225:
            return (Direction.LEFT, distance)
        elif 225 <= angle < 315:
            return (Direction.DOWN, distance)
        return (Direction.RIGHT, distance)

class HorizontalJoystick(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 60)  # wider than tall
        self.__maxDistance = 80  # horizontal range
        self.knobX = 0.0  # horizontal offset from center
        self.dragging = False

    def paintEvent(self, event):
        painter = QPainter(self)
        center = self.center()
        # Draw base line
        painter.setPen(Qt.GlobalColor.black)
        painter.setBrush(QColor(220, 220, 220))
        baseRect = QRectF(-self.__maxDistance, -10, self.__maxDistance * 2, 20).translated(center)
        painter.drawRoundedRect(baseRect, 10, 10)
        # Draw knob
        painter.setBrush(Qt.GlobalColor.black)
        knobRect = QRectF(-15, -20, 30, 40).translated(center + QPointF(self.knobX, 0))
        painter.drawEllipse(knobRect)

    def center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def mousePressEvent(self, event):
        center = self.center()
        knobRect = QRectF(-15, -20, 30, 40).translated(center + QPointF(self.knobX, 0))
        if knobRect.contains(event.position()):
            self.dragging = True

    def mouseMoveEvent(self, event):
        if self.dragging:
            centerX = self.center().x()
            x = event.position().x() - centerX
            # Clamp to range
            self.knobX = max(-self.__maxDistance, min(self.__maxDistance, x))
            self.update()
            print("Value:", self.value())  # debug output

    def mouseReleaseEvent(self, event):
        self.dragging = False
        self.knobX = 0.0  # snap back to center
        self.update()

    def value(self):
        # Return normalized horizontal value: -1.0 (left) -> 1.0 (right)
        return self.knobX / self.__maxDistance

class GripControl(QWidget):
    # Signals to notify button presses
    openClicked = pyqtSignal()
    closeClicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # Create buttons
        self.open_button = QPushButton("Open")
        self.close_button = QPushButton("Close")

        # Connect buttons to signals
        self.open_button.clicked.connect(self.openClicked.emit)
        self.close_button.clicked.connect(self.closeClicked.emit)

        # Layout: horizontal side-by-side
        layout = QHBoxLayout()
        layout.addWidget(self.open_button)
        layout.addWidget(self.close_button)
        self.setLayout(layout)  

class SensorDisplay(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Data")

        # Create labels for each sensor
        self.temp_label = QLabel("Temperature: -- °C")
        self.press_label = QLabel("Pressure: -- hPa")
        self.accel_label = QLabel("Acceleration: -- m/s²")
        self.alt_label = QLabel("Altitude: -- m")

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.temp_label)
        layout.addWidget(self.press_label)
        layout.addWidget(self.accel_label)
        layout.addWidget(self.alt_label)
        self.setLayout(layout)

        # Timer to update data every second
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000)  # 1000 ms = 1 sec

    def update_data(self):
        # Replace these random values with your real sensor readings
        temperature = round(random.uniform(20, 25), 1)
        pressure = round(random.uniform(1000, 1020), 1)
        acceleration = round(random.uniform(0, 1), 2)
        altitude = round(random.uniform(100, 120), 1)

        # Update labels
        self.temp_label.setText(f"Temperature: {temperature} °C")
        self.press_label.setText(f"Pressure: {pressure} hPa")
        self.accel_label.setText(f"Acceleration: {acceleration} m/s²")
        self.alt_label.setText(f"Altitude: {altitude} m")

class SensorGraph(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Sensor Data")
        self.resize(900, 700)

        layout = QGridLayout()
        self.setLayout(layout)

        # Create four plots
        self.temp_plot = pg.PlotWidget(title="Temperature (°C)")
        self.press_plot = pg.PlotWidget(title="Pressure (hPa)")
        self.accel_plot = pg.PlotWidget(title="Acceleration (m/s²)")
        self.alt_plot = pg.PlotWidget(title="Altitude (m)")

        # Add to layout in 2x2 grid
        layout.addWidget(self.temp_plot, 0, 0)
        layout.addWidget(self.press_plot, 0, 1)
        layout.addWidget(self.accel_plot, 1, 0)
        layout.addWidget(self.alt_plot, 1, 1)

        # Initialize data storage
        self.max_points = 100
        self.timestamps = []  # shared x-axis in seconds
        self.temp_data, self.press_data, self.accel_data, self.alt_data = [], [], [], []

        # Create curve objects
        self.temp_curve = self.temp_plot.plot(pen="r")
        self.press_curve = self.press_plot.plot(pen="b")
        self.accel_curve = self.accel_plot.plot(pen="g")
        self.alt_curve = self.alt_plot.plot(pen="y")

        # Start time
        self.start_time = time.time()

        # Timer to simulate sensor updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(200)  # update every 200 ms

    def update_data(self):
        # Current elapsed time (seconds)
        t = time.time() - self.start_time

        # Replace these with your actual sensor readings
        temperature = round(random.uniform(20, 22), 2)
        pressure = round(random.uniform(1000, 1005), 2)
        acceleration = round(random.uniform(0, .5), 2)
        altitude = round(random.uniform(100, 102), 2)

        # Append new data
        self.timestamps.append(t)
        self.temp_data.append(temperature)
        self.press_data.append(pressure)
        self.accel_data.append(acceleration)
        self.alt_data.append(altitude)

        # Keep only last N points
        self.timestamps = self.timestamps[-self.max_points:]
        self.temp_data = self.temp_data[-self.max_points:]
        self.press_data = self.press_data[-self.max_points:]
        self.accel_data = self.accel_data[-self.max_points:]
        self.alt_data = self.alt_data[-self.max_points:]

        # Update curves with shared time axis
        self.temp_curve.setData(self.timestamps, self.temp_data)
        self.press_curve.setData(self.timestamps, self.press_data)
        self.accel_curve.setData(self.timestamps, self.accel_data)
        self.alt_curve.setData(self.timestamps, self.alt_data)

app = QApplication(sys.argv)

# Create a Qt widget, which will be our window.
window = MainWindow()
window.show()  # IMPORTANT!!!!! Windows are hidden by default.

# Start the event loop.
app.exec()

