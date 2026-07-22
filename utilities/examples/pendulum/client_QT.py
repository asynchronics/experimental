import sys
import time
import timeit
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg

from nexosim import Simulation
from nexosim.time import Duration


class SimulationPlotter(QMainWindow):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.current_time_s = 0.0

        # Define the simulation step duration (16 ms)
        self.step_dt_s = 0.016
        self.step_duration = Duration.create(microseconds=int(self.step_dt_s * 1e6))
        self.last_real_time = None

        # Array to store time and position data for plotting
        self.times = []
        self.positions = []

        # Qt window setup
        self.setWindowTitle("Real-time Pendulum Position Plot")
        self.resize(800, 550)

        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Start/Pause button
        self.is_running = True
        self.btn_toggle = QPushButton("Pause Simulation (Space)")
        self.btn_toggle.setStyleSheet(
            "font-size: 14px; padding: 8px; font-weight: bold; background-color: #f44336; color: white;"
        )
        self.btn_toggle.clicked.connect(self.toggle_simulation)
        self.btn_toggle.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        layout.addWidget(self.btn_toggle)

        # Plot widget setup
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")

        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)

        self.plot_widget.setTitle("Real-time Pendulum Position Plot", size="14pt")
        self.plot_widget.setLabel("bottom", "Simulation Time [s]")
        self.plot_widget.setLabel("left", "Position [rad]")
        self.plot_widget.showGrid(x=True, y=True)

        self.curve = self.plot_widget.plot(pen=pg.mkPen("b", width=2))

        # QTimer setup for periodic simulation stepping
        self.timer = QTimer()
        self.timer.timeout.connect(self.step_and_pos)
        # Start the timer with the defined step duration in milliseconds
        self.timer.start(int(self.step_dt_s * 1000))

    def keyPressEvent(self, event):
        # Handle key press events for controlling the simulation.
        # Space: Toggle simulation start/pause.
        # Up Arrow: Increase setpoint by 10.0.
        # Down Arrow: Decrease setpoint by 10.0.
        if event.key() == Qt.Key.Key_Space:
            self.toggle_simulation()
        elif event.key() == Qt.Key.Key_Up:
            current_setpoint = self.sim.try_read_events("setpoint")
            new_setpoint = (
                current_setpoint[-1] + 10.0
                if current_setpoint
                else 10.0
            )
            self.sim.process_event("setpoint", new_setpoint)
        elif event.key() == Qt.Key.Key_Down:
            current_setpoint = self.sim.try_read_events("setpoint")
            new_setpoint = (
                current_setpoint[-1] - 10.0
                if current_setpoint
                else -10.0
            )
            self.sim.process_event("setpoint", new_setpoint)
        else:
            super().keyPressEvent(event)

    def toggle_simulation(self):
        # Toggle the simulation state between running and paused.
        if self.is_running:
            self.timer.stop()
            self.last_real_time = None
            self.btn_toggle.setText("Start Simulation (Space)")
            self.btn_toggle.setStyleSheet(
                "font-size: 14px; padding: 8px; font-weight: bold; background-color: #4CAF50; color: white;"
            )
            self.is_running = False
        else:
            self.timer.start(int(self.step_dt_s * 1000))
            self.btn_toggle.setText("Pause Simulation (Space)")
            self.btn_toggle.setStyleSheet(
                "font-size: 14px; padding: 8px; font-weight: bold; background-color: #f44336; color: white;"
            )
            self.is_running = True

    def step_and_pos(self):
        now = time.perf_counter()

        # If this is the first call, initialize last_real_time and return without stepping the simulation. 
        if self.last_real_time is None:
            self.last_real_time = now
            return

        # Calculate the elapsed real time since the last simulation step
        real_elapsed = now - self.last_real_time
        self.last_real_time = now

        # Step the simulation by the elapsed real time.
        step_duration = Duration.create(microseconds=int(real_elapsed * 1e6))
        self.sim.step_until(step_duration)

        # Update the current simulation time based on the real elapsed time.
        self.current_time_s += real_elapsed

        # Read events.
        pos_events = self.sim.try_read_events("position")
        # If there are position events, take the latest one and update the plot.
        if pos_events:
            latest_pos = pos_events[-1]
            self.times.append(self.current_time_s)
            self.positions.append(latest_pos)
            self.curve.setData(self.times, self.positions)


def main():
    app = QApplication(sys.argv)

    with Simulation("0.0.0.0:41633") as sim:
        sim.build()
        sim.init()
        sim.process_event("setpoint", 90.0)

        print("Initial position events:", sim.try_read_events("position"))

        window = SimulationPlotter(sim)
        window.show()

        sys.exit(app.exec())


if __name__ == "__main__":
    main()