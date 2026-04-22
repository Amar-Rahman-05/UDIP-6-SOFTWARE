import os
import sys
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QPushButton, QVBoxLayout, QLabel,
    QStackedWidget, QComboBox,
    QHBoxLayout, QSpinBox, QSizePolicy,
    QFrame,
)
from PyQt5.QtCore import Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import UDIP_6_GROUND_Analysis_Sensor as sensor_graphs
import UDIP_6_GROUND_Analysis_Sweep  as sweep_graphs
import UDIP_6_GROUND_Conversion as conversion


# ─────────────────────────────────────────
# MATPLOTLIB CANVAS
# ─────────────────────────────────────────
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


def _make_btn(label: str, height: int = 45) -> QPushButton:
    btn = QPushButton(label)
    btn.setFixedHeight(height)
    btn.setStyleSheet("font-size: 14px;")
    return btn


# ─────────────────────────────────────────
# HOME PAGE
# ─────────────────────────────────────────
class HomePage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        self.main_window = main_window
        self.stack = main_window.stack

        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.folder_path = os.path.join(self.base_dir, "SD-DATA")

        layout = QVBoxLayout()

        title = QLabel("UDIP-6 ANALYSIS")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 28px; font-weight: bold;")

        self.dropdown = QComboBox()
        self.dropdown.setFixedWidth(400)

        self.load_files()

        self.sensorBtn = QPushButton("SENSOR")
        self.sweepBtn = QPushButton("SWEEP")

        for btn in (self.sensorBtn, self.sweepBtn):
            btn.setFixedSize(300, 80)
            btn.setStyleSheet("font-size: 20px;")

        layout.addStretch()
        layout.addWidget(title)
        layout.addSpacing(30)
        layout.addWidget(self.dropdown, alignment=Qt.AlignCenter)
        layout.addSpacing(40)
        layout.addWidget(self.sensorBtn, alignment=Qt.AlignCenter)
        layout.addSpacing(20)
        layout.addWidget(self.sweepBtn, alignment=Qt.AlignCenter)
        layout.addStretch()

        self.setLayout(layout)

        self.sensorBtn.clicked.connect(self._open_sensor)
        self.sweepBtn.clicked.connect(self._open_sweep)

    def load_files(self):
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

        self.dropdown.clear()

        files = [
            f for f in os.listdir(self.folder_path)
            if os.path.isfile(os.path.join(self.folder_path, f))
        ]

        self.dropdown.addItems(files)

    # ─────────────────────────────
    # SINGLE CONVERSION (CACHED)
    # ─────────────────────────────
    def _get_dat_number(self):
        filename = self.dropdown.currentText()
        if not filename:
            return None

        if self.main_window.current_file == filename:
            return self.main_window.current_dat_number

        dat_path = Path(self.folder_path) / filename

        try:
            dat_number = conversion.process_dat_file(dat_path)
        except Exception as e:
            print("[ERROR] Conversion failed:", e)
            return None

        self.main_window.current_file = filename
        self.main_window.current_dat_number = dat_number

        return dat_number

    def _open_sensor(self):
        dat_number = self._get_dat_number()
        if not dat_number:
            return

        self.main_window.sensors.set_dat_number(dat_number)
        self.stack.setCurrentIndex(1)

    def _open_sweep(self):
        dat_number = self._get_dat_number()
        if not dat_number:
            return

        self.main_window.sweep.load_sweeps(dat_number)
        self.stack.setCurrentIndex(2)


# ─────────────────────────────────────────
# SENSOR PAGE (UNCHANGED UI)
# ─────────────────────────────────────────
class SensorsPage(QWidget):

    def __init__(self, stack):
        super().__init__()

        self.dat_number = None
        self.stack = stack

        mainLayout = QHBoxLayout()
        menuLayout = QVBoxLayout()

        title = QLabel("Sensors")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        menuLayout.addWidget(title)

        accelBtn = _make_btn("Accelerometer")
        magBtn = _make_btn("Magnetometer")
        gyroBtn = _make_btn("Gyroscope")
        highAccelBtn = _make_btn("High Range Accel")
        tempBtn = _make_btn("Temperature")
        trajBtn = _make_btn("Flight Trajectory")
        allBtn = _make_btn("Cumulative Graph")

        for btn in (accelBtn, magBtn, gyroBtn, highAccelBtn, tempBtn, trajBtn, allBtn):
            menuLayout.addWidget(btn)

        backBtn = _make_btn("Back", 40)
        menuLayout.addWidget(backBtn)
        menuLayout.addStretch()

        self.canvas = MplCanvas()

        mainLayout.addLayout(menuLayout, 1)
        mainLayout.addWidget(self.canvas, 4)

        self.setLayout(mainLayout)

        accelBtn.clicked.connect(lambda: sensor_graphs.plot_accel(self.canvas))
        magBtn.clicked.connect(lambda: sensor_graphs.plot_mag(self.canvas))
        gyroBtn.clicked.connect(lambda: sensor_graphs.plot_gyro(self.canvas))
        highAccelBtn.clicked.connect(lambda: sensor_graphs.plot_high_accel(self.canvas))
        tempBtn.clicked.connect(lambda: sensor_graphs.plot_temp(self.canvas))
        trajBtn.clicked.connect(lambda: sensor_graphs.plot_trajectory(self.canvas))
        allBtn.clicked.connect(lambda: sensor_graphs.plot_cumulative(self.canvas))

        backBtn.clicked.connect(lambda: self.stack.setCurrentIndex(0))

    def set_dat_number(self, dat_number):
        self.dat_number = dat_number


# ─────────────────────────────────────────
# SWEEP PAGE
# ─────────────────────────────────────────
class SweepPage(QWidget):

    def __init__(self, stack):
        super().__init__()

        self.stack = stack
        self.sweeps = []

        mainLayout = QHBoxLayout()
        menuLayout = QVBoxLayout()

        title = QLabel("Sweep Data")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        menuLayout.addWidget(title)

        self.sweepSelector = QSpinBox()
        self.sweepSelector.setMinimum(0)
        self.sweepSelector.setMaximum(0)
        self.sweepSelector.setSuffix("  (index)")
        menuLayout.addWidget(self.sweepSelector)

        self.sweepCountLabel = QLabel("No sweeps loaded")
        self.sweepCountLabel.setAlignment(Qt.AlignCenter)
        menuLayout.addWidget(self.sweepCountLabel)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        menuLayout.addWidget(sep)

        ivBtn = _make_btn("I–V Curve")
        p95Btn = _make_btn("P95 Current (all sweeps)")
        neTimeBtn = _make_btn("Electron Density vs Time")
        neAltBtn = _make_btn("Electron Density vs Altitude")
        teTimeBtn = _make_btn("Electron Temperature vs Time")
        teAltBtn = _make_btn("Electron Temperature vs Altitude")

        for btn in (ivBtn, p95Btn, neTimeBtn, neAltBtn, teTimeBtn, teAltBtn):
            menuLayout.addWidget(btn)

        backBtn = _make_btn("Back", 40)
        menuLayout.addWidget(backBtn)
        menuLayout.addStretch()

        self.canvas = MplCanvas()

        mainLayout.addLayout(menuLayout, 1)
        mainLayout.addWidget(self.canvas, 4)

        self.setLayout(mainLayout)

        ivBtn.clicked.connect(lambda: sweep_graphs.plot_iv_curve(
            self.canvas, self.sweeps, self.sweepSelector.value()
        ))

        p95Btn.clicked.connect(lambda: sweep_graphs.plot_p95(self.canvas, self.sweeps))
        neTimeBtn.clicked.connect(lambda: sweep_graphs.plot_ne_time(self.canvas, self.sweeps))
        neAltBtn.clicked.connect(lambda: sweep_graphs.plot_ne_altitude(self.canvas, self.sweeps))
        teTimeBtn.clicked.connect(lambda: sweep_graphs.plot_te_time(self.canvas, self.sweeps))
        teAltBtn.clicked.connect(lambda: sweep_graphs.plot_te_altitude(self.canvas, self.sweeps))

        backBtn.clicked.connect(lambda: self.stack.setCurrentIndex(0))

    def load_sweeps(self, dat_number):
        try:
            self.sweeps = sweep_graphs.load_data(dat_number)

            n = len(self.sweeps)
            self.sweepSelector.setMaximum(max(0, n - 1))
            self.sweepCountLabel.setText(f"{n} sweeps loaded")

        except Exception as e:
            self.sweeps = []
            self.sweepCountLabel.setText(f"Error: {e}")


# ─────────────────────────────────────────
# MAIN WINDOW
# ─────────────────────────────────────────
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("UDIP-6 Analysis")
        self.setFixedSize(1000, 650)

        self.current_file = None
        self.current_dat_number = None

        self.stack = QStackedWidget()

        self.home = HomePage(self)
        self.sensors = SensorsPage(self.stack)
        self.sweep = SweepPage(self.stack)

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.sensors)
        self.stack.addWidget(self.sweep)

        self.setCentralWidget(self.stack)


# ─────────────────────────────────────────
# RUN
# ─────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
