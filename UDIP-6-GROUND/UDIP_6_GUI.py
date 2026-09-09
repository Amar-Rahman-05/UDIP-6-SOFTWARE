import os
import sys
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QPushButton, QVBoxLayout, QLabel,
    QStackedWidget, QComboBox,
    QHBoxLayout, QSpinBox, QSizePolicy,
    QFrame, QDoubleSpinBox, QTextEdit
)
from PyQt5.QtCore import Qt

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg,
    NavigationToolbar2QT
)

import UDIP_6_GROUND_Analysis_Sensor as sensor_graphs
import UDIP_6_GROUND_Analysis_Sweep  as sweep_graphs
import UDIP_6_GROUND_Conversion as conversion
from calibration import CAL

# Matplotlib canvas wrapper used for embedding plots inside PyQt5 UI
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self):
        # Create a matplotlib figure
        self.fig = Figure()
        # Add a single subplot axis for plotting
        self.ax = self.fig.add_subplot(111)
        # Initialize the canvas with the figure
        super().__init__(self.fig)
        # Make the canvas expandable inside layouts
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

# Helper function to create styled QPushButton widgets
def _make_btn(label: str, height: int = 45) -> QPushButton:
    # Create button with text label
    btn = QPushButton(label)
    # Fix button height for consistent UI layout
    btn.setFixedHeight(height)
    # Apply simple font styling
    btn.setStyleSheet("font-size: 14px;")
    return btn

# Home page UI for selecting datasets and navigating to analysis pages
class HomePage(QWidget):

    def __init__(self, main_window):
        super().__init__()

        # Reference to main window for navigation
        self.main_window = main_window
        self.stack = main_window.stack

        # Get base directory of the script
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.folder_path = os.path.join(self.base_dir, "SD-DATA")

        # Main vertical layout for homepage UI
        layout = QVBoxLayout()

        # Title label at top of page
        title = QLabel("UDIP-6 ANALYSIS")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 28px; font-weight: bold;")

        # Dropdown for selecting dataset files
        self.dropdown = QComboBox()
        self.dropdown.setFixedWidth(400)

        # Load available files into dropdown
        self.load_files()

        # Navigation buttons for different analysis pages
        self.sensorBtn = QPushButton("SENSOR")
        self.sweepBtn = QPushButton("SWEEP")
        
        # Style both buttons consistently
        for btn in (self.sensorBtn, self.sweepBtn):
            btn.setFixedSize(300, 80)
            btn.setStyleSheet("font-size: 20px;")

        # Build layout structure with spacing and alignment 
        layout.addStretch()
        layout.addWidget(title)
        layout.addSpacing(30)
        layout.addWidget(self.dropdown, alignment=Qt.AlignCenter)
        layout.addSpacing(40)
        layout.addWidget(self.sensorBtn, alignment=Qt.AlignCenter)
        layout.addSpacing(20)
        layout.addWidget(self.sweepBtn, alignment=Qt.AlignCenter)
        layout.addStretch()

        # Set final layout for the widget
        self.setLayout(layout)

        # Connect buttons to navigation functions
        self.sensorBtn.clicked.connect(self._open_sensor)
        self.sweepBtn.clicked.connect(self._open_sweep)

    # Populate the dropdown with available data files
    def load_files(self):
        # Ensure the data folder exists (create if missing)
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)

        # Clear existing dropdown entries before reloading
        self.dropdown.clear()

        # Get list of all files (not directories) in the data folder
        files = [
            f for f in os.listdir(self.folder_path)
            if os.path.isfile(os.path.join(self.folder_path, f))
        ]

        # Add file names to the dropdown menu
        self.dropdown.addItems(files)

    # Convert selected filename into a dat number identifier
    def _get_dat_number(self):
        # Get currently selected filename from dropdown
        filename = self.dropdown.currentText()

        # If nothing selected, return None
        if not filename:
            return None
        
        # If same file already processed, reuse stored result (avoid recomputation)
        if self.main_window.current_file == filename:
            return self.main_window.current_dat_number
        
        # Build full path to selected .dat file
        dat_path = Path(self.folder_path) / filename

        try:
            # Convert .dat file into processed format and get dat number
            dat_number = conversion.process_dat_file(dat_path)
        except Exception as e:
            # Handle conversion errors gracefully
            print("[ERROR] Conversion failed:", e)
            return None
        
        # Cache results in main window to avoid reprocessing
        self.main_window.current_file = filename
        self.main_window.current_dat_number = dat_number

        return dat_number
    
    # Open sensor analysis page using selected dataset
    def _open_sensor(self):
        # Get dat number from selected file
        dat_number = self._get_dat_number()

        # If conversion failed or no file selected, do nothing
        if not dat_number:
            return
        
        # If conversion failed or no file selected, do nothing
        self.main_window.sensors.set_dat_number(dat_number)

        # Switch to sensor page in stacked UI
        self.stack.setCurrentIndex(1)

    # Open sweep analysis page using selected dataset
    def _open_sweep(self):
        # Get dat number from selected file
        dat_number = self._get_dat_number()

        # If conversion failed or no file selected, do nothing
        if not dat_number:
            return
        
        # Load sweep data into sweep analysis page
        self.main_window.sweep.load_sweeps(dat_number)

        # Switch to sweep page in stacked UI
        self.stack.setCurrentIndex(2)

# SensorsPage handles visualization of all onboard sensor data
class SensorsPage(QWidget):

    def __init__(self, stack):
        super().__init__()

        # Stores the currently selected dataset identifier
        self.dat_number = None

        # Reference to stacked widget for navigation between pages
        self.stack = stack

        # Main horizontal layout: left menu + right plotting area
        mainLayout = QHBoxLayout()

        # Vertical layout for sensor selection buttons
        menuLayout = QVBoxLayout()

        # Page title label
        title = QLabel("Sensors")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        menuLayout.addWidget(title)

        # Buttons for different sensor visualizations
        accelBtn = _make_btn("Accelerometer")
        magBtn = _make_btn("Magnetometer")
        gyroBtn = _make_btn("Gyroscope")
        highAccelBtn = _make_btn("High Range Accel")
        tempBtn = _make_btn("Temperature")
        

        # Add all sensor buttons to the menu layout
        for btn in (accelBtn, magBtn, gyroBtn, highAccelBtn, tempBtn):
            menuLayout.addWidget(btn)

        # Back button returns to home page
        backBtn = _make_btn("Back", 40)
        menuLayout.addWidget(backBtn)
        menuLayout.addStretch()

        # Matplotlib canvas for rendering sensor plots
        self.canvas = MplCanvas()

        # Add menu (left) and canvas (right) to main layout
        mainLayout.addLayout(menuLayout, 1)
        mainLayout.addWidget(self.canvas, 4)

        # Set final layout for widget
        self.setLayout(mainLayout)

        # Connect buttons to plotting functions
        accelBtn.clicked.connect(lambda: sensor_graphs.plot_accel(self.canvas, self.dat_number))
        magBtn.clicked.connect(lambda: sensor_graphs.plot_mag(self.canvas, self.dat_number))
        gyroBtn.clicked.connect(lambda: sensor_graphs.plot_gyro(self.canvas, self.dat_number))
        highAccelBtn.clicked.connect(lambda: sensor_graphs.plot_high_accel(self.canvas, self.dat_number))
        tempBtn.clicked.connect(lambda: sensor_graphs.plot_temp(self.canvas, self.dat_number))

        # Navigate back to home page
        backBtn.clicked.connect(lambda: self.stack.setCurrentIndex(0))


    # Update dataset ID used for all sensor plots
    def set_dat_number(self, dat_number):
        self.dat_number = dat_number
        
class SweepPage(QWidget):

    def __init__(self, stack):
        super().__init__()

        self.stack = stack
        self.sweeps = []
        self.dat_number = None

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

        calLabel = QLabel("Calibration")
        calLabel.setAlignment(Qt.AlignCenter)
        calLabel.setStyleSheet("font-weight: bold;")

        self.shunt_input = QDoubleSpinBox()
        self.shunt_input.setDecimals(6)
        self.shunt_input.setRange(1e-9, 1e9)
        self.shunt_input.setValue(CAL.shunt_resistance)
        self.shunt_input.setSuffix(" Ω")

        self.current_gain_input = QDoubleSpinBox()
        self.current_gain_input.setDecimals(6)
        self.current_gain_input.setRange(1e-9, 1e9)
        self.current_gain_input.setValue(CAL.current_gain)
        self.current_gain_input.setSuffix(" gain")

        self.voltage_gain_input = QDoubleSpinBox()
        self.voltage_gain_input.setDecimals(6)
        self.voltage_gain_input.setRange(1e-9, 1e9)
        self.voltage_gain_input.setValue(CAL.voltage_gain)
        self.voltage_gain_input.setSuffix(" Vgain")

        menuLayout.addSpacing(10)
        menuLayout.addWidget(calLabel)
        menuLayout.addWidget(self.shunt_input)
        menuLayout.addWidget(self.current_gain_input)
        menuLayout.addWidget(self.voltage_gain_input)

        self.eqLabel = QTextEdit()

        self.eqLabel.setReadOnly(True)

        self.eqLabel.setMinimumHeight(250)

        self.eqLabel.setStyleSheet("""
            QTextEdit {
               font-size: 10pt;
               background-color: #f0f0f0;
               border: 1px solid gray;
               padding: 5px;
            }
            """)

        menuLayout.addWidget(self.eqLabel)

        eqSep = QFrame()
        eqSep.setFrameShape(QFrame.HLine)
        menuLayout.addWidget(eqSep)

        self.update_equation_display()

        self.shunt_input.valueChanged.connect(self.update_calibration)
        self.current_gain_input.valueChanged.connect(self.update_calibration)
        self.voltage_gain_input.valueChanged.connect(self.update_calibration)

        ivBtn = _make_btn("Corrected I–V Curve")
        rawIvBtn = _make_btn("Raw I–V Curve")
        neTimeBtn = _make_btn("Electron Density vs Time")
        teTimeBtn = _make_btn("Electron Temperature vs Time")
        teAltBtn = _make_btn("Electron Temperature vs Altitude")
        neAltBtn = _make_btn("Electron Density vs Altitude")
        


        for btn in (ivBtn, rawIvBtn, neTimeBtn, teTimeBtn, teAltBtn, neAltBtn):
            btn.setFixedHeight(28)

        for btn in (ivBtn, rawIvBtn, neTimeBtn, teTimeBtn, teAltBtn, neAltBtn):
            menuLayout.addWidget(btn)

        self.sweepSelector.valueChanged.connect(self._refresh_iv)

        backBtn = _make_btn("Back", 40)
        menuLayout.addWidget(backBtn)
        menuLayout.setSpacing(3)

        self.canvas = MplCanvas()
        self.toolbar = NavigationToolbar2QT(self.canvas, self)

        graphLayout = QVBoxLayout()
        graphLayout.addWidget(self.toolbar)
        graphLayout.addWidget(self.canvas)

        mainLayout.addLayout(menuLayout, 2)
        mainLayout.addLayout(graphLayout, 4)
        self.setLayout(mainLayout)

        ivBtn.clicked.connect(lambda: sweep_graphs.plot_iv_curve(
            self.canvas, self.sweeps, self.sweepSelector.value()
        ))

        rawIvBtn.clicked.connect(lambda: sweep_graphs.plot_raw_iv_curve(
            self.canvas, self.sweeps, self.sweepSelector.value()
        ))


        
        neTimeBtn.clicked.connect(lambda: sweep_graphs.plot_ne_time(self.canvas, self.sweeps))
        teTimeBtn.clicked.connect(lambda: sweep_graphs.plot_te_time(self.canvas, self.sweeps))

        backBtn.clicked.connect(lambda: self.stack.setCurrentIndex(0))

        teAltBtn.clicked.connect(lambda: sweep_graphs.plot_te_altitude(
            self.canvas,
            self.sweeps,
            time_offset_s=180.0
    ))

        neAltBtn.clicked.connect(lambda: sweep_graphs.plot_ne_altitude(
            self.canvas,
            self.sweeps,
            time_offset_s=180.0
            ))

    def _refresh_iv(self):
        if self.sweeps:
            sweep_graphs.plot_iv_curve(
                self.canvas,
                self.sweeps,
                self.sweepSelector.value()
            )

    def update_equation_display(self):

        self.eqLabel.setPlainText(
            "Voltage Conversion\n"
        "────────────────────\n"
            f"VX1 = VrefA × {CAL.adc_vref:.3f} / 4095\n"
            f"VX2 = VrefB × {CAL.adc_vref:.3f} / 4095\n\n"

            f"Vprobe = (VX2 - VX1) × {CAL.voltage_gain:.3f}\n\n"

            "Current Conversion\n"
            "────────────────────\n"
            f"VA = ADC_A × {CAL.adc_vref:.3f} / 4095\n"
            f"VB = ADC_B × {CAL.adc_vref:.3f} / 4095\n\n"

            f"Vdiff = (VA - VB) × {CAL.current_gain:.3f}\n\n"

            f"I = Vdiff / {CAL.shunt_resistance:.3e}\n\n"

            "I(nA) = I × 1e9"
        )

    def update_calibration(self):
        CAL.shunt_resistance = self.shunt_input.value()
        CAL.current_gain = self.current_gain_input.value()
        CAL.voltage_gain = self.voltage_gain_input.value()

        self.update_equation_display()

        if self.dat_number:
            self.sweeps = sweep_graphs.load_data(self.dat_number)

        self._refresh_iv()

    def load_sweeps(self, dat_number):
        try:
            self.dat_number = dat_number
            self.sweeps = sweep_graphs.load_data(dat_number)

            n = len(self.sweeps)
            self.sweepSelector.setMaximum(max(0, n - 1))
            self.sweepCountLabel.setText(f"{n} sweeps loaded")

            self.shunt_input.blockSignals(True)
            self.current_gain_input.blockSignals(True)
            self.voltage_gain_input.blockSignals(True)

            self.shunt_input.setValue(CAL.shunt_resistance)
            self.current_gain_input.setValue(CAL.current_gain)
            self.voltage_gain_input.setValue(CAL.voltage_gain)

            self.shunt_input.blockSignals(False)
            self.current_gain_input.blockSignals(False)
            self.voltage_gain_input.blockSignals(False)


            # reset selector
            self.sweepSelector.setValue(0)

            # auto-render first sweep
            if self.sweeps:
                sweep_graphs.plot_iv_curve(self.canvas, self.sweeps, 0)
                self.canvas.draw_idle()

        except Exception as e:
            self.sweeps = []
            self.sweepCountLabel.setText(f"Error: {e}")


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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
