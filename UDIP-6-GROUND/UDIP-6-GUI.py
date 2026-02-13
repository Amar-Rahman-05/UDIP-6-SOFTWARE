import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QSizePolicy
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("University Of Delaware Ionospheric Probe")
        self.setMinimumSize(400, 300)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout
        layout = QVBoxLayout()
        layout.setSpacing(30)
        central_widget.setLayout(layout)

        # Sensor button
        sensor_btn = QPushButton("Sensor")
        sensor_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sensor_btn.setStyleSheet("font-size: 18pt;")

        # Sweep button
        sweep_btn = QPushButton("Sweep")
        sweep_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sweep_btn.setStyleSheet("font-size: 18pt;")

        layout.addWidget(sensor_btn)
        layout.addWidget(sweep_btn)

        sensor_btn.clicked.connect(self.open_sensor)
        sweep_btn.clicked.connect(self.open_sweep)

    def open_sensor(self):
        print("Sensor menu clicked")

    def open_sweep(self):
        print("Sweep menu clicked")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
