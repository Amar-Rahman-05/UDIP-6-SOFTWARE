from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDateEdit,
    QDateTimeEdit,
    QDial,
    QDoubleSpinBox,
    QFontComboBox,
    QLabel,
    QLCDNumber,
    QLineEdit,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QRadioButton,
    QSlider,
    QSpinBox,
    QTimeEdit,
    QVBoxLayout,QHBoxLayout, QGridLayout,QSizePolicy,
    QWidget,
)
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        widget = QWidget()
        #set background color
        self.setStyleSheet("background-color: lightgrey")
        self.setWindowTitle("UDIP-6 2026 Data")
        #Nested components
            #Layout 1 - Holds all layouts of window
            #Layout 2 - header containers
            #Layout 3 - sensor buttons & sweep/next and prev buttons containers
            #Layout 4 - graphs
        #Whole layout
        main_layout = QGridLayout()
        #main_layout.setContentsMargins(0,0,0,0)
        #main_layout.setSpacing(0)

        #Header Layout
        #UDIP header
        UDIP_header = QLabel("UDIP-6 Ground Software")
        UDIP_header.setStyleSheet("border: 2px solid black; font-size: 14pt; color: black; font-family: futura")

        #Packet Count & Graph Name
        header_container = QWidget()
        header_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        header_container.setStyleSheet("border: 2px solid black; font-size: 14pt; color: black; font-family: futura")
        
        #Layout for container
        header_container_layout = QHBoxLayout()
        header_container.setLayout(header_container_layout)