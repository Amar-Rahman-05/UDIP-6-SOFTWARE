class Calibration:
    def __init__(self):
        self.shunt_resistance = 1.0e6
        self.current_gain = 4.01
        self.voltage_gain = 4.01
        self.adc_vref = 3.3

CAL = Calibration()
