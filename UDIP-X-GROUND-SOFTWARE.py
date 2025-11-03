# -*- coding: utf-8 -*-
"""
Class definitions for Reading Packets from University of Delaware Ionosphere
Probe RockSat-X 2025 Mission
Last updated 19/05/25
@author: Ameer Abdelnasser, Nate Riehl
"""
from numpy import tile
import numpy as np

from struct import unpack
from scipy import interpolate

import math
import os
import matplotlib.pyplot as plt

# A*n*e*SQRT((k_b*T_e)/(2*PI()*m))*SQRT((1+(e*V)/(k_b*T_e))

A = 0.000296 # m^2
e = 1.6e-19 # C
k_b = 1.38e-23 # J/K
m = 9.11e-31 # kg

STEP_LEN = 8
REF_LEN = 4
NUM_STEPS = 356

VERBOSE = False
lenHedr = 15

lenSens = 24
# lenBrst = 133 * 8 + 12
lenMed = NUM_STEPS * 8 + 4
# lenLrg = 253 * 8 + 12
# lenCst = 100 * 10 + 6

typeSens = 0x01
typeMed  = 0x10
# typeLrg  = 0x11
# typeBrst = 0x20
# typeCst  = 0x30
#------------------------------------------------------#
#Parent Class of all Packets
#Attributes:
#Global:
#   totCnt- Total number of created packets
#Local:
#   tInitial- Time of packet creation in ms (float)
#   tFinal- Final time of packet creation in ms (float)
#   pcktType- Type of Packet (int)
#       Sensor = 0x01        Medium Sweep = 0x10
#       Large Sweep = 0x11   Burst Sweep = 0x20
#   payloadLen- Length of packet payload (int)
#------------------------------------------------------#
class Packet:
    """
    Parent Class of all Packets. Subclasses: Sensor_Packet, Medium_Sweep,
    Large_Sweep, Burst_Sweep.
    
    Attributes:
        Global:
            totCnt- Total number of created packets
        Local:
            count- This object's count
            tInitial- Time of packet creation in ms (float)
            tFinal- Final time of packet creation in ms (float)
            pcktType- Type of Packet (int)
                Sensor = 0x01        Medium Sweep = 0x10
                Large Sweep = 0x11   Burst Sweep = 0x20
            payloadLen- Lenght of packet payload (int)
    """
    totCnt = 0
    def __init__(self, count, tInitial, tFinal, myType, payloadLen):
        '''
        Initialization function for class: Packet.
        
        Parameters:
            count: 2 byte unsigned int 
            tInitial: 4 byte unsigned int
            tFinal: 4 byte unsigned int
            myType: 1 byte "char" for redundancy on packet types
            payloadLen: 2 byte int, total number of bytes in the payload
        '''
        Packet.totCnt += 1
        self.count = count
        self.tInitial = tInitial
        self.tFinal = tFinal
        self.pcktType = myType
        self.payloadLen = payloadLen
        
    
    def __del__(self):
        type(self).totCnt -= 1
        
    
    

class Sensor_Packet(Packet):
    """
    Subclass of Packet, holds the sensor packet data.
    
    Attributes:
        All Packet attributes
        pyld: List of bytes to convert
        accelScale: Maximum range of accelerometer 2, 4, 8, or 16 g
        accX, accY, accZ: Acceleration in g (9.81 m/s^2) in three axes
        gyrX, gyrY, gyrZ: Spin rate in degrees per second in three axes
        magX, magY, magZ: Magnetic field in Gauss in three axes
        accAna: Accerleration in g (9.81 m/s^2) in z axis (along path of rocket) from analog sensor
        temperature: Internal canister temperature
        temperatureAna: Internal canister temperature from analog sensor
    """
    SENSITIVITY_ACCELEROMETER_2 = 0.000061
    SENSITIVITY_ACCELEROMETER_4 = 0.000122
    SENSITIVITY_ACCELEROMETER_8 = 0.000244
    SENSITIVITY_ACCELEROMETER_16= 0.000732
    
    SENSITIVITY_GYROSCOPE_2000  = 0.07
    
    SENSITIVITY_MAGNETOMETER_4  = 0.00014
    
    temperatureConversion = 0.0625

    def __init__(self, count, tInitial, tFinal, pcktType, payloadLen, pyld):
        """
        Initialization method for class: Sensor_Packet.
        
        Parameters:
            count: 2 byte unsigned int 
            tInitial: 4 byte unsigned int
            tFinal: 4 byte unsigned int
            myType: 1 byte "char" for redundancy on packet types
            payloadLen: 2 byte int, total number of bytes in the payload
            pyld: List of bytes
        """
        super().__init__(count, tInitial, tFinal, pcktType, payloadLen)
        self.pyld = pyld
        self.readPyld()
        
           
    
    def readPyld(self):
        """Extracts bytes from pyld and populates sensor attributes"""
        #Unpack Acceleration data from payload
        #self.accScale = unpack('<B', self.pyld[lenSens - 1:lenSens])[0]
        #self.acc = unpack('<hhhH',self.pyld[0:8])#three signed two byte integers and 1 unsigned two byte integer
        self.acc_m = unpack('<hhh',self.pyld[0:6])
        self.acc_h = unpack("<h", self.pyld[6:8])[0]
        self.calibrateDigAcc()

        #Acceleration data from analog sensor
        
        
        #Unpack Gyroscope data from payload
        self.gyr = unpack('<hhh', self.pyld[8:14])
        self.calibrateDigGyr()
        
        #Unpack Magnetic Field data from payload
        self.mag = unpack('<hhh', self.pyld[14:20])
        self.calibrateDigMag()
        
        #Unpack Temperature data from payload
        tmp_a, tmp_d = unpack('<Hh', self.pyld[20:24])
        tmp_a /= 16
        voltage_at_adc = tmp_a * (3300 / 4095.0)

        # 3) apply the TMP36 transfer function
        self.temperature_a = (voltage_at_adc - 500) / 10

        raw12 = tmp_d >> 4

        # Now convert: 16 LSB per °C, zero at +25 °C
        self.temperature_d = 25.0 + (raw12 / 16.0)

        # print(f"tempA raw voltage (mV) : {tmp_a}")
        # print(f"raw IMU temp = {self.temperature_d}")


        b = 3455
        # if(tmp_p == 0):
        #     print('Here')
        #     tmp_p = 0.1
        # if(tmp_s == 0):
        #     tmp_s = 0.1
        # tmp_p_res = ((3.3 * 27000) / ((tmp_p*3.3)/(4096*16))) - 27000
        # tmp_s_res = ((3.3 * 27000) / ((tmp_s*3.3)/(4096*16))) - 27000
        # self.temperature_p = b / math.log(tmp_p_res / (10000 * math.exp((-1 * b) / 298 ))) - 273
        # self.temperature_s = b / math.log(tmp_s_res / (10000 * math.exp((-1 * b) / 298 ))) - 273
        
        #Unpack photodiode data
        # self.pd_1, self.pd_2 = unpack('<HH', self.pyld[26:30])
        
    def calibrateDigAcc(self):
        self.accScale = 16 
        """Calibrates the acceleration and stores it in respective class attributes in g"""
        self.accH = self.acc_h / 33.4
        # if(self.accScale == 2):
        #     self.accX = self.acc_m[0] * self.SENSITIVITY_ACCELEROMETER_2
        #     self.accY = self.acc_m[1] * self.SENSITIVITY_ACCELEROMETER_2
        #     self.accZ = self.acc_m[2] * self.SENSITIVITY_ACCELEROMETER_2
        # elif(self.accScale == 4):
        #     self.accX = self.acc_m[0] * self.SENSITIVITY_ACCELEROMETER_4
        #     self.accY = self.acc_m[1] * self.SENSITIVITY_ACCELEROMETER_4
        #     self.accZ = self.acc_m[2] * self.SENSITIVITY_ACCELEROMETER_4
        # elif(self.accScale == 8):
        #     self.accX = self.acc_m[0] * self.SENSITIVITY_ACCELEROMETER_8
        #     self.accY = self.acc_m[1] * self.SENSITIVITY_ACCELEROMETER_8
        #     self.accZ = self.acc_m[2] * self.SENSITIVITY_ACCELEROMETER_8
        # elif(self.accScale == 16):
        #     self.accX = self.acc_m[0] * self.SENSITIVITY_ACCELEROMETER_16
        #     self.accY = self.acc_m[1] * self.SENSITIVITY_ACCELEROMETER_16
        #     self.accZ = self.acc_m[2] * self.SENSITIVITY_ACCELEROMETER_16
        # else:
        #     #Not valid accel scale
        #     self.accX = 0
        #     self.accY = 0
        #     self.accZ = 0
        self.accX = self.acc_m[0] / 95.43
        self.accY = self.acc_m[1] / 95.43
        self.accZ = self.acc_m[2] / 95.43
            
    
    def calibrateDigGyr(self):
        """Calibrates the spin rate and stores it in respective class attributes in degrees per second"""
        SENS_GYRO_2000 = 0.05    # °/s per LSB
        self.gyroX = self.gyr[0] * SENS_GYRO_2000
        self.gyroY = self.gyr[1] * SENS_GYRO_2000
        self.gyroZ = self.gyr[2] * SENS_GYRO_2000
                
    
    def calibrateDigMag(self):
        """Calibrates the magnetic field data and stores it in respective class attributes in Gauss"""
        '''self.magX = self.mag[0] * self.SENSITIVITY_MAGNETOMETER_4
        self.magY = self.mag[1] * self.SENSITIVITY_MAGNETOMETER_4
        self.magZ = self.mag[2] * self.SENSITIVITY_MAGNETOMETER_4'''
        rawX, rawY, rawZ = self.mag

        # 1) collect first N readings
        cls = type(self)
        if not hasattr(cls, "_mag_bias"):
            cls._mag_buffer = getattr(cls, "_mag_buffer", [])
            cls._mag_buffer.append((rawX, rawY, rawZ))
            if len(cls._mag_buffer) >= 50:
                arr = np.array(cls._mag_buffer, float)
                cls._mag_bias = tuple(arr.mean(axis=0))
                del cls._mag_buffer

        # 2) subtract bias (0 until _mag_bias exists)
        bx, by, bz = getattr(cls, "_mag_bias", (0.0, 0.0, 0.0))
        x_corr = rawX - bx
        y_corr = rawY - by
        z_corr = rawZ - bz

        # 3) scale to Gauss
        self.magX = x_corr * self.SENSITIVITY_MAGNETOMETER_4
        self.magY = y_corr * self.SENSITIVITY_MAGNETOMETER_4
        self.magZ = z_corr * self.SENSITIVITY_MAGNETOMETER_4


class Medium_Sweep(Packet):
    """
    Subclass of Packet, creates an object of type Sweep.
    
    Attributes:
        All Packet Attributes
        sweep: Object of type Sweep that holds Voltage and Current values
    """
    
    nSteps = NUM_STEPS
    
    def __init__(self, count, tInitial, tFinal, pcktType, payloadLen, pyld):
        super().__init__(count, tInitial, tFinal, pcktType, payloadLen)
        self.sweep = Sweep(pyld,self.nSteps)
        return

class Sweep():
    """
    Holds Voltage and Current values from a voltage sweep.
    
    Attributes:
        R* are resistor values on board analog board
        pyld: List of bytes to convert
        nSteps: Number of steps in the voltage sweep
        vPos, vNeg: Voltage supply to analog board used as reference
        sweepVoltage: List of voltages applied to the probe
        adc1Sig: List of currents induced by the probe
        adc2Sig: List of currents induced by the probe with one gain stage
        adc3Sig: List of currents induced by the probe with two gain stages
        
    """
    nStepsMed = NUM_STEPS
    # nStepsLrg = 253
    # nStepsBrst = 133 * 10
    
    nAvg = 16.0

    ## U1
    # Inputs -> DAC0, VREF
    # Outputs -> V_SWEEP
    R1 = 2.91e3
    RG2 = 432
    RG1 = 3.92e3
    RG3 = 100e3

    ## U2
    # Inputs -> V_SWEEP, Ip
    # Outputs -> V_TIA
    RFT1 = 500e3

    ## U3
    # Inputs -> V_SWEEP, V_TIA
    # Outputs -> V_SIGNAL
    R2 = 10e3
    R3 = 10e3
    R4 = 10e3
    R5 = 10e3

    ## U4
    # Inputs -> V_SIGNAL, VREF
    # Outputs -> V_ADC1
    R6 = 10e3
    R7 = 10e3
    R8 = 30.1e3
    R9 = 10e3

    ## U5
    # Inputs -> V_SIGNAL
    # Outputs -> SIGNAL_20
    R10 = 10e3
    R11 = 191e3

    ## U6
    # Inputs -> SIGNAL_20, VREF
    # Outputs -> V_ADC2
    R12 = 10e3
    R13 = 10e3
    R14 = 30.1e3
    R15 = 10e3

    ## U7
    # Inputs -> SIGNAL_20
    # Outputs -> SIGNAL_400
    R16 = 10e3
    R17 = 191e3

    ## U8
    # Inputs -> SIGNAL_400, VREF
    # Outputs -> V_ADC3
    R18 = 10e3
    R19 = 10e3
    R20 = 30.1e3
    R21 = 10e3

    ## U9
    # Inputs -> V_SWEEP, VREF
    # Outputs -> V_ADC0
    R22 = 10e3
    R23 = 10e3
    R24 = 30.1e3
    R25 = 10e3

    R26 = 30.1e3
    R27 = 10e3

    ## U10
    # Inputs -> -12V, VREF
    # Outputs -> V_ADC5
    R30 = 10e3
    R31 = 10e3
    R32 = 30.1e3
    R33 = 10e3

    def __init__(self, pyld, nSteps):
        """
        Initialization method of class Sweep.
        
        Parameters:
            pyld: List of bytes
            nSteps: Number of steps in the voltage sweep. Sets up the length of
            attributes
        """
        self.nSteps = nSteps
        self.pyld = pyld
        # debug in Sweep.__init__, just after `self.pyld = pyld`
        print("  → payload len:", len(self.pyld))
        print("  → first 12 bytes:", list(self.pyld[:12]))

        # decode them manually
        vpos = int.from_bytes(self.pyld[0:2], 'little')
        vneg = int.from_bytes(self.pyld[2:4], 'little')
        adc0 = int.from_bytes(self.pyld[4:6], 'little')
        adc1 = int.from_bytes(self.pyld[6:8], 'little')
        adc2 = int.from_bytes(self.pyld[8:10], 'little')
        adc3 = int.from_bytes(self.pyld[10:12], 'little')
        print(f"  → SWP_VPOS={vpos}, SWP_VNEG={vneg}")
        print(f"  → first step: ADC0={adc0}, ADC1={adc1}, ADC2={adc2}, ADC3={adc3}")

        self.vPosVal = unpack('<H', pyld[0:2])[0]
        self.vNegVal = unpack('<H', pyld[2:4])[0]


        # updated 4/21
        self.vPos = (self.vPosVal / 16.0) * (3.3 / 4096.0) * (self.R26 + self.R27) / self.R27
        # 1 / R23 * ((R24 + R25) * (R22 + R23) * Vadc / R25 - R22 * VREF)
        foo = self.vNegVal / 16. * (3.3 / 4096.)
        self.vNeg = 1 / self.R31 * ((self.R32 + self.R33) * (self.R30 + self.R31) * foo / self.R33 - self.R30 * self.vPos)
        # (self.RN1 + self.RN2) / self.RN1 * ( ( (self.vNegVal / 16.0) * (3.3 / 4096.0) ) - (self.vPos ) * (self.RN2 / (self.RN1 + self.RN2)))
        print(self.vPos, self.vNeg)
        # previously

        self.adc0Val =  tile(0. , nSteps)
        self.adc0Volt = tile(0., nSteps)
        self.Vsweep = tile(0., nSteps)
        
        self.adc1Val =  tile(0. , nSteps)
        self.adc1Volt = tile(0., nSteps)
        self.adc1Sig = tile(0., nSteps)
        self.curr0 = tile(0., nSteps)

        self.adc2Val =  tile(0. , nSteps)
        self.adc2Volt = tile(0., nSteps)
        self.adc2Sig = tile(0., nSteps)
        self.curr20 = tile(0., nSteps)

        self.adc3Val =  tile(0. , nSteps)
        self.adc3Volt = tile(0., nSteps)
        self.adc3Sig = tile(0., nSteps)
        self.curr400 = tile(0., nSteps)
        # self.ProbeVolt = tile(0. , self.nSteps)
        
        self.adc1SigMax = 0
        self.adc2SigMax = 0
        self.adc3SigMax = 0
        self.adc1SigMin = 0
        self.adc2SigMin = 0
        self.adc3SigMin = 0
        
        self.fillVals()
        self.fillVolt()
        self.fillSig()
        self.sweepTransfer()
        self.makeRCurrent()
        return
            
    
    def setDACcmd(dacArr):
        x = 1
        return
    
    def fillVals(self):
        """Unpacks the DAC voltage and three ADC values"""
        for i in range(self.nSteps):
            start = REF_LEN + i * STEP_LEN
            arr = unpack('<HHHH', self.pyld[start: start + STEP_LEN])
            self.adc0Val[i] = arr[0] / self.nAvg
            self.adc1Val[i] = arr[1] / self.nAvg
            self.adc2Val[i] = arr[2] / self.nAvg
            self.adc3Val[i] = arr[3] / self.nAvg
        return

    
    #change to fit our ADC names
    def fillVolt(self):
        """Converts a value from ADC to the voltage"""
        conv = 3.3 / 4096.
        for i in range(self.nSteps):
            self.adc0Volt[i] = self.adc0Val[i] * conv
            self.adc1Volt[i] = self.adc1Val[i] * conv
            self.adc2Volt[i] = self.adc2Val[i] * conv
            self.adc3Volt[i] = self.adc3Val[i] * conv
            # self.ProbeVolt[i] = (self.adc0Val[i] * conv - 1.5) * 24
        return
         
    #correlates to signal_to_current(Vsig, gain), but nate takes in signal voltage and gain
    def fillSig(self):
        """"Converts the voltage to ADC to a current from the probe"""
        for i in range(self.nSteps):
            self.adc1Sig[i] = 1 / self.R7 * ((self.R8 + self.R9) * (self.R6 + self.R7) * self.adc1Volt[i] / self.R9 - self.R6 * self.vPos) # (1/self.RF) * ((self.R1+self.R2)/self.R2) * ( ((self.R4+self.R3)/self.R4) * self.adc0Volt[i] - (self.R1/(self.R1+self.R2)) * self.vPos ) * ((self.RG1/(self.RG1+self.RG2))**0) * 1E9
            self.adc2Sig[i] = 1 / self.R13 * ((self.R14 + self.R15) * (self.R12 + self.R13) * self.adc2Volt[i] / self.R15 - self.R12 * self.vPos)# (1/self.RF) * ((self.R1+self.R2)/self.R2) * ( ((self.R4+self.R3)/self.R4) * self.adc1Volt[i] - (self.R1/(self.R1+self.R2)) * self.vPos ) * ((self.RG1/(self.RG1+self.RG2))**1) * 1E9
            self.adc3Sig[i] = 1 / self.R19 * ((self.R20 + self.R21) * (self.R18 + self.R19) * self.adc3Volt[i] / self.R21 - self.R18 * self.vPos)# (1/self.RF) * ((self.R1+self.R2)/self.R2) * ( ((self.R4+self.R3)/self.R4) * self.adc2Volt[i] - (self.R1/(self.R1+self.R2)) * self.vPos ) * ((self.RG1/(self.RG1+self.RG2))**2) * 1E9

        return
           
    #correlates to adc_to_signal(Vadc) in 
    def sweepTransfer(self):
        """Converts voltage to ADC from DAC pin to voltage applied to probe"""
        self.Vsweep =  1  / self.R23 * ((self.R24 + self.R25) * (self.R22 + self.R23) * self.adc0Volt / self.R25 - self.R22 * self.vPos)
        return
    
    def makeRCurrent(self):
        # -1 / RFT1 * Vsig / gain
        self.curr0 = - 1 / (self.RFT1 * 1) *  self.adc1Sig
        self.curr20 = - 1 / (self.RFT1 * 20.1) *  self.adc2Sig
        self.curr400 = - 1 / (self.RFT1 * 404.01) *  self.adc3Sig



def readFile(fileName):
    """
    Main function of Conversion Software. Reads a file and creates a list of packet objects
    
    Parameters:
        fileName (str): Name of file
        
    Returns:
        myPackets (list): List of packet objects
    
    """
    myPackets = []
    myFile = open(fileName,"rb")
    raw = myFile.read()
    loc = 0
    i = 0
    while(loc < len(raw)):
        sync     = unpack('<BB',raw[loc + 0:loc + 2])
        count    = unpack('<H', raw[loc + 2:loc + 4])[0]
        tInitial = unpack('<I', raw[loc + 4:loc + 8])[0]
        tFinal   = unpack('<I', raw[loc + 8:loc + 12])[0]
        pcktType = unpack('<B', raw[loc + 12:loc + 13])[0]
        pyldLen  = unpack('<H', raw[loc + 13:loc + 15])[0]
        
        # *** NEW: make sure the file actually contains the whole packet ***
        if loc + lenHedr + pyldLen > len(raw):
            if VERBOSE:
                print(f"  → Incomplete packet at offset {loc}: "
                      f"header says {pyldLen} payload bytes, "
                      f"but only {len(raw)-loc-lenHedr} available.")
            break
        
        if(verifyHeader(sync, pcktType, pyldLen)):
            if(pcktType == typeSens):
                packet = Sensor_Packet(count, tInitial, tFinal, pcktType, pyldLen, raw[(loc+lenHedr) : (loc+lenHedr+pyldLen)] )
                myPackets.append(packet)
            elif(pcktType == typeMed):
                packet = Medium_Sweep(count, tInitial, tFinal, pcktType, pyldLen, raw[(loc+lenHedr) : (loc+lenHedr+pyldLen)])
                myPackets.append(packet)
            else:
                loc+=1
                print('Missing link')
            loc += lenHedr+pyldLen
            if(VERBOSE):
                print("Packet #: ", i)
            i+=1
        else:
            loc+=1
        
    return myPackets
                
    
def verifyHeader(sync, pcktType, pyldLen):
    """
    Helper function for readFile
    
    Parameters:
        sync (list): two byte array
        pcktType (int): type of packet in numerical value
        pyldLen (int): length of payload in bytes
        
    Returns:
        Boolean for if the header is valid
    """
    if(sync[0] != 0x55 or sync[1] != 0x44):
        return False
    if(pcktType == typeSens):
        if(pyldLen != lenSens):
            return False
        else:
            return True
    if(pcktType == typeMed):
        if(pyldLen != lenMed):
            return False
        else:
            return True

def readTele(fileName):
    """
    Extracts the data from telemetry file
    
    Parameters
    ----------
    fileName : String
        Name of the file, should be telemetry file DQCA_quicklook...

    Returns
    -------
    flightTime : list
        list of the times from telemetry
    altitude : list
    
    horRange : list
    
    velocity : list

    """
    teleFile = open(fileName, "r")
    #get rid of the first 5 lines which hold no data
    for line in range(5):
        teleFile.readline()
    
    fileArr = teleFile.readlines()
    
    arrLen = len(fileArr)
    
    flightTime = []
    altitude = []
    horRange = []
    velocity = []
    
    for i in range(arrLen):
        data = fileArr[i].split()
        flightTime.append(float(data[0]))
        altitude.append(float(data[2]))
        horRange.append(float(data[3]))
        velocity.append(float(data[4]))
    
    return flightTime, altitude, horRange, velocity

def fitFlight(fileName):
    """
    Creates the fit function for the altitude as a function of time (seconds)

    Parameters
    ----------
    fileName : String
        DESCRIPTION.

    Returns
    -------
    Interpolatant function which takes a time (in seconds) and returns the altitude

    """
    flightTime, alt, rng, vel = readTele(fileName)
    
    return interpolate.interp1d(flightTime,alt), interpolate.interp1d(flightTime, vel)
    

    

def makeIVPlot(Vsweep, curr, idx, gain, sweepNum, pkt):
    plt.subplot(3, 1, idx)
    plt.scatter(Vsweep[95:250], curr[95:250], label="Ipr")
    # # mask = np.abs(Vsweep) < 6
    # p = np.polyfit(Vsweep, curr, 1)
    # l = p[0] * Vsweep + p[1]
    # plt.plot(Vsweep, l, label="m ≈ 1/R", color='r')
    plt.xlabel("$V_{sweep}$", loc='right')
    plt.ylabel("$I_{r}$")
    plt.legend()
    plt.title(f"{gain} gain I-V curve, sweep {sweepNum}, t = {pkt.tInitial * 1e-3} sec")
    # plot window (same as your scatter)
    

def makeVPlot(Vsweep, sig, idx, gain):
    plt.subplot(3, 1, idx)
    plt.plot(Vsweep, label="Vsweep")
    plt.plot(sig, label=f"Vsig{gain}")
    plt.xlabel("n-samples", loc='right')
    plt.ylabel("V")
    plt.legend()
    plt.title(f"{gain} gain Vsweep, Vsig, 10M$\Omega$")
     
if __name__ == '__main__':
    #Unit test cases
    # Make ROOT be the directory where this script lives:
    ROOT = os.path.dirname(os.path.abspath(__file__))

    # Then load the DAT file right next to it:
    # packetList = readFile(os.path.join(ROOT, "2025-05-21T18.39.38.bin"))
    packetList = readFile("/Users/nriehl/Documents/UDIP-X/FLIGHT-DATA/UDIP0087.DAT")

    accXList = []
    accYList = []
    accZList = []
    accHList = []
    gyrXList = []
    gyrYList = []
    gyrZList = []
    magXList = []
    magYList = []
    magZList = []
    tmpAList = []
    tmpDList = []
    timeList = []

    sensList = []
    medSwpList = []

    for packet in packetList:
        if(packet.pcktType == typeSens):
            sensList.append(packet)
            accXList.append(packet.accX)
            accYList.append(packet.accY)
            accZList.append(packet.accZ)
            accHList.append(packet.accH)
            gyrXList.append(packet.gyroX)
            gyrYList.append(packet.gyroY)
            gyrZList.append(packet.gyroZ)
            magXList.append(packet.magX)
            magYList.append(packet.magY)
            magZList.append(packet.magZ)
            tmpAList.append(packet.temperature_a)
            tmpDList.append(packet.temperature_d)
            timeList.append(packet.tFinal)
            
        elif(packet.pcktType == typeMed):
            medSwpList.append(packet)

        else:
            print('Why?')


    print(f"total time = {sensList[-1].tFinal - sensList[0].tInitial} ms")

    # Print accelerations
    timeList = [i * 1e-3 for i in timeList]
    plt.plot(timeList, accXList, label="x")
    plt.plot(timeList, accYList, label="y")
    plt.plot(timeList, accZList, label="z")
    plt.plot(timeList, accHList, label='HR')
    print(f"timeList[0] {timeList[0]}")
    print(f"accZList[10] {accZList[10]}")
    plt.ylabel(f"Acceleration $ m/s^2 $")
    pwrOnX = 0
    pwrOnY = 10
    launchTimeX = 180
    launchTimeY = 10
    firstStageCutoffX = 180
    firstStageCutoffY = 175
    secondStageIgnitionX = 195
    secondStageIgnitionY = 0
    losX = 200
    losY = 230
    plt.annotate('Launch', xy=(180, 10), xytext=(100, 50), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    plt.annotate('First stage cutoff', xy=(firstStageCutoffX, firstStageCutoffY), xytext=(firstStageCutoffX - 150, firstStageCutoffY - 40), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    plt.annotate('Second stage ignition', xy=(secondStageIgnitionX, secondStageIgnitionY), xytext=(secondStageIgnitionX + 80, secondStageIgnitionY + 20), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    plt.annotate('Erroneous data begins', xy=(losX, losY), xytext=(losX + 80, losY - 20), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    plt.annotate('Power on', xy=(pwrOnX, pwrOnY), xytext=(pwrOnX + 40, pwrOnY + 20), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))
    # plt.plot(timeList, accHList, label="HR")
    plt.legend()
    plt.title("UDIP-X Accelerometer: Acceleration vs Time")
    plt.show()

    plt.plot(timeList, accHList, label='HR')
    plt.ylabel(f"Acceleration $ m/s^2 $")
    plt.xlabel('Time (s)')
    plt.title("High range accelerometer")
    plt.show()

    # # Print gyroscopes
    

    plt.plot(timeList, gyrXList, label='x')
    plt.plot(timeList, gyrYList, label='y')
    plt.plot(timeList, gyrZList, label='z')
    plt.title("UDIP-X Gyroscope: Angular frequency vs Time")
    plt.xlabel("Time (s)")
    plt.ylabel(f"Angular frequency (dps)")
    plt.legend()
    plt.show()

    # # print magnetometer

    plt.plot(timeList, magXList, label='x')
    plt.plot(timeList, magYList, label='y')
    plt.plot(timeList, magZList, label='z')
    plt.title("UDIP-X Magnetometer: Magentic flux density vs time")
    plt.xlabel("Time (s)")
    plt.ylabel("Magnetic Flux Density (G)")
    plt.legend()
    plt.show()

    window = 10

    plt.plot(timeList, tmpAList, label="analog temp")
    plt.plot(timeList, tmpDList, label="digital temp")
    plt.title("Temperature vs time")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (C)")
    plt.legend()
    plt.show()

    print(f"medSwpList len = {len(medSwpList)}")


    currents_95 = [0] * len(medSwpList);
    for i in range(len(medSwpList)):
        pkt_i = medSwpList[i]
        currents_95[i] = np.percentile(pkt_i.sweep.curr20, 95)


    plt.figure()
    plt.scatter(range(len(medSwpList)), currents_95)
    plt.xlabel("Sweep index")
    plt.ylabel("Current (A)")
    plt.title("95th percentile current")
    plt.annotate('Best case', xy=(241, 1.02e-6), xytext=(160, 0.9e-6), arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=4))

    plt.show()

    sweepNum = 241
    pkt = medSwpList[sweepNum]
    print(f"pkt.sweep.curr0 = {pkt.sweep.curr0}")
    
    Vsweep = pkt.sweep.Vsweep
    V_sig = pkt.sweep.adc1Sig
    # curr = pkt.sweep.curr0

    curr0 = pkt.sweep.curr0
    curr20 = pkt.sweep.curr20
    curr400 = pkt.sweep.curr400

    sig0 = pkt.sweep.adc1Sig
    sig20 = pkt.sweep.adc2Sig
    sig400 = pkt.sweep.adc3Sig

    currs = [curr0, curr20, curr400]
    sigs = [sig0, sig20, sig400]
    gains = [0, 20, 400]

    plt.figure()
    for i in range(len(currs)):
        makeIVPlot(Vsweep, currs[i], i + 1, gains[i], sweepNum, pkt)

    plt.figure()
    for i in range(len(sigs)):
        makeVPlot(Vsweep, sigs[i], i + 1, gains[i])


    plt.show()

    print(pkt.tInitial)
    print(pkt.tFinal)