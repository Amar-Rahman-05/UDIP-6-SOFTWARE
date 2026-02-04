#UDIP-6 GROUND CONVERSION


from struct import unpack

import csv

from datetime import datetime

typeSens = 0x01
typeSweep  = 0x10

NUM_STEPS = 356

lenHedr = 15

lenSens = 24
lenMed = NUM_STEPS * 8 + 4
fileName = "UDIP0087.DAT"

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
sensor_csv = f"sensor_packets_{timestamp}.csv"
sweep_csv  = f"sweep_packets_{timestamp}.csv"


"""REQUIRED PACKETS
    Packet (Parent)
    Sensor Packet
    Sweep Packet 

    read raw data from bin file
    iterate through raw data
    use raw data to create packets 
    sort packets to senspacketlist and sweeppacketlist    

    iterate through packetlist and senspacketlist and sweeppacketlist
    write from packetlist to packetcsv  
    write from senspacketlist to senscsv 
    write from sweeppacketlist to sweepcsv    
"""

class Packet:
    totCnt = 0
    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        Packet.totCnt += 1
        self.count = count
        self.tInitial = tInitial
        self.tFinal = tFinal
        self.pcktType = pcktType
        self.pyldLen = pyldLen
        self.pyld = pyld


class sensorPacket(Packet):
    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        super().__init__(count, tInitial, tFinal, pcktType, pyldLen, pyld)
        self.accel_M = None 
        self.accel_H = None 
        self.gyro_M = None 
        self.mag_M = None 
        self.temp = None 
        self.photo = None
        self.readPyld()

    def readPyld(self):
        self.accel_M = unpack('<hhh', self.pyld[0:6])      
        self.accel_H = unpack('<hhh', self.pyld[6:12])    #CHECK FORMAT OF ACCEL_HIGH RANGE, IS IT [X,Y,Z] or just an int?
        self.gyro_M  = unpack('<hhh', self.pyld[12:18])   
        self.mag_M   = unpack('<hhh', self.pyld[18:24])    
        self.temp    = unpack('<h', self.pyld[24:26])[0]  
        self.photo  = unpack('<h', self.pyld[26:28])[0]  
       
    
class sweepPacket(Packet):
    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        super().__init__(count, tInitial, tFinal, pcktType, pyldLen, pyld)
        self.v_A = None 
        self.v_B = None 
        self.i_A = None 
        self.i_B = None 
        self.readPyld()

    def readPyld(self):
        self.v_A = unpack('<h',self.pyld[0:2])[0] 
        self.v_B = unpack("<h", self.pyld[2:4])[0] 
        self.i_A = unpack('<h', self.pyld[4:6])[0]
        self.i_B = unpack('<h', self.pyld[8:10])[0] 


def readFile(fileName):
    myPackets = []
    myFile = open(fileName,"rb")
    raw = myFile.read()
    loc = 0
    lenHedr = 15
    while(loc < len(raw)):
        sync = unpack('<BB', raw[loc:loc+2])
        count    = unpack('<H', raw[loc + 2:loc + 4])[0]
        tInitial = unpack('<I', raw[loc + 4:loc + 8])[0]
        tFinal   = unpack('<I', raw[loc + 8:loc + 12])[0]
        pcktType = unpack('<B', raw[loc + 12:loc + 13])[0]
        pyldLen  = unpack('<H', raw[loc + 13:loc + 15])[0]
        pyldStart = loc + lenHedr
        pyldEnd   = pyldStart + pyldLen

        if pyldEnd > len(raw):
            break

        pyld = raw[pyldStart:pyldEnd]

        if(pcktType == typeSens):
                packet = sensorPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld)
                myPackets.append(packet)
        elif(pcktType == typeSweep):
                packet = sweepPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld)
                myPackets.append(packet)

        loc += lenHedr + pyldLen

    return myPackets

def verifyHeader(sync, pcktType, pyldLen):
    if(sync[0] != 0x55 or sync[1] != 0x44):
        return False
    if(pcktType == typeSens):
        if(pyldLen != lenSens):
            return False
        else:
            return True
    if(pcktType == typeSweep):
        if(pyldLen != lenMed):
            return False
        else:
            return True


packetList = readFile(fileName)
sensPacketList= []
sweepPacketList= []

for pkt in packetList:
     if isinstance(pkt, sensorPacket):
          sensPacketList.append(pkt)           

     elif isinstance(pkt, sweepPacket):
          sweepPacketList.append(pkt) 

          
          
def write_packets_to_csv(packets):
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    sensor_csv = f"sensor_packets_{timestamp}.csv"
    sweep_csv  = f"sweep_packets_{timestamp}.csv"

    with open(sensor_csv, "w", newline="") as sf, open(sweep_csv, "w", newline="") as wf:
        sensor_writer = csv.writer(sf)
        sweep_writer  = csv.writer(wf)
        sensor_writer.writerow([
            "count", "tInitial", "tFinal",
            "accel_M_x", "accel_M_y", "accel_M_z",
            "accel_H_x", "accel_H_y", "accel_H_z",
            "gyro_M_x", "gyro_M_y", "gyro_M_z",
            "mag_M_x", "mag_M_y", "mag_M_z",
            "temp", "photo"
        ])

        sweep_writer.writerow([
            "count", "tInitial", "tFinal",
            "v_A",
            "v_B",
            "i_A",
            "i_B"
        ])

        for pkt in sensPacketList:
            sensor_writer.writerow([pkt.count, pkt.tInitial,  pkt.tFinal, *pkt.accel_M, *pkt.accel_H, *pkt.gyro_M, *pkt.mag_M, pkt.temp, pkt.photo])

        for pkt in sweepPacketList:
            sweep_writer.writerow([pkt.count, pkt.tInitial, pkt.tFinal, pkt.v_A, pkt.v_B, pkt.i_A, pkt.i_B])

    print("CSV files written:")
    print(sensor_csv)
    print(sweep_csv)
          
     

