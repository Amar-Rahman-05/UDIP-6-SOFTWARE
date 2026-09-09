from struct import unpack
import csv
from pathlib import Path
import numpy as np

typeSens  = 0x01
typeSweep = 0x10
NUM_STEPS = 356
lenHedr   = 15
ZERO_VOLT_DAC = 1737

BASE_DIR = Path(__file__).resolve().parent

class Packet:
    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld, global_id):
        self.count     = count
        self.tInitial  = tInitial
        self.tFinal    = tFinal
        self.pcktType  = pcktType
        self.pyldLen   = pyldLen
        self.pyld      = pyld
        self.global_id = global_id  # This tracks the absolute order in the file

class sensorPacket(Packet):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.readPyld()

    def readPyld(self):
        # Unpack sensor data (14 bytes of IMU/Mag + 2 bytes Temp)
        self.accel_M = unpack('<hhh', self.pyld[0:6])
        self.accel_H = unpack('<h',   self.pyld[6:8])[0]
        self.gyro_M  = unpack('<hhh', self.pyld[8:14])
        self.mag_M   = unpack('<hhh', self.pyld[14:20])
        self.temp    = unpack('<h',   self.pyld[20:22])[0]

class sweepPacket(Packet):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.v_refA = []
        self.v_refB = []
        self.adc_A  = []
        self.adc_B  = []
        self.readPyld()

    def readPyld(self):
        # 1. extract all v_refA to calculate the mean
        temp_vrefA = []
        offset = 4  # skip header
        for i in range(NUM_STEPS):
            v_a = unpack('<H', self.pyld[offset : offset+2])[0]
            temp_vrefA.append(v_a)
            offset += 6
        
        # 2. Reset offset and populate all lists in one pass
        offset = 4
        for i in range(NUM_STEPS):
            v_a = temp_vrefA[i]
            adc_a = unpack('<H', self.pyld[offset+2 : offset+4])[0]
            adc_b = unpack('<H', self.pyld[offset+4 : offset+6])[0]
            
            self.v_refA.append(v_a)
            
            self.v_refB.append(int((2 * ZERO_VOLT_DAC) - v_a))
            
            self.adc_A.append(adc_a)
            self.adc_B.append(adc_b)
            
            offset += 6

def readFile(fileName):
    packets = []
    master_packet_counter = 0  # global counter
    
    with open(fileName, "rb") as f:
        raw = f.read()

    loc = 0
    while loc < len(raw) - lenHedr:
        if raw[loc] != 0x55 or raw[loc+1] != 0x44:
            loc += 1
            continue

        # After packet is found, increment the global sequence ID
        master_packet_counter += 1

        count    = unpack('<H', raw[loc+2:loc+4])[0]
        tInitial = unpack('<I', raw[loc+4:loc+8])[0]
        tFinal   = unpack('<I', raw[loc+8:loc+12])[0]
        pcktType = unpack('<B', raw[loc+12:loc+13])[0]
        pyldLen  = unpack('<H', raw[loc+13:loc+15])[0]
        
        pyldStart = loc + lenHedr
        pyldEnd   = pyldStart + pyldLen
        if pyldEnd > len(raw): break
        
        pyld = raw[pyldStart:pyldEnd]

        if pcktType == typeSens:
            packets.append(sensorPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld, master_packet_counter))
        elif pcktType == typeSweep:
            packets.append(sweepPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld, master_packet_counter))
        
        loc += lenHedr + pyldLen

    return packets

def write_packets_to_csv(sensorPackets, sweepPackets, file_path_sensor, file_path_sweep):
    # Sensor File: includes 'packet_num' which corresponds to global file order
    with open(file_path_sensor, "w", newline="") as sf:
        writer = csv.writer(sf)
        writer.writerow(["count", "tInitial", "tFinal", "accel_x", "accel_y", "accel_z", "accel_high", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z", "temp", "packet_num"])
        for pkt in sensorPackets:
            writer.writerow([pkt.count, pkt.tInitial, pkt.tFinal, *pkt.accel_M, pkt.accel_H, *pkt.gyro_M, *pkt.mag_M, pkt.temp, pkt.global_id])

    # Sweep File: 'packet_num' stays the same for all 356 steps of a single sweep
    with open(file_path_sweep, "w", newline="") as wf:
        writer = csv.writer(wf)
        writer.writerow(["count", "tInitial", "tFinal", "step", "v_refA", "v_refB", "adc_A", "adc_B", "packet_num"])
        for pkt in sweepPackets:
            for step in range(min(NUM_STEPS, len(pkt.v_refA))):
                writer.writerow([pkt.count, pkt.tInitial, pkt.tFinal, step, pkt.v_refA[step], pkt.v_refB[step], pkt.adc_A[step], pkt.adc_B[step], pkt.global_id])

def process_dat_file(dat_path: Path):
    try:
        # Extract the 4-digit ID (e.g., '0096')
        dat_number = dat_path.stem[-4:]
        
        # Read and parse the binary data
        packetList = readFile(dat_path)
        
        # Separate packets by type
        s_pkts = [p for p in packetList if isinstance(p, sensorPacket)]
        w_pkts = [p for p in packetList if isinstance(p, sweepPacket)]

        # Updated file paths to match your naming convention: sweep_packets_00xx.csv
        out_sens = BASE_DIR / "CSV-DATA" / "SENSOR" / f"sensor_packets_{dat_number}.csv"
        out_swp  = BASE_DIR / "CSV-DATA" / "SWEEP" / f"sweep_packets_{dat_number}.csv"
        
        # Ensure directories exist
        out_sens.parent.mkdir(parents=True, exist_ok=True)
        out_swp.parent.mkdir(parents=True, exist_ok=True)

        # Write to CSV
        write_packets_to_csv(s_pkts, w_pkts, out_sens, out_swp)
        
        print(f"[SUCCESS] Global sequence maintained. Last packet ID: {len(packetList)}")
        
        # RETURN the dat_number so the GUI can proceed to the next page
        return dat_number

    except Exception as e:
        print(f"[ERROR] process_dat_file failed: {e}")
        return None
