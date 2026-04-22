from struct import unpack
import csv
from pathlib import Path

typeSens  = 0x01
typeSweep = 0x10

NUM_STEPS = 356

lenHedr  = 15
lenSens  = 24


BASE_DIR = Path(__file__).resolve().parent


# ─────────────────────────────────────────
# MAIN ENTRY POINT
# ─────────────────────────────────────────
def process_dat_file(dat_path: Path):

    try:
        dat_number = dat_path.stem[-4:]

        sensor_csv = f"sensor_packets_{dat_number}.csv"
        sweep_csv  = f"sweep_packets_{dat_number}.csv"

        folder_sensor = BASE_DIR / "CSV-DATA" / "SENSOR"
        folder_sweep  = BASE_DIR / "CSV-DATA" / "SWEEP"

        folder_sensor.mkdir(parents=True, exist_ok=True)
        folder_sweep.mkdir(parents=True, exist_ok=True)

        file_path_sensor = folder_sensor / sensor_csv
        file_path_sweep  = folder_sweep / sweep_csv

        packetList = readFile(dat_path)

        sensorPackets = []
        sweepPackets  = []

        for pkt in packetList:
            if isinstance(pkt, sensorPacket):
                sensorPackets.append(pkt)
            elif isinstance(pkt, sweepPacket):
                sweepPackets.append(pkt)

        write_packets_to_csv(
            sensorPackets,
            sweepPackets,
            file_path_sensor,
            file_path_sweep
        )

        return dat_number

    except Exception as e:
        print(f"[ERROR] process_dat_file failed: {e}")
        return None

class Packet:
    totCnt = 0

    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        Packet.totCnt += 1
        self.count    = count
        self.tInitial = tInitial   # offset_start from header (abs file offset)
        self.tFinal   = tFinal     # offset_end from header (abs file offset)
        self.pcktType = pcktType
        self.pyldLen  = pyldLen
        self.pyld     = pyld


class sensorPacket(Packet):

    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        super().__init__(count, tInitial, tFinal, pcktType, pyldLen, pyld)

        self.accel_M = None
        self.accel_H = None
        self.gyro_M  = None
        self.mag_M   = None
        self.temp    = None
        

        self.readPyld()

    def readPyld(self):
        self.accel_M = unpack('<hhh', self.pyld[0:6])
        self.accel_H = unpack('<h',   self.pyld[6:8])[0]
        self.gyro_M  = unpack('<hhh', self.pyld[8:14])
        self.mag_M   = unpack('<hhh', self.pyld[14:20])
        self.temp    = unpack('<h',   self.pyld[20:22])[0]
        

class sweepPacket(Packet):

    def __init__(self, count, tInitial, tFinal, pcktType, pyldLen, pyld):
        super().__init__(count, tInitial, tFinal, pcktType, pyldLen, pyld)

        self.v_ref = []    # ADC readback of DAC output (was v_A)
        self.adc_A = []    # ADC channel A             (was i_A)
        self.adc_B = []    # ADC channel B             (was i_B)

        self.readPyld()

    def readPyld(self):
        offset = 4  # skip SWP_REF_LEN
        for step in range(NUM_STEPS):
            v_ref = unpack('<H', self.pyld[offset:offset+2])[0]
            adc_a = unpack('<H', self.pyld[offset+2:offset+4])[0]
            adc_b = unpack('<H', self.pyld[offset+4:offset+6])[0]
            self.v_ref.append(v_ref)
            self.adc_A.append(adc_a)
            self.adc_B.append(adc_b)
            offset += 6


def readFile(fileName):

    packets = []

    with open(fileName, "rb") as f:
        raw = f.read()

    loc = 0

    while loc < len(raw) - lenHedr:

        # Validate sync
        if raw[loc] != 0x55 or raw[loc+1] != 0x44:
            print(f"[WARN] Bad sync at 0x{loc:08X}, skipping byte")
            loc += 1
            continue

        count    = unpack('<H', raw[loc+2:loc+4])[0]
        tInitial = unpack('<I', raw[loc+4:loc+8])[0]   # offset_start
        tFinal   = unpack('<I', raw[loc+8:loc+12])[0]  # offset_end
        pcktType = unpack('<B', raw[loc+12:loc+13])[0]
        pyldLen  = unpack('<H', raw[loc+13:loc+15])[0]
        print(f"[DEBUG] count={count} type=0x{pcktType:02X} pyldLen={pyldLen} loc=0x{loc:08X}")
        pyldStart = loc + lenHedr
        pyldEnd   = pyldStart + pyldLen

        if pyldEnd > len(raw):
            print(f"[WARN] Truncated packet at 0x{loc:08X}, stopping")
            break

        pyld = raw[pyldStart:pyldEnd]

        if pcktType == typeSens:
            packet = sensorPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld)
            packets.append(packet)

        elif pcktType == typeSweep:
            packet = sweepPacket(count, tInitial, tFinal, pcktType, pyldLen, pyld)
            packets.append(packet)

        else:
            print(f"[WARN] Unknown type 0x{pcktType:02X} at 0x{loc:08X}")

        loc += lenHedr + pyldLen

    print(f"[INFO] Parsed {len(packets)} packets")
    return packets


def write_packets_to_csv(sensorPackets, sweepPackets, file_path_sensor, file_path_sweep):

    with open(file_path_sensor, "w", newline="") as sf:
        writer = csv.writer(sf)
        writer.writerow([
            "count", "tInitial", "tFinal",
            "accel_x", "accel_y", "accel_z",
            "accel_high",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z",
            "temp"
        ])

        for pkt in sensorPackets:
            writer.writerow([
                pkt.count,
                pkt.tInitial,
                pkt.tFinal,
                pkt.accel_M[0],
                pkt.accel_M[1],
                pkt.accel_M[2],
                pkt.accel_H,
                pkt.gyro_M[0],
                pkt.gyro_M[1],
                pkt.gyro_M[2],
                pkt.mag_M[0],
                pkt.mag_M[1],
                pkt.mag_M[2],
                pkt.temp
            ])

    with open(file_path_sweep, "w", newline="") as wf:
        writer = csv.writer(wf)
        writer.writerow([
            "count", "tInitial", "tFinal",
            "step", "v_ref", "adc_A", "adc_B"
        ])

        for pkt in sweepPackets:
            n = min(NUM_STEPS, len(pkt.v_ref))

            for step in range(n):
                writer.writerow([
                    pkt.count,
                    pkt.tInitial,
                    pkt.tFinal,
                    step,
                    pkt.v_ref[step],
                    pkt.adc_A[step],
                    pkt.adc_B[step]
                ])

    print("CSV files written:")
    print(file_path_sensor)
    print(file_path_sweep)
