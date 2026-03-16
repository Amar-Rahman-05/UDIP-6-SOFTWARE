/*
  University of Delaware Ionosphere Probe
  Department of Physics and Astronomy
  Team 2025-2026

  --------------------------------------------------
  Display documentation
  0x00 - Phase 1 of flight, (T-180: T+87). 
  0x01 - Phase 2 of flight, (T+87: T+400 *TBD IDK*). 
  0x03 - SD card failed to initialize.
  0x04 - Mid range IMU failed to initialize.
  0x05 - High range accel failed to initialize.

  Notes for future (2/2/26):
  -get values from electrical for DAC sweep array
*/

/*Includes*/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>

/*Define Display*/
#define A 41
#define B 39
#define C 51
#define D 49
#define E 47
#define F 43
#define G 45
#define DP 53 // decimal
uint32_t dotTime = 0;
bool dotState = 1; //1 is decimal on, 0 is decimal off
int seg[] {A, B, C, D, E, F, G, DP};
byte Chars[17][7] {
  {1, 1, 1, 1, 1, 1, 0}, //0
  {0, 1, 1, 0, 0, 0, 0}, //1
  {1, 1, 0, 1, 1, 0, 1}, //2
  {1, 1, 1, 1, 0, 0, 1}, //3
  {0, 1, 1, 0, 0, 1, 1}, //4
  {1, 0, 1, 1, 0, 1, 1}, //5
  {1, 0, 1, 1, 1, 1, 1}, //6
  {1, 1, 1, 0, 0, 0, 0}, //7
  {1, 1, 1, 1, 1, 1, 1}, //8
  {1, 1, 1, 1, 0, 1, 1}, //9
  {1, 1, 1, 0, 1, 1, 1}, //A/10
  {0, 0, 1, 1, 1, 1, 1}, //b/11
  {1, 0, 0, 1, 1, 1, 0}, //C/12
  {0, 1, 1, 1, 1, 0, 1}, //d/13
  {1, 0, 0, 1, 1, 1, 1}, //E/14
  {1, 0, 0, 0, 1, 1, 1}, //F/15
  {0, 0, 0, 0, 0, 0, 0}  //blank
};

/*Packet Types -- Sensor and Sweep Packets*/
#define TYPE_SENS 0x01
#define TYPE_SWP 0x10

/*Packet Headers*/
#define HEDR_POS_SYNC       0
#define HEDR_POS_COUNT      2
#define HEDR_POS_T_INITIAL  4
#define HEDR_POS_T_FINAL    8
#define HEDR_POS_TYPE      12
#define HEDR_POS_PYLD_LEN  13
#define HEDR_LEN           15

/*Packet Headers -- Sync word*/
#define PCKT_SYNC_0 0x55 //ASCII 'U'
#define PCKT_SYNC_1 0x44 //ASCII 'D'

/*Sensor Payload*/
#define SENS_POS_ACCEL_M      0 //3 two byte signed ints. Mid range accels
#define SENS_POS_ACCEL_H      6 //1 two byte signed int. High range accels
#define SENS_POS_GYRO         8 //3 two byte signed ints. Gyroscopes
#define SENS_POS_MAG         14 //3 two byte signed ints. Magnetometers
#define SENS_POS_TMP_D       20 //1 two byte unsigned int. Digital Board Temp

/*Sweep Reference Payload*/
#define SWP_POS_A       0 //two byte unsigned int for probe A reading
#define SWP_POS_B       2 //two byte unsigned int for probe B reading
#define SENS_POS_PHOTO  4 //2 two byte signed ints. Photodiodes
#define SWP_REF_LEN     8 //1 two byte unsigned int for reference payload length

/*Sweep Packet ADC positions -- CHECK W/ ELECTRICAL*/
#define SWP_POS_SWP_V       0 //two byte unsigned int for reading of the voltage from DAC
#define SWP_POS_ADC_VREF    2 //two byte unsigned int for reading of voltage ADC
#define SWP_POS_ADC_A       4 //two byte unsigned int for reading of probe A ADC
#define SWP_POS_ADC_B       6 //two byte unsigned int for reading of probe B ADC

#define SWP_ADC_LEN         8 //two byte unsigned int for how many ADC readings are taken in each sweep step

/*Arduino Due Pins for Voltage Sweep*/
#define SWEEP_PIN DAC0  //voltage array is written to this pin
#define ADC_SWP A0  //sweep voltage
#define ADC_A A1  //probe A
#define ADC_B A3  //probe B
#define ADC_TEST1 A5 //checks +-12V
#define ADC_TEST2 A6 //checks 12V again

//stepped voltage zero value (in between the dac min and max)
#define ZERO_VOLT_DAC 1737 
//pre-step voltage min and max
#define DAC_MIN 837 //1 VOLT
#define DAC_MAX 2693 // 2 VOLTS

/*Digital Board Temp. Pin --> analog read from pin */
#define PIN_TMP 4

/*Define number of ADCs using for a sweep*/
#define N_SWP_ADC   6 //num. of ADCs used for a sweep (3 sweep measurements + 1 for DAC + 2 check ADCs)

#define N_SWP_STEP  2000 //num. of steps per sweep

const uint16_t swpLen = N_SWP_STEP * SWP_ADC_LEN + SWP_REF_LEN; //length of sweep
#define MAX_SWP_LEN  (N_SWP_STEP * SWP_ADC_LEN)
/*Define SwpDAC[] -- CHECK W/ ELECTRICAL*/
const uint16_t swpDAC[N_SWP_STEP] = {// 0 V → +8 V (89 points, step = +7)
  ZERO_VOLT_DAC,
  1744, 1751, 1758, 1765, 1772, 1779, 1786, 1793, 1800, 1807, 1814, 1821, 1828, 1835, 1842,
  1849, 1856, 1863, 1870, 1877, 1884, 1891, 1898, 1905, 1912, 1919, 1926, 1933, 1940, 1947,
  1954, 1961, 1968, 1975, 1982, 1989, 1996, 2003, 2010, 2017, 2024, 2031, 2038, 2045, 2052,
  2059, 2066, 2073, 2080, 2087, 2094, 2101, 2108, 2115, 2122, 2129, 2136, 2143, 2150, 2157,
  2164, 2171, 2178, 2185, 2192, 2199, 2206, 2213, 2220, 2227, 2234, 2241, 2248, 2255, 2262,
  2269, 2276, 2283, 2290, 2297, 2304, 2311, 2318, 2325, 2332, 2339, 2346, 2353,

  // +8 V → 0 V (89 points, step = –7)
  2353, 2346, 2339, 2332, 2325, 2318, 2311, 2304, 2297, 2290, 2283, 2276, 2269, 2262, 2255,
  2248, 2241, 2234, 2227, 2220, 2213, 2206, 2199, 2192, 2185, 2178, 2171, 2164, 2157, 2150,
  2143, 2136, 2129, 2122, 2115, 2108, 2101, 2094, 2087, 2080, 2073, 2066, 2059, 2052, 2045,
  2038, 2031, 2024, 2017, 2010, 2003, 1996, 1989, 1982, 1975, 1968, 1961, 1954, 1947, 1940,
  1933, 1926, 1919, 1912, 1905, 1898, 1891, 1884, 1877, 1870, 1863, 1856, 1849, 1842, 1835,
  1828, 1821, 1814, 1807, 1800, 1793, 1786, 1779, 1772, 1765, 1758, 1751, 1744, ZERO_VOLT_DAC,

  // 0 V → –8 V (89 points, step = –7)
  ZERO_VOLT_DAC,
  1730, 1723, 1716, 1709, 1702, 1695, 1688, 1681, 1674, 1667, 1660, 1653, 1646, 1639, 1632,
  1625, 1618, 1611, 1604, 1597, 1590, 1583, 1576, 1569, 1562, 1555, 1548, 1541, 1534, 1527,
  1520, 1513, 1506, 1499, 1492, 1485, 1478, 1471, 1464, 1457, 1450, 1443, 1436, 1429, 1422,
  1415, 1408, 1401, 1394, 1387, 1380, 1373, 1366, 1359, 1352, 1345, 1338, 1331, 1324, 1317,
  1310, 1303, 1296, 1289, 1282, 1275, 1268, 1261, 1254, 1247, 1240, 1233, 1226, 1219, 1212,
  1205, 1198, 1191, 1184, 1177, 1170, 1163, 1156, 1149, 1142, 1135, 1128, 1121,

  // –8 V → 0 V (89 points, step = +7)
  1121, 1128, 1135, 1142, 1149, 1156, 1163, 1170, 1177, 1184, 1191, 1198, 1205, 1212, 1219,
  1226, 1233, 1240, 1247, 1254, 1261, 1268, 1275, 1282, 1289, 1296, 1303, 1310, 1317, 1324,
  1331, 1338, 1345, 1352, 1359, 1366, 1373, 1380, 1387, 1394, 1401, 1408, 1415, 1422, 1429,
  1436, 1443, 1450, 1457, 1464, 1471, 1478, 1485, 1492, 1499, 1506, 1513, 1520, 1527, 1534,
  1541, 1548, 1555, 1562, 1569, 1576, 1583, 1590, 1597, 1604, 1611, 1618, 1625, 1632, 1639,
  1646, 1653, 1660, 1667, 1674, 1681, 1688, 1695, 1702, 1709, 1716, 1723, 1730, ZERO_VOLT_DAC};

//SD Card Pin Define
#define SD_CS 10

//Relay Pin Define
#define PIN_RELAY_TE 2

/*Function Headers*/
/*sdInit
  Begins wire connection to SD card
  Create FILE_CNT.DAT on SD card to hold 1 byte of data for num. files on card
  If FILE_CNT.DAT exists --> update fileCount w/ value in count file
  Parameters: none
  Returns: boolean that symbolizes success of reading FILE_CNT.DAT & creating new file
*/
bool sdInit();
/*sdOpen
  Creates a new file with 'UDIP####.DAT" name --> #'s represent fileCount value
  Opens the new file to be written
  Assigns the file to datFile
  Parameters: none
  Returns: void
*/
void sdOpen();
/*makeHedr
  Adds sync word, count, and initial time to parameter packet's header
  When new header is created --> count is incremented
  Parameters: pointer to array of bytes (a packet), pointer to packet count variable
  Returns: void
*/
void makeHedr(byte *, uint16_t *);
/*makeSensPyld
  Reads values from the sensor & 9DOF boards
  Assigns values to associated sensor variables
  Parameters: pointer to array of bytes (a packet)
  Returns: void
*/
void makeSensPyld(byte *);
/*makeSensPckt
  Populates type & length fields of packet header
  Calls makeSensPyld to get sensor readings & add to payload
  Parameters: pointer to array of bytes (a packet), pointer to the count variable
  Returns: void
*/
void makeSensPckt(byte *, uint16_t *);
/*getADC
  Takes 18 ADC readings --> filters out highest and lowest values
  Goal is to reduce outliers & ensure stable data
  Parameters: integer representing an ADC pin
  Returns: unsigned int that represents sum of 16 ADC readings (not incl. min. & max.)
*/
uint16_t getADC(int);
/*makeSweepPckt
  Populates data within a singular sweep packet
  Parameters: pointer to array of bytes (a packet), unsigned int. representing packet count
  Returns: void
*/
void makeSweepPckt(byte *, uint16_t *);
/*doStep -- CHECK W/ ELECTRICAL
  Performs a singular step of the sweep --> writes voltage out, reads in ADC pins
  Parameters: pointer to array of bytes (a packet), unsigned int. representing DAC output, 
  int representing location in the packet
  Returns: void
*/
void doStep(byte *, uint16_t, int);
/*makeSweep
  Sweep function. Calls doStep with appropriate parameters N_SWP_STEP times
  Paramters: pointer to array of bytes (a packet)
  Returns: void 
*/
void makeSweep(byte *pckt);
/*writePckt
  Writes the bytes in the packet to the SD card
  Parameters: file to write to, pointer to array of bytes (a packet), 
  int. representing length of the bytes to write
  Returns: void
*/
void writePckt(File, byte *, uint16_t);
/*displayPrint
  Shows the inputted parameter on the 7-segment display
  Paramters: char representing a digit or symbol to display 
  Returns: void
*/
void displayPrint(char);

/*Global Variables*/
const int senLen = 24;
File datFile;
byte fileCount;
byte sensPckt[HEDR_LEN + senLen];
byte swpPckt[HEDR_LEN + SWP_REF_LEN + MAX_SWP_LEN];
uint16_t count = 0; //total collected packet count
bool is_active = false; //boolean that determines if in phase 2 of flight -- changes at TE relay
unsigned long tInitial;
unsigned long tFinal;
int test = 0;

/*H3LIS331 -- High Range Accelerometer*/
Adafruit_H3LIS331 HighA = Adafruit_H3LIS331();
bool HighAFlag = true;
int16_t acc_h;

/*LSM9DS1 -- Sensor Board*/
Adafruit_LSM9DS1 MidIMU = Adafruit_LSM9DS1();
bool MidIMUFlag = true;
int16_t acc[3];
int16_t gyr[3];
int16_t mag[3];
float tmp;

void setup() {
  is_active = false;
  Serial.begin(9600);
  Serial.println("Starting initialization...");
  while(!Serial) {} //wait for serial monitor

  /*Initialize I2C communication*/
  Wire.begin();
  Wire.setClock(100000); //set I2C to fast-mode (400kHz)

  /*Configure pins for 7-segment display*/
  pinMode(seg[0], OUTPUT);
  pinMode(seg[1], OUTPUT);
  pinMode(seg[2], OUTPUT);
  pinMode(seg[3], OUTPUT);
  pinMode(seg[4], OUTPUT);
  pinMode(seg[5], OUTPUT);
  pinMode(seg[6], OUTPUT);
  pinMode(seg[7], OUTPUT);

  /*Digital board temp sensor init.*/
  pinMode(PIN_TMP, INPUT);
  //TE pin init.
  pinMode(PIN_RELAY_TE, INPUT);

  /*Set analog resolutions to 12 bits*/
  analogWriteResolution(12);
  analogReadResolution(12);

  /*ADC pins init.*/
  pinMode(ADC_SWP, INPUT);
  pinMode(ADC_A, INPUT);
  pinMode(ADC_B, INPUT);
  pinMode(ADC_TEST1, INPUT);
  pinMode(ADC_TEST2, INPUT);

  /*SD card init.*/
  // if (!sdInit()) {
  //   Serial.println("SD card initialization failed.");
  //   displayPrint(0x03);
  //   while (!sdInit()) {
  //     yield();
  //   }
  // }
  //sdOpen();

  /*Mid Range IMU init.*/
  // Serial.println(MidIMU.begin());
  // if (!MidIMU.begin()) {
  //   Serial.println("Mid Range IMU initialization failed.");
  //   displayPrint(0x04);
  //   uint8_t i = 0;
  //   while (!MidIMU.begin() && i < 20) {
  //     delay(1000);
  //     i++;
  //   }
  //   if (i == 20) {
  //     MidIMUFlag = false;
  //   }
  // }
  // if (MidIMUFlag) {
  //   Serial.println("Mid Range IMU initialization success.");
  //   MidIMU.setupAccel(MidIMU.LSM9DS1_ACCELRANGE_16G);
  //   MidIMU.setupMag(MidIMU.LSM9DS1_MAGGAIN_4GAUSS);
  //   MidIMU.setupGyro(MidIMU.LSM9DS1_GYROSCALE_2000DPS);
  // }

  /*High-G Accelerometer Init.*/
  if (!HighA.begin_I2C()) {
    Serial.println("High Range Accel. initialization failed.");
    displayPrint(0x05);
    uint8_t i = 0;
    while (!HighA.begin_I2C() && i < 5) {
      delay(10);
      i++;
    }
    if (i == 5) {
      HighAFlag = false;
    }
  }
  if (HighAFlag) {
    Serial.println("High Range Accel. initialization success.");
    HighA.setRange(H3LIS331_RANGE_100_G);
    HighA.setDataRate(LIS331_DATARATE_1000_HZ);
  }
}

void loop() {
  /*If TE event --> start Phase 2*/
  if ((PIN_RELAY_TE) == LOW) {
    is_active = true;
  }
  /*Phase 1 -- Power on (only collecting sensor data)*/
  if (is_active == false) {
    if (MidIMUFlag && HighAFlag) {
      displayPrint(0x00);
    }
    /*Take high frequency sensor readings & write packet to file*/
    //makeSensPckt(sensPckt, &count);
    //writePckt(datFile, sensPckt, HEDR_LEN + senLen);

    /*TEST CODE*/
    // MidIMU.read();
    // sensors_event_t ac, mg, gy, temp;
    // MidIMU.getEvent(&ac, &mg, &gy, &temp);
    // tmp = analogRead(PIN_TMP);
    // float voltage = (tmp * 3.3) / 4096.0;
    // float tempC = (voltage - 0.5) * 100.0;
    HighA.read();
    //acceleration
    // Serial.println("Acceleration values: \n");
    // Serial.println(int16_t(ac.acceleration.x));
    // Serial.println(int16_t(ac.acceleration.y));
    // Serial.println(int16_t(ac.acceleration.z));
    // Serial.println("\n");
    // Serial.println("Gyroscope values: \n");
    // Serial.println(int16_t(gy.gyro.x * 936.25));
    // Serial.println(int16_t(gy.gyro.y * 936.25));
    // Serial.println(int16_t(gy.gyro.z * 936.25));
    // Serial.println("\n");
    // Serial.println("Magnetomoter values: \n");
    // Serial.println(int16_t(mg.magnetic.x * 409.6));
    // Serial.println(int16_t(mg.magnetic.y * 409.6));
    // Serial.println(int16_t(mg.magnetic.z * 409.6));
    // Serial.println("\n");
    // Serial.println("Temperature values: \n");
    // Serial.println(tempC);
    // Serial.println("~~~~~~~~~~~~~~~~~~~~~~~");
    
    
    // Serial.println("High A values: \n");
    // sensors_event_t high_a;
    // HighA.getEvent(&high_a);
    // Serial.println(high_a.acceleration.z);
    // Serial.println("\n");
    // delay(1000);
  }
  /*Phase 2 -- TE event 1 (alternating collection of sweep and sensor data)*/
  else if (is_active == true) {
    displayPrint(0x01);
    // Serial.println("Starting DAC test");
    // float shuntResistor = 100.0; // ohms
    // if (test != 1) {
    // for (int i = 0; i < N_SWP_STEP; i++) {
    //       analogWrite(SWEEP_PIN, swpDAC[i]);
    //       delay(50);
    //       float adcSweep = getADC(ADC_SWP);
    //       float currentA = getADC(ADC_A);

    //       float voltage = adcSweep * (5.0 / 1023.0);
    //       float current = (currentA * (5.0 / 1023.0)) / shuntResistor;
    //       //currentB = getADC(ADC_B);
    //       Serial.print(voltage);
    //       Serial.println(",");
    //       Serial.println(current);
    //       delay(2000);
    //       test++;
    //   }
    // }

    /*Make sensor and sweep packets*/
    //makeSensPckt(sensPckt, &count);
    makeSweepPckt(swpPckt, &count);

    /*Write packets to SD card*/
    //writePckt(datFile, sensPckt, HEDR_LEN + senLen);
    //writePckt(datFile, swpPckt, HEDR_LEN + swpLen);
  }
}

/*Helper Functions*/
bool sdInit() {
  File cntFile;
  if (!SD.begin(SD_CS)) {
    return false;
  }
  if (SD.exists("FILE_CNT.DATA")) {
    cntFile = SD.open("FILE_CNT.DAT", O_READ);
    cntFile.read(&fileCount, 1);
  }
  else {
    fileCount = 0;
    cntFile = SD.open("FILE_CNT.DAT", O_CREAT | O_TRUNC | O_WRITE);
    cntFile.write(fileCount);
    cntFile.flush();
    cntFile.close();
  }
  return true;
}
void sdOpen() {
  char fileName[13];
  File cntFile;

  sprintf(fileName, "UDIP%04d.DAT", fileCount);
  fileCount++;

  cntFile = SD.open("FILE_CNT.DAT", O_CREAT | O_TRUNC | O_WRITE);
  cntFile.write(fileCount);
  cntFile.flush();
  cntFile.close();

  datFile = SD.open(fileName, O_CREAT | O_TRUNC | O_WRITE );
  return;
}
void makeHedr(byte *pckt, uint16_t *pcktCount) {
  tInitial = millis();
  pckt[HEDR_POS_SYNC] = PCKT_SYNC_0;
  pckt[HEDR_POS_SYNC + 1] = PCKT_SYNC_1;

  memcpy(&pckt[HEDR_POS_COUNT], pcktCount, 2);
  (*pcktCount)++;

  memcpy(&pckt[HEDR_POS_T_INITIAL], &tInitial, 4);
  return;
}
void makeSensPyld(byte *pckt) {
  if (MidIMUFlag) {
    MidIMU.read();
    sensors_event_t ac, mg, gy, temp;
    MidIMU.getEvent(&ac, &mg, &gy, &temp);
    acc[0] = int16_t(ac.acceleration.x * 95.43);
    acc[1] = int16_t(ac.acceleration.y * 95.43);
    acc[2] = int16_t(ac.acceleration.z * 95.43);
    gyr[0] = int16_t(gy.gyro.x * 936.25);
    gyr[1] = int16_t(gy.gyro.y * 936.25);
    gyr[2] = int16_t(gy.gyro.z * 936.25);
    mag[0] = int16_t(mg.magnetic.x * 409.6);
    mag[1] = int16_t(mg.magnetic.y * 409.6);
    mag[2] = int16_t(mg.magnetic.z * 409.6);
    tmp = analogRead(PIN_TMP);
  }
  else {
    acc[0] = 0xffff; acc[1] = 0xffff; acc[2] = 0xffff;
    gyr[0] = 0xffff; gyr[1] = 0xffff; gyr[2] = 0xffff;
    mag[0] = 0xffff; mag[1] = 0xffff; mag[2] = 0xffff;
  }
  if (HighAFlag) {
    HighA.read();
    sensors_event_t high_a;
    HighA.getEvent(&high_a);
    acc_h = int16_t(high_a.acceleration.z * 33.4);
  }
  else {
    acc_h = 0xffff;
  }
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_M], &acc, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_H], &acc_h, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_GYRO], &gyr, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_MAG], &mag, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_D], &tmp, 2);

  tFinal = millis();
  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);
  return;
}
void makeSensPckt(byte *pckt, uint16_t *count) {
  makeHedr(pckt, count);
  pckt[HEDR_POS_TYPE] = TYPE_SENS;
  memcpy(&pckt[HEDR_POS_PYLD_LEN], &senLen, 2);
  makeSensPyld(pckt);
  tFinal = millis();
  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);
  return;
}
uint16_t getADC(int ADCpin) {
  uint16_t val = 0;
  uint32_t val_sum = 0;
  uint16_t val_max = 0;
  uint16_t val_min = 0xffff;
  for (int i = 0; i < 18; i++) {
    val = analogRead(ADCpin);
    if (val < val_min) {
      val_min = val;
    }
    if (val > val_max) {
      val_max = val;
    }
    val_sum += val;
  }
  val_sum -= (val_max + val_min);
  return (uint16_t(val_sum));
}
void makeSweepPckt(byte *pckt, uint16_t *count) {
  makeHedr(pckt, count);
  pckt[HEDR_POS_TYPE] = TYPE_SWP;
  memcpy(&pckt[HEDR_POS_PYLD_LEN], &swpLen, 2);
  makeSweep(pckt);
  tFinal = millis();
  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);
  return;
}
void doStep(byte *pckt, uint16_t dacLevel, int loc) {
  uint16_t adcSweep = 0;
  uint16_t adcA = 0;
  uint16_t adcB = 0;

  //send analog voltage out
  analogWrite(SWEEP_PIN, dacLevel);
  //read ADC outputs
  adcSweep = getADC(ADC_SWP);
  adcA = getADC(ADC_A);
  adcB = getADC(ADC_B);

  memcpy(&pckt[HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_SWP_V], &adcSweep, 2);
  memcpy(&pckt[HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_ADC_A], &adcA, 2);
  memcpy(&pckt[HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_ADC_B], &adcB, 2);
}
void makeSweep(byte *pckt){
  for (int i = 0; i < N_SWP_STEP; i++) {
    doStep(pckt, swpDAC[i], i * SWP_ADC_LEN);
  }
  return;
}

void writePckt(File f, byte *pckt, uint16_t pcktLen) {
  f.write(pckt, pcktLen);
  return;
}
void displayPrint(char n) {
  for (char i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[n][i]);
  }
}
