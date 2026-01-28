/*
  University of Delaware Ionosphere Probe
  Department of Physics and Astronomy
  Team 2024-2025

  Removed mentions of PD2, and PD1
  Implemented: 
  - functionality sending telemetry data 
  - phase 1 and 2 activated off timer event calls rather than relays

  -------------------------------------------------------

  Display documentation
  0x00 - Phase 1 of flight, (T-180: T+87). 
  0x01 - Phase 2 of flight, (T+87: T+400 *TBD IDK*). 
  0x03 - SD card failed to initialize.
  0x04 - Mid range IMU failed to initialize.
  0x05 - High range accel failed to initialize.
*/

//Declare Includes
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>

//Define Display
#define A 41
#define B 39
#define C 51
#define D 49
#define E 47
#define F 43
#define G 45
#define DP 53 // decimal
uint32_t dotTime = 0;
bool dotState = 1;
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

//Define Packet Types
#define TYPE_SENS 0x01
#define TYPE_MED_SWP 0x10

//Packet Header Positions
#define HEDR_POS_SYNC      0
#define HEDR_POS_COUNT     2
#define HEDR_POS_T_INITIAL 4
#define HEDR_POS_T_FINAL   8
#define HEDR_POS_TYPE     12
#define HEDR_POS_PYLD_LEN 13

#define HEDR_LEN 15

#define PCKT_SYNC_0 0x55 //ASCII 'U'
#define PCKT_SYNC_1 0x44 //ASCII 'D'


//Sensor Payload Positions
#define SENS_POS_ACCEL_M      0 //3 two byte signed ints. Mid range accels
#define SENS_POS_ACCEL_H      6 //1 two byte signed int. High range accels
#define SENS_POS_GYRO         8 //3 two byte signed ints. Gyroscopes
#define SENS_POS_MAG         14 //3 two byte signed ints. Magnetometers
#define SENS_POS_TMP_A       20 //1 two byte unsigned int. analog board temp
#define SENS_POS_TMP_D       22 //1 two byte unsigned int. digital board temp

const int senLen = 24;

//Sweep Reference Payload Positions (Range)
//-12 -> +12 is always gonna be that way because of the PDU (max reading)
#define SWP_POS_VPOS          0 //two byte unsigned int for reading of +12 voltage line
#define SWP_POS_VNEG          2 //two byte unsigned int for reading of -12 voltage line
//Sweep Reference length
#define SWP_REF_LEN           4 //used to be 8 in old code but changed to 4 to account for no need of photodiode

//Sweep ADC Postions
#define SWP_POS_SWP_V         0 //two byte unsigned int for reading of the voltage from DAC
#define SWP_POS_ADC1          2 //two byte unsigned int for reading of the 1 gain ADC
#define SWP_POS_ADC2          4 //two byte unsigned int for reading of the 2 gain ADC
#define SWP_POS_ADC3          6 //two byte unsigned int for reading of the 3 gain ADC

#define SWP_ADC_LEN           8

//Sweep Packet DAC Value Arrays
//updated with nate 4/15
//stepped voltage zero value (in between the dac min and max)
#define ZERO_VOLT_DAC 1737 
//pre-step voltage min and max
#define DAC_MIN 837 //1 VOLT
#define DAC_MAX 2693 // 2 VOLTS

//Define Arduino Due pins for digital port systems
//updated with nate 4/15

//Define Arduino Due Pins for Voltage Sweep // TBD!!
//updated with nate 4/15
#define SWEEP_PIN DAC0 //this sweep pin is what the voltage array is getting written to, outputting +1V->+2V (around 1 like 1.3)
#define ADC_G1 A1  //1 Gain
#define ADC_G2 A2  //2 Gain
#define ADC_G3 A3  //3 Gain
#define ADC_SWP A0 //Sweep voltage //after shifted and is between -12V -> +12V

//updated with nate 4/15
#define ADC_VPOS A6 //+12V reference ADC
#define ADC_VNEG A5 //-12V reference ADC

//Define number of ADC we are using for sweep: 3 for current measurements
//and 1 for DAC measurements   
//updated with nate 4/15
#define N_SWP_ADC 4

//#define N_STEP_AVG 10   TBD : TOOK THIS OUT BECAUSE IT DOESNT GET USED IN THE OLD CODE AND I DONT THINK WE NEED IT



const uint16_t medSwpLen = N_MED_SWP_STEP * SWP_ADC_LEN + SWP_REF_LEN;
#define MAX_SWP_LEN  (N_MED_SWP_STEP * SWP_ADC_LEN)
const uint16_t medSwpDAC[N_MED_SWP_STEP] = {
  // 0 V → +8 V (89 points, step = +7)
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
  1646, 1653, 1660, 1667, 1674, 1681, 1688, 1695, 1702, 1709, 1716, 1723, 1730, ZERO_VOLT_DAC
};



//LSM9DS1
Adafruit_LSM9DS1 MidIMU = Adafruit_LSM9DS1();
bool MidIMUFlag = true;
//Adafruit_LSM9DS1 MidIMU; //Sensor object for LSM9DS1
int16_t acc[3];
int16_t gyr[3];
int16_t mag[3];
int16_t temp_1;

//H3LIS331
Adafruit_H3LIS331 HighA = Adafruit_H3LIS331();
bool HighAFlag = true;
int16_t acc_h;

//TMP36
// removed ADC_TR1 (rearrange traces of this variable) (4/15)
//updated with nate (4/15)
// #define TempAddr 0b01001000
#define PIN_TMP A4
uint16_t tmpInput;

//SD PIN
//updated with kirin (4/15)
#define SD_CS 10

// Relay Setup // TBD (DO WE NEED RELAY???)
#define PIN_RELAY_TE 2   //TE-A/B, Stars at High, Gets turned to LOW and then Phase 2 begins

//Function Declarations
//SD Functions
bool sdInit();
void sdOpen();

//Packet Functions
void makeHedr(byte *, uint16_t *);
void writePckt(File, byte *, uint16_t);
// for sending telemtry
void sendPckt(byte *pckt, uint16_t pcktLen);

//Sensor Packet Functions
void makeSensPckt(byte *, uint16_t *);
void makeSensPyld(byte *);

//Sweep Packet Functions
void makeSweepPckt(byte *, uint16_t *, byte);
void makeMedSweep(byte *);
void addVoltageRef(byte *);
void doStep(byte *, uint16_t, int);

//Global Variables
File datFile;
byte fileCount;
byte sensPckt[HEDR_LEN + senLen];
byte swpPckt[HEDR_LEN + SWP_REF_LEN + MAX_SWP_LEN]; //Adds 4 bytes for voltage references at start of sweep
uint16_t count = 0;
bool is_active = false;
unsigned long tInitial;
unsigned long tFinal;

void setup() {
  // Initialize I2C communication
  //test
  is_active = false;
  Serial1.begin(115200);
  Serial.begin(9600);
  while (!Serial) {}  // Wait for serial monitor
  Serial.println("Starting initialization...");
  //test^
  Wire.begin();
  Wire.setClock(100000);  // Set I2C to fast-mode (400kHz)
//  pinMode(SCL, INPUT_PULLUP);
  //pinMode(SDA, INPUT_PULLUP);
  //Display setup
  //Serial.println("\nRunning I2C scanner...");
  // //TESTING STARTS HERE
  // //
  // byte error, address;
  // int nDevices = 0;

  // for(address = 1; address < 127; address++ ) {
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();
    
  //   if (error == 0) {
  //     Serial.print("I2C device found at 0x");
  //     if (address<16) Serial.print("0");
  //     Serial.print(address,HEX);
  //     Serial.println("  !");
  //     nDevices++;
  //   }
  // }
  
  // if (nDevices == 0) {
  //   Serial.println("No I2C devices found!");
  //   displayPrint(0x04);  // Show IMU error if no devices
  //   displayPrint(0x05);  // Show accelerometer error
  // }
  // else {
  //   Serial.println("I2C scan complete");
  // }

  // delay(5000);  // Give time to read results
  // //TESTING ENDS HERE
  // //
  // //
  
  pinMode(seg[0], OUTPUT);
  pinMode(seg[1], OUTPUT);
  pinMode(seg[2], OUTPUT);
  pinMode(seg[3], OUTPUT);
  pinMode(seg[4], OUTPUT);
  pinMode(seg[5], OUTPUT);
  pinMode(seg[6], OUTPUT);
  pinMode(seg[7], OUTPUT);

  //Digital board temp sensor setup
  pinMode(PIN_TMP, INPUT);
  
  //TE PINMODE SETUP
  pinMode(PIN_RELAY_TE, INPUT);

  //Set analog resolutions to 12 bits 0-4095
  analogWriteResolution(12);
  analogReadResolution(12);

  //Set pinmodes of ADC pins
  pinMode(ADC_G1, INPUT);
  pinMode(ADC_G2, INPUT);
  pinMode(ADC_G3, INPUT);
  pinMode(ADC_SWP, INPUT);
  pinMode(ADC_VPOS, INPUT);
  pinMode(ADC_VNEG, INPUT);

  if (!sdInit()) {
    Serial.println("sd card initialization failed");
    displayPrint(0x03);
    while (!sdInit()) {
      yield();
    }
  }
  sdOpen();

  // Initialize Mid Range IMU
  if (!MidIMU.begin()) {
    Serial.println("Mid Range IMU Failed to start.");
    displayPrint(0x04);
    uint8_t i = 0;
    while (!MidIMU.begin() && i < 5) { // Corrected '&' to '&&'
      delay(10);
      i++;
    }
    if (i == 5) {
      MidIMUFlag = false;
    }
  }
  if (MidIMUFlag) {
    MidIMU.setupAccel(MidIMU.LSM9DS1_ACCELRANGE_16G);
    MidIMU.setupMag(MidIMU.LSM9DS1_MAGGAIN_4GAUSS);
    MidIMU.setupGyro(MidIMU.LSM9DS1_GYROSCALE_2000DPS);
  }

  // Initialize High-G Accelerometer
  if (!HighA.begin_I2C()) { // Ensure correct I2C address if different
    Serial.println("High Range accel fail to initialize");
    displayPrint(0x05);
    uint8_t i = 0;
    while (!HighA.begin_I2C() && i < 5) { // Corrected '&' to '&&'
      delay(10);
      i++;
    }
    if (i == 5) {
      HighAFlag = false;
    }
  }
  if (HighAFlag) {
    HighA.setRange(H3LIS331_RANGE_100_G);
    HighA.setDataRate(LIS331_DATARATE_1000_HZ);
  }
}


uint8_t sciCount = 0;
void loop() {
  uint32_t currentTime = millis();

  // Blinks dot on and off every 256 measuring cycles
  if (sciCount % 8 <= 4) { //would be digital read to update flag for what phase we're in
    digitalWrite(DP, LOW);
  } else {
    digitalWrite(DP, HIGH);
  }

  //  if (sciCount%64 == 0){
  //    datFile.flush();
  //  }
  datFile.flush();

  // Phase 1 (TBD) AT POWER ON 
  if (digitalRead(PIN_RELAY_TE) == LOW) {
    is_active = true;
  }
  if (is_active == false) { //instead of current time would check flag if pin is high or low
    if (MidIMUFlag & HighAFlag) {
      displayPrint(0x00);
    }
    //Do high frequency sensor readings
    makeSensPckt(sensPckt, &count);
     (datFile, sensPckt, HEDR_LEN + senLen);
    //if (count % 5 == 0) {
    sendPckt(sensPckt, HEDR_LEN + senLen);
    //}
    delay(10); 
  }

  //consider adding some sort of check incase voltage spikes up or down so the phase doesnt change
  // Phase 2
  else if (is_active == true) {
    displayPrint(0x01);
    
    //Do science readings, sweeps and sensor data
    //Make First Sensor Packet
    makeSensPckt(sensPckt, &count);    
    
    //medium sweep packet & write
    makeSweepPckt(swpPckt, &count, TYPE_MED_SWP);
    writePckt(datFile, sensPckt, HEDR_LEN + senLen);
    writePckt(datFile, swpPckt, HEDR_LEN + medSwpLen);
    //if (count % 5 == 0) {
    sendPckt(sensPckt, HEDR_LEN + senLen);
    sendPckt(swpPckt, HEDR_LEN + medSwpLen);
    //}
  }
  sciCount++;
}
// OK I THINK
/*sdInit
   Begins wire connection to SD card
   Creates a new file on the SD card called FILE_CNT.DAT to hold one byte of data for the number of files on the sd card
   If the file already exists, update the fileCount variable with the value in the count file
   Parameters: None
   Returns: A boolean for the success of reading from FILE_CNT.DAT and creating a new File
*/
bool sdInit() {
  File cntFile;
  if (!SD.begin(SD_CS)) {
    return false;
  }
  if (SD.exists("FILE_CNT.DAT")) {
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
// OK I THINK
/* sdOpen
    Creates a new .dat file with unique name "UDIP####.DAT" where the # are replaced by the fileCount
    Opens the file to be written and assigns it to global variable datFile
*/
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

/*makeHedr
   Populates the header of the packet with the sync words, the count, and initial time
   Updates the total count variable
   Parameters: A pointer to an array of bytes aka the packet and a pointer to the count variable
   Returns: Nothing
*/
void makeHedr(byte *pckt, uint16_t *count) {
  //Serial.println("Making Header");
  tInitial = millis();
  pckt[HEDR_POS_SYNC] = PCKT_SYNC_0;
  pckt[HEDR_POS_SYNC + 1] = PCKT_SYNC_1;

  memcpy(&pckt[HEDR_POS_COUNT], count, 2);
  (*count)++;
  //everytime new header is made count is updated by 1

  memcpy(&pckt[HEDR_POS_T_INITIAL], &tInitial, 4);
  return;
}
/*makeSensPckt
   Populates the packet specific headers: Type and Length
   Calls function to read the sensors
   Parameters: A pointer to an array of bytes, and a pointer to the count varaible
   Returns: Nothing
*/
void makeSensPckt(byte *pckt, uint16_t *count) {
  makeHedr(pckt, count);
  pckt[HEDR_POS_TYPE] = TYPE_SENS;
  memcpy(&pckt[HEDR_POS_PYLD_LEN], &senLen, 2);
  makeSensPyld(pckt);
  tFinal = millis();

  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal , 4);

  return;
}
/*makeSensPyld
   Reads values from the sensors, LSM9DS1, MMA1210, and TMP36 into associated variables
   Parameters: A pointer to the packet array
   Returns: Nothing
*/
void makeSensPyld(byte *pckt) { //sensor payload
  //Read mid range IMU
  if (MidIMUFlag) {
    MidIMU.read();
    sensors_event_t a, m, g, temp;
    MidIMU.getEvent(&a, &m, &g, &temp);
    //Library returns a float. This scales it properly and makes it a int16_t.
    //******REVIEW (CHECK DOCS) *****
    acc[0] = int16_t(a.acceleration.x * 95.43); acc[1] = int16_t(a.acceleration.y * 95.43); acc[2] = int16_t(a.acceleration.z * 95.43);
    gyr[0] = int16_t(g.gyro.x * 936.25); 
    gyr[1] = int16_t(g.gyro.y * 936.25); 
    gyr[2] = int16_t(g.gyro.z * 936.25);
    mag[0] = int16_t(m.magnetic.x * 409.6); mag[1] = int16_t(m.magnetic.y * 409.6); mag[2] = int16_t(m.magnetic.z * 409.6);
    temp_1 = temp.temperature;
  }
  else {
    acc[0] = 0xffff; acc[1] = 0xffff; acc[2] = 0xffff;
    gyr[0] = 0xffff; gyr[1] = 0xffff; gyr[2] = 0xffff;
    mag[0] = 0xffff; mag[1] = 0xffff; mag[2] = 0xffff;
  }

  //Read high range accel
  if (HighAFlag) {
    sensors_event_t high_a_event;
    HighA.getEvent(&high_a_event);
    //Library returns a float. This scales it properly and makes it a int16_t.
    acc_h = int16_t(high_a_event.acceleration.z * 33.4);
  }
  else {
    acc_h = 0xffff;
  }

  //Read analog board temp
  uint16_t tmpInput = analogRead(PIN_TMP);



  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_M], &acc, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_H], &acc_h, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_GYRO], &gyr, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_MAG], &mag, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_A], &tmpInput, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_D], &temp_1, 2); //instead of temp_1 used to have tr1 (4/15)
  //pckt[HEDR_LEN + SENS_POS_ACCEL_SCALE] = 16;
  return;
}
/*makeSweepPckt
   Makes the header for this packet. Determines which type of sweep is being made, and fills in the final Time
   Parameters: A pointer to the packet array, a pointer to the count variable, and a byte for the sweepType
   Returns: Nothing
*/
void makeSweepPckt(byte *pckt, uint16_t *count, byte sweepType) {
  makeHedr(pckt, count);
  pckt[HEDR_POS_TYPE] = sweepType;
  //uint16_t voltageStep[N_SWP_ADC];
  //uint16_t pd_arr[2] = {0, 0};
  addVoltageRef(pckt);

  if (sweepType == TYPE_MED_SWP) {
    //Medium Sweep
    memcpy(&pckt[HEDR_POS_PYLD_LEN], &medSwpLen, 2);
    makeMedSweep(pckt);
  }
  tFinal = millis();
  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);
}
/*makeMedSweep
   Medium sweep specific function. Calls doStep with appropriate parameters
   Parameters: A pointer to the packet array
   Returns: Nothing
*/
void makeMedSweep(byte *pckt) { //, uint16_t *voltageStep){
  //Serial.println("Making med sweep packets");
  //Serial.flush();
  for (int i = 0; i < N_MED_SWP_STEP; i++) {
    doStep(pckt, medSwpDAC[i], i * SWP_ADC_LEN);
  }
  return;
}

/*addVoltageRef
   Reads voltage from V+ and V- adc pins and places them in the packet
   Parameters: A pointer to the packet array
   Returns: Nothing
*/
void addVoltageRef(byte *pckt) {
  uint16_t vPos = getADC(ADC_VPOS);
  uint16_t vNeg = getADC(ADC_VNEG);

  memcpy(&pckt[HEDR_LEN + SWP_POS_VPOS], &vPos, 2);
  memcpy(&pckt[HEDR_LEN + SWP_POS_VNEG], &vNeg, 2);
  return;
}

//looked over with nate and this gets divded in the ground software (4/15)
uint16_t getADC(int ADCpin) {
  uint16_t val = 0;
  uint32_t val_sum = 0;
  uint16_t val_max = 0;
  uint16_t val_min = 0xffff;
  //why is this loop 18 times (4/15)
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
/*doStep
   Performs one step of the sweep. Writes an anolog voltage out, and reads in ADC pins
   Parameters: A pointer to packet arrays, the value of the DAC output, the location in the packet to memcpy
   Returns: Nothing
*/
void doStep(byte *pckt, uint16_t dacLevel, int loc) { //uint16_t *stepBuf, int loc){
  uint16_t adc1 = 0;
  uint16_t adc2 = 0;
  uint16_t adc3 = 0;
  uint16_t adcDac = 0;

  analogWrite(SWEEP_PIN, dacLevel);
  adcDac = getADC(ADC_SWP);
  adc1 = getADC(ADC_G1);
  adc2 = getADC(ADC_G2);
  adc3 = getADC(ADC_G3);

  memcpy( &pckt[ HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_SWP_V ], &adcDac , 2);
  memcpy( &pckt[ HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_ADC1], &adc1   , 2);
  memcpy( &pckt[ HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_ADC2], &adc2   , 2);
  memcpy( &pckt[ HEDR_LEN + SWP_REF_LEN + loc + SWP_POS_ADC3], &adc3   , 2);

  return;
}
/*writePckt
   Writes the bytes in the packet array to the sd card
   Parameters: A file to write the bytes to, a pointer to the packet array, and the length of the bytes to write
   Returns: Nothing
*/
void writePckt(File f , byte *pckt, uint16_t pcktLen) {
  f.write(pckt, pcktLen);
  return;
}

// call Serial.begin(115200) (or whatever) in setup()
void sendPckt(byte *pckt, uint16_t pcktLen) {
  // Send all bytes out the UART
  Serial1.write(pckt, pcktLen);
  // (optional) wait until it’s all out
  Serial1.flush();
}

void displayPrint(char n) {
  for (char i = 0; i < 7; i++) {
    digitalWrite(seg[i], Chars[n][i]);
  }
}
