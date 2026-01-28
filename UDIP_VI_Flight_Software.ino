/*
  University of Delaware Ionosphere Probe
  Department of Physics and Astronomy
  Team 2025-2026
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

/*Arduino Due Pins for Voltage Sweep -- CHECK W/ ELECTRICAL*/
#define SWEEP_PIN DAC0  //voltage array is written to this pin
#define ADC_SWP A0  //sweep voltage
#define ADC_A A1  //probe A
#define ADC_B A2  //probe B

/*Define number of ADCs using for a sweep -- CHECK W/ ELECTRICAL*/
#define N_SWP_ADC   4 //num. of ADCs used for a sweep (3 sweep measurements + 1 for DAC)

#define N_SWP_STEP  2 //num. of steps per sweep

const uint16_t SwpLen = N_SWP_STEP * SWP_ADC_LEN + SWP_REF_LEN; //length of sweep
#define MAX_SWP_LEN  (N_SWP_STEP * SWP_ADC_LEN)
//Define SwpDAC[] -- CHECK W/ ELECTRICAL
const uint16_t swpDAC[N_SWP_STEP] = {};

//analog board temp sensor??

/*SD Card Pin Define -- CHECK W/ ELECTRICAL*/
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
int16_t temp_1;

void setup() {
  //Initialize I2C communication
  Serial1.begin(115200);
  Serial.begin(9600); //starts serial communication
  while (!Serial) {} //wait for serial monitor
  Serial.println("Initializing...");
  Wire.begin();
  Wire.setClock(10000); //set I2C to fast-mode (400kHz)
  //add test code here

  //configure pins for 7-segment display
  pinMode(seg[0], OUTPUT);
  pinMode(seg[1], OUTPUT);
  pinMode(seg[2], OUTPUT);
  pinMode(seg[3], OUTPUT);
  pinMode(seg[4], OUTPUT);
  pinMode(seg[5], OUTPUT);
  pinMode(seg[6], OUTPUT);
  pinMode(seg[7], OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

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
    //check if this calibration is accurate
    acc[0] = int16_t(ac.acceleration.x * 95.43);
    acc[1] = int16_t(ac.acceleration.y * 95.43);
    acc[2] = int16_t(ac.acceleration.z * 95.43);
    gyr[0] = int16_t(gy.gyro.x * 936.25);
    gyr[1] = int16_t(gy.gyro.y * 936.25);
    gyr[2] = int16_t(gy.gyro.z * 936.25);
    mag[0] = int16_t(mg.magnetic.x * 409.6);
    mag[1] = int16_t(mg.magnetic.y * 409.6);
    mag[2] = int16_t(mg.magnetic.z * 409.6);
    temp_1 = temp.temperature;
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
  //analog board temp?? are we measuring that or just the temp on sensor board

  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_M], &acc, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_H], &acc_h, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_GYRO], &gyr, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_MAG], &mag, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_D], &temp_1, 2);

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
