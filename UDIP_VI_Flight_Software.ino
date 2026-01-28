/*
  University of Delaware Ionosphere Probe
  Department of Physics and Astronomy
  Team 2025-2026
*/

//Includes
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_H3LIS331.h>

//Define Display

//Packet Types -- Sensor and Sweep Packets
#define TYPE_SENS 0x01
#define TYPE_SWP 0x10

//Packet Headers -- Number is byte offset
#define HEDR_POS_SYNC       0
#define HEDR_POS_COUNT      2
#define HEDR_POS_T_INITIAL  4
#define HEDR_POS_T_FINAL    8
#define HEDR_POS_TYPE      12
#define HEDR_POS_PYLD_LEN  13
#define HEDR_LEN           15

//Packet Headers -- Sync word
#define PCKT_SYNC_0 0x55 //ASCII 'U'
#define PCKT_SYNC_1 0x44 //ASCII 'D'

//Sensor Payload
#define SENS_POS_ACCEL_M      0 //3 two byte signed ints. Mid range accels
#define SENS_POS_ACCEL_H      6 //1 two byte signed int. High range accels
#define SENS_POS_GYRO         8 //3 two byte signed ints. Gyroscopes
#define SENS_POS_MAG         14 //3 two byte signed ints. Magnetometers
#define SENS_POS_TMP_D       20 //1 two byte unsigned int. Digital Board Temp
#define SENS_POS_PHOTO       22 //1 two byte unsigned int. Photodiodes

//Sweep Reference Payload
//Two voltages/currents simultaneously recorded (probe A and probe B)
#define SWP_POS_V_A    0 //1 two byte unsigned int for reading of probe A voltage line
#define SWP_POS_V_B    2 //1 two byte unsigned int for reading of probe B voltage line
#define SWP_CURR_A     4 //1 two byte unsigned int for reading of probe A current
#define SWP_CURR_B     6 //1 two byte unsigned int for reading of probe B current
#define SWP_REF_LEN    8 //1 two byte unsigned int for reference payload length

//Sweep Packet ADC positions -- 3 ADCs -- CHECK W/ ELECTRICAL
//ADC0 -- voltage applied to probe (+9V to -9V)
//ADC1 -- probe A current
//ADC2 -- probe B current 

//Sweep Packet DAC Value Arrays -- CHECK W/ ELECTRICAL
// #define ZERO_VOLT_DAC   1737
// #define DAC_MIN          837
// #define DAC_MAX         2693

//Arduino Due Pins for Voltage Sweep -- CHECK W/ ELECTRICAL
//Which DAC is used to send voltage/create sweep (define SWEEP_PIN)

//Define number of ADCs using for a sweep (3 + 1 for DAC measurements) -- CHECK W/ ELECTRICAL
#define N_SWP_ADC   4

//Define SwpLen = SWP_STEP * SWP_ADC_LEN + SWP_REF_LEN
// #define MAX_SWP_LEN  (SWP_STEP * SWP_ADC_LEN)
//Define SwpDAC[] -- CHECK W/ ELECTRICAL

//LSM9DS1 -- Sensor Board
Adafruit__LSM9DS1 MidIMU = Adafruit_LSM9DS1();
bool MidIMUFlag = true;int16_t acc[3];
int16_t gyr[3];
int16_t mag[3];
int16_t temp_1;

//H3LIS331 -- High Range Accelerometer
Adafruit_H3LIS331 HighA = Adafruit_H3LIS331();
bool HighAFlag = true;
int16_t acc_h;

//Photodiode(s)

//SD Card Pin Define

//Relay Pin Define

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
