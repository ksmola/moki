#include "Globals.h"

// These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

int gps_enable = D17; // GPS enable pin, active LOW, PTA12
int myLed = 13;       // USER LED
int imu_int = D18;    // IMU interrupt pin, PTA13
int intPin = D18;

// Set initial input parameters

float ax, ay, az, gx, gy, gz, mx, my, mz;
float q[4]; // vector to hold quaternion
float eInt[3];
float Quat[4];                   // quaternion data register
float GyroMeasError = PI * (40.0f / 180.0f); 
float GyroMeasDrift = PI * (0.0f / 180.0f);
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; 

uint32_t delt_t, count, sumCount; // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;               // Capitalized: Hardware, non-cap: software
float deltat, sum;                                     // integration interval for both filter schemes
uint32_t lastUpdate, firstUpdate;                      // used to calculate integration interval
uint32_t Now;                                          // used to calculate integration interval
uint8_t param[4];                                      // used for param transfer
uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];  // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];

float magCalibration[3];                                  // Factory mag calibration and mag bias
float gyroBias[3], accelBias[3], magBias[3], magScale[3]; // Bias corrections for gyro, accelerometer, mag
int16_t tempCount, rawPressure, rawTemperature;           // pressure, temperature raw count output
float temperature, pressure, altitude;                    // Stores the MPU9250 internal chip temperature in degrees Celsius
float SelfTest[6];                                        // holds results of gyro and accelerometer self test

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp;    // pressure and temperature raw count output for BMP280

float aRes, gRes, mRes; // scale resolutions per LSB for the sensors
uint8_t sentral_motion;
// Specify sensor full scale
uint8_t Gscale;
uint8_t Ascale;
uint8_t Mscale; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode;  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

uint8_t Posr, Tosr, Mode, IIRFilter, SBy; // set pressure amd temperature output data rate
int32_t t_fine;

//BMA 280:
float aaRes;                      // scale resolutions per LSB for the sensor
int16_t aaccelCount[3];           // Stores the 16-bit signed accelerometer sensor output
float bma_ax, bma_ay, bma_az;     // variables to hold latest sensor data values
uint32_t adelt_t = 0, acount = 0; // used to control display output rate
bool bma_motion;
unsigned long previousMillis = 0;
float bma_ax_then = 0;
float bma_ay_then = 0;
float bma_az_then = 0;
bool bma_printout = 0;
float threshold_x = 0.22; //.22
float threshold_y = 0.25;   // .25
float threshold_z = 0.05;  // .03
int interval = 350;
bool calibrated = false;
bool triggered_x = false;
bool triggered_y = false;
bool triggered_z = false;

//moki
bool rgbblink = false;
bool armed = false;