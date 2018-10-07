#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>

// enum Ascale;
// enum Gscale;
// enum Mscale;
// enum Posr;
// enum Tosr;
// enum IIRFilter;
// enum Mode;
// enum SBy;

enum Ascale
{
    AFS_2G,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale
{
    GFS_250DPS,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum Mscale
{
    MFS_14BITS, // 0.6 mG per LSB
    MFS_16BITS  // 0.15 mG per LSB
};

enum Posr
{
    P_OSR_00, // no op
    P_OSR_01,
    P_OSR_02,
    P_OSR_04,
    P_OSR_08,
    P_OSR_16
};

enum Tosr
{
    T_OSR_00, // no op
    T_OSR_01,
    T_OSR_02,
    T_OSR_04,
    T_OSR_08,
    T_OSR_16
};

enum IIRFilter
{
    full, // bandwidth at full sample rate
    BW0_223ODR,
    BW0_092ODR,
    BW0_042ODR,
    BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode
{
    BMP280Sleep,
    forced,
    forced2,
    normal
};

enum SBy
{
    t_00_5ms,
    t_62_5ms,
    t_125ms,
    t_250ms,
    t_500ms,
    t_1000ms,
    t_2000ms,
    t_4000ms,
};

extern int gps_enable, myLed, imu_int, intPin;

extern float ax, ay, az, gx, gy, gz, mx, my, mz;
extern float q[4]; // vector to hold quaternion
extern float eInt[3];
extern float Quat[4];
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
extern float GyroMeasError; // gyroscope measurement error in rads/s (start at 40 deg/s)
extern float GyroMeasDrift;  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
extern float beta; // compute beta
extern float zeta; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

extern uint32_t delt_t, count, sumCount; // used to control display output rate
extern float pitch, yaw, roll, Yaw, Pitch, Roll;
extern float deltat, sum;                                     // integration interval for both filter schemes
extern uint32_t lastUpdate, firstUpdate;                      // used to calculate integration interval
extern uint32_t Now;                                          // used to calculate integration interval
extern uint8_t param[4];                                      // used for param transfer
extern uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

extern int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
extern int16_t gyroCount[3];  // Stores the 16-bit signed gyro sensor output
extern int16_t magCount[3];

extern float magCalibration[3];                                  // Factory mag calibration and mag bias
extern float gyroBias[3], accelBias[3], magBias[3], magScale[3]; // Bias corrections for gyro, accelerometer, mag
extern int16_t tempCount, rawPressure, rawTemperature;           // pressure, temperature raw count output
extern float temperature, pressure, altitude;                    // Stores the MPU9250 internal chip temperature in degrees Celsius
extern float SelfTest[6];

extern uint16_t dig_T1, dig_P1;
extern int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
extern double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
extern int32_t rawPress, rawTemp;    // pressure and temperature raw count output for BMP280

extern float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

extern uint8_t sentral_motion;
//

// Specify sensor full scale
extern uint8_t Gscale;
extern uint8_t Ascale;
extern uint8_t Mscale; // Choose either 14-bit or 16-bit magnetometer resolution
extern uint8_t Mmode;  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

extern uint8_t Posr, Tosr, Mode, IIRFilter, SBy; // set pressure amd temperature output data rate
extern int32_t t_fine;

//BMA280:
extern float aaRes;                      // scale resolutions per LSB for the sensor
extern int16_t aaccelCount[3];           // Stores the 16-bit signed accelerometer sensor output
extern float bma_ax, bma_ay, bma_az;     // variables to hold latest sensor data values
extern uint32_t adelt_t, acount; // used to control display output rate
extern unsigned long previousMillis;
extern bool bma_motion;
extern float bma_ax_then, bma_ay_then, bma_az_then;
extern bool bma_printout;
extern float threshold_x, threshold_y, threshold_z;
extern int interval;
extern bool triggered_x, triggered_y, triggered_z;
extern bool calibrated;
// extern double center_x, center_y, center_z;

//moki
extern bool rgbblink;
extern bool armed;
#endif