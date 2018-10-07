/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The BMA280 is an inexpensive (~$1), three-axis, high-resolution (14-bit) acclerometer in a tiny 2 mm x 2 mm LGA12 package with 32-slot FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (15 - 2000 Hz), full range (2 - 16 g), low power modes, 
 *  and interrupt detection behaviors. This accelerometer is nice choice for low-frequency sound and vibration analysis,
 *  tap detection and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "BMA280.h"
#include "Globals.h"

BMA280::BMA280(uint8_t intPin1, uint8_t intPin2)
{
  pinMode(intPin1, INPUT);
  _intPin1 = intPin1;
  pinMode(intPin2, INPUT);
  _intPin2 = intPin2;
}

void BMA280::Calibrate()
{
  float readings_x[4096];
  float readings_y[4096];
  float readings_z[4096];
  // memset(readings_x, 0, sizeof(readings_x));
  // memset(readings_y, 0, sizeof(readings_y));
  // memset(readings_z, 0, sizeof(readings_z));

  int16_t bma_accel_now[3];

  for (int i = 0; i < (sizeof(readings_x) / sizeof(readings_x[0])); i++)
  {
    readBMA280AccelData(bma_accel_now);
    readings_x[i] = (float)bma_accel_now[0] * _aRes / 4.0f;
    readings_y[i] = (float)bma_accel_now[1] * _aRes / 4.0f;
    readings_z[i] = (float)bma_accel_now[2] * _aRes / 4.0f;
    // delay(5);
  }
  _center_x = GetAverage(readings_x);
  _center_y = GetAverage(readings_y);
  _center_z = GetAverage(readings_z);
}

double BMA280::GetAverage(float readings[])
{

  double avg = 0;

  for (int i = 0; i < (sizeof(readings) / sizeof(readings[0])); i++)
  {
    avg = avg + readings[i];
  }
  return avg / (sizeof(readings) / sizeof(readings[0]));
}

void BMA280::MotionDetection()
{
  int16_t bma_accel_now[3];
  readBMA280AccelData(bma_accel_now);
  //Serial.println(_center_x);

  if ((((float)aaccelCount[0] * aRes / 4.0f) > (_center_x + threshold_x)) || ((float)aaccelCount[0] * aRes / 4.0f) < (_center_x - threshold_x))
  {
    bma_motion = true;
    triggered_x = true;
    if (calibrated){
      Serial.println("-- X --");
      delay(100);
    }
  }
  else if ((((float)aaccelCount[1] * aRes / 4.0f) > (_center_y + threshold_y)) || ((float)aaccelCount[1] * aRes / 4.0f) < (_center_y - threshold_y))
  {
    bma_motion = true;
    triggered_y = true;
    if (calibrated){
      Serial.println("-- Y --");
      delay(100);
    }
  }
  else if ((((float)aaccelCount[2] * aRes / 4.0f) > (_center_z + threshold_z)) || ((float)aaccelCount[2] * aRes / 4.0f) < (_center_z - threshold_z))
  {
    bma_motion = true;
    triggered_z = true;
    if (calibrated){
      Serial.println("-- Z --");
      delay(100);
    }
  }
  else
    bma_motion = false;
    triggered_x = false;
    triggered_y = false;
    triggered_z = false;
}

// void BMA280::MotionDetection(float threshold_x, float threshold_y, float threshold_z, int interval) //FIXME
// {
//   float bma_ax_now;
//   float bma_ay_now;
//   float bma_az_now;
//   int16_t bma_accel_now[3];
//   float now = millis();
  

//   //bma_ax = (float)aaccelCount[0] * aRes / 4.0f; // get actual g value, this depends on scale being set
//   //bma_ay = (float)aaccelCount[1] * aRes / 4.0f;
//   //bma_az = (float)aaccelCount[2] * aRes / 4.0f;

//   if (now - previousMillis >= interval)
//   {
//     previousMillis = now;
//     readBMA280AccelData(bma_accel_now);
//     bma_ax_now = (float)bma_accel_now[0] * _aRes / 4.0f; // get actual g value, this depends on scale being set
//     bma_ay_now = (float)bma_accel_now[1] * _aRes / 4.0f; // get actual g value, this depends on scale being set
//     bma_az_now = (float)bma_accel_now[2] * _aRes / 4.0f; // get actual g value, this depends on scale being set

//     // Serial.println("");
//     // Serial.println("");
//     // Serial.println("");
//     // Serial.print("\t");
//     // Serial.print(bma_ax);
//     // Serial.print(" ");
//     // Serial.println(bma_ax_then);
//     // Serial.println("");
//     if((bma_ax_now > (bma_ax_then + threshold_x)) || (bma_ax_now < (bma_ax_then - threshold_x)) || (bma_ay_now > (bma_ay_then + threshold_y)) || (bma_ay_now < (bma_ay_then - threshold_y)) || (bma_az_now > (bma_az_then + threshold_z)) || (bma_az_now < (bma_az_then - threshold_z)))
//     {
//       bma_ax_then = bma_ax_now; 
//       bma_ay_then = bma_ay_now;
//       bma_az_then = bma_az_now;
//       bma_motion = true;
//     }
//     else
//     bma_motion = false;
//   }
// }
uint8_t BMA280::getChipID()
{
  uint8_t c = readByte(BMA280_ADDRESS, BMA280_BGW_CHIPID);
  return c;
}

uint8_t BMA280::getTapType()
{
  uint8_t c = readByte(BMA280_ADDRESS, BMA280_INT_STATUS_0);
  return c;
}

uint8_t BMA280::getTapStatus()
{
  uint8_t c = readByte(BMA280_ADDRESS, BMA280_INT_STATUS_2);
  return c;
}

float BMA280::getAres(uint8_t Ascale)
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs , 4 Gs , 8 Gs , and 16 Gs .
  case AFS_2G:
    _aRes = 2.0f / 8192.0f; // per data sheet
    return _aRes;
    break;
  case AFS_4G:
    _aRes = 4.0f / 8192.0f;
    return _aRes;
    break;
  case AFS_8G:
    _aRes = 8.0f / 8192.0f;
    return _aRes;
    break;
  case AFS_16G:
    _aRes = 16.0f / 8192.0f;
    return _aRes;
    break;
  }
}

void BMA280::initBMA280(uint8_t Ascale, uint8_t BW, uint8_t power_Mode, uint8_t sleep_dur)
{
  writeByte(BMA280_ADDRESS, BMA280_PMU_RANGE, Ascale);                         // set full-scale range
  writeByte(BMA280_ADDRESS, BMA280_PMU_BW, BW);                                // set bandwidth (and thereby sample rate)
  writeByte(BMA280_ADDRESS, BMA280_PMU_LPW, power_Mode << 5 | sleep_dur << 1); // set power mode and sleep duration

  writeByte(BMA280_ADDRESS, BMA280_INT_EN_0, 0x47);             // activate slope and orientation interrupt
  writeByte(BMA280_ADDRESS, BMA280_INT_EN_1, 0x0F);             // activate high-g and low-g interrupt 
  writeByte(BMA280_ADDRESS, BMA280_INT_EN_2, 0x07);             // activate no motion interrupt slow motion: 0x07
  writeByte(BMA280_ADDRESS, BMA280_INT_MAP_0, 0x4F);            //INT1 mapping, 0x08 = slow/nomo int, 0x20 = tap int
  writeByte(BMA280_ADDRESS, BMA280_INT_OUT_CTRL, 0x05);         // INT1 push-pull, active LOW (bits 0:3)
  writeByte(BMA280_ADDRESS, BMA280_INT_SRC, 0x3F);              // filtered/unfiltered  data used for int triggering
  writeByte(BMA280_ADDRESS, BMA280_INT_RST_LATCH, 0x04);        // interrupt mode; 0x06 = 8s latching

  //LSB MAPPING
    // p7 p6 p5 p4 p3 p2 p1 p0
    
}

void BMA280::setinterrupt()
{

  //contains the definition of the number of samples to be used for the slo/nomo int trigger delay
  //writeByte(BMA280_ADDRESS, BMA280_INT_5, 0x00); // 0x00 = no delay

  //low-g threshold; 1 = 7.81mg, 0g to 1.992g
  writeByte(BMA280_ADDRESS, BMA280_INT_1, 0x08);
  //high-g threshold; 1 = 7.81mg, 0g to 1.992g
  writeByte(BMA280_ADDRESS, BMA280_INT_4, 0xA0);
  //set slo/no-motion duration
  writeByte(BMA280_ADDRESS, BMA280_INT_5, 0x14); 
  //slope threshold
  writeByte(BMA280_ADDRESS, BMA280_INT_6, 0x06);
  //no-motion threshold
  writeByte(BMA280_ADDRESS, BMA280_INT_7, 0x08);

}

void BMA280::fastCompensationBMA280()
{
  Serial.println("hold flat and motionless for bias calibration");
  delay(2000);

  uint8_t rawData[2];    // x/y/z accel register data stored here
  float FCres = 7.8125f; // fast compensation offset mg/LSB

  writeByte(BMA280_ADDRESS, BMA280_OFC_SETTING, 0x20 | 0x01); // set target data to 0g, 0g, and +1 g, cutoff at 1% of bandwidth
  writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x20);           // x-axis calibration
  while (!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL)))
  {
  };                                                // wait for calibration completion
  writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x40); // y-axis calibration
  while (!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL)))
  {
  };                                                // wait for calibration completion
  writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x60); // z-axis calibration
  while (!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL)))
  {
  }; // wait for calibration completion

  readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_X, 2, &rawData[0]);
  int16_t offsetX = ((int16_t)rawData[1] << 8) | 0x00;
  Serial.print("x-axis offset = ");
  Serial.print((float)(offsetX)*FCres / 256.0f, 1);
  Serial.println("mg");
  readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_Y, 2, &rawData[0]);
  int16_t offsetY = ((int16_t)rawData[1] << 8) | 0x00;
  Serial.print("y-axis offset = ");
  Serial.print((float)(offsetY)*FCres / 256.0f, 1);
  Serial.println("mg");
  readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_Z, 2, &rawData[0]);
  int16_t offsetZ = ((int16_t)rawData[1] << 8) | 0x00;
  Serial.print("z-axis offset = ");
  Serial.print((float)(offsetZ)*FCres / 256.0f, 1);
  Serial.println("mg");
}

void BMA280::resetBMA280()
{
  writeByte(BMA280_ADDRESS, BMA280_BGW_SOFTRESET, 0xB6); // software reset the BMA280
}

void BMA280::selfTestBMA280()
{
  uint8_t rawData[2]; // x/y/z accel register data stored here

  writeByte(BMA280_ADDRESS, BMA280_PMU_RANGE, AFS_4G); // set full-scale range to 4G
  float STres = 4000.0f / 8192.0f;                      // mg/LSB for 4 g full scale

  // x-axis test
  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x04 | 0x01); // positive x-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_X_LSB, 2, &rawData[0]);
  int16_t posX = ((int16_t)rawData[1] << 8) | rawData[0];

  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x00 | 0x01); // negative x-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_X_LSB, 2, &rawData[0]);
  int16_t negX = ((int16_t)rawData[1] << 8) | rawData[0];

  Serial.print("x-axis self test = ");
  Serial.print((float)(posX - negX) * STres / 4.0f, 1);
  Serial.println("mg, should be > 800 mg");

  // y-axis test
  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x04 | 0x02); // positive y-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_Y_LSB, 2, &rawData[0]);
  int16_t posY = ((int16_t)rawData[1] << 8) | rawData[0];

  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x00 | 0x02); // negative y-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_Y_LSB, 2, &rawData[0]);
  int16_t negY = ((int16_t)rawData[1] << 8) | rawData[0];

  Serial.print("y-axis self test = ");
  Serial.print((float)(posY - negY) * STres / 4.0f, 1);
  Serial.println("mg, should be > 800 mg");

  // z-axis test
  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x04 | 0x03); // positive z-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_Z_LSB, 2, &rawData[0]);
  int16_t posZ = ((int16_t)rawData[1] << 8) | rawData[0];
  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x10 | 0x00 | 0x03); // negative z-axis
  delay(100);
  readBytes(BMA280_ADDRESS, BMA280_ACCD_Z_LSB, 2, &rawData[0]);
  int16_t negZ = ((int16_t)rawData[1] << 8) | rawData[0];

  Serial.print("z-axis self test = ");
  Serial.print((float)(posZ - negZ) * STres / 4.0f, 1);
  Serial.println("mg, should be > 400 mg");

  writeByte(BMA280_ADDRESS, BMA280_PMU_SELF_TEST, 0x00); // disable self test
  /* end of self test*/
}

void BMA280::readBMA280AccelData(int16_t *destination)
{
  uint8_t rawData[6];                                           // x/y/z accel register data stored here
  readBytes(BMA280_ADDRESS, BMA280_ACCD_X_LSB, 6, &rawData[0]); // Read the 6 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];     // Turn the MSB and LSB into a signed 14-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

int16_t BMA280::readBMA280GyroTempData()
{
  uint8_t temp = readByte(BMA280_ADDRESS, BMA280_ACCD_TEMP); // Read the raw data register
  return (((int16_t)temp << 8) | 0x00) >> 8;                 // Turn into signed 8-bit temperature value
}

// I2C scan function
// void BMA280::I2Cscan()
// {
//   // scan for i2c devices
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++)
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmission to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error == 4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
// }

// I2C read/write functions for the BMA280

bool BMA280::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
  return true;
}

byte BMA280::readByte(uint8_t address, uint8_t subAddress)
{
  byte value = 0;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  return value;
}

void BMA280::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
  uint8_t temp[count];
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)count);
  for (int i = 0; i < count; i++)
  {
    dest[i] = Wire.read();
  }
  //return temp[];
  // Wire.transfer(address, &subAddress, 1, dest, count);
}
