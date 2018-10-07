#ifndef QFILTER_h
#define QFILTER_h

#include "Wire.h"

class QFilter
{
public:
  QFilter();
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
};

#endif