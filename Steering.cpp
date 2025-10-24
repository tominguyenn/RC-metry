#include "Steering.h"
#include <Wire.h>

Steering::Steering() {}

bool Steering::begin() 
{
  Wire.begin();  
  return encoder.begin();
}

float Steering::getAngle() 
{
  int raw = encoder.rawAngle();          // 0 – 4095
  float angle = (raw * 360.0) / 4096.0;  // scale to 0–360
  angle = angle - 259;                   // your offset
  angle = 2.8 * angle;                   // your scaling
  return angle;
}
