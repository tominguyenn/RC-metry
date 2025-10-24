#ifndef VELOCITY_H
#define VELOCITY_H

#include <Wire.h>
#include "MPU6050.h"

class Velocity {
  public:
    Velocity();
    bool begin();
    void update();        // call in loop()
    float getVelocity();  // m/s
    float getYaw();       // degrees
    float getAccelY();

  private:
    MPU6050 mpu;
    float velocityY;
    float yaw;
    float accY;
    unsigned long lastTime;
    float accY_offset;
    float gyroZ_offset;

};

#endif
