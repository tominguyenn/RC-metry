#ifndef VELOCITY_H
#define VELOCITY_H

#include <Wire.h>
#include "MPU6050.h"

class Velocity {
  public:
    Velocity();
    bool begin();
    void update();
    
    // NEW: Separate X and Y velocity getters
    float getVelocityX();     // Forward/backward velocity
    float getVelocityY();     // Sideways velocity (for drift!)
    
    float getVelocity();      // CHANGED: Now returns total magnitude
    float getYaw();
    
    // NEW: Separate X acceleration getter
    float getAccelX();
    float getAccelY();
    
    // NEW: Reset function
    void reset();

  private:
    MPU6050 mpu;
    
    // NEW: Added velocityX (your original only had velocityY)
    float velocityX;          // Forward/backward velocity
    float velocityY;          // Sideways velocity
    float yaw;
    
    // NEW: Added accX
    float accX;               // X acceleration
    float accY;               // Y acceleration
    
    unsigned long lastTime;
    
    // NEW: Added accX_offset
    float accX_offset;        // Calibration for X axis
    float accY_offset;
    float gyroZ_offset;
};

#endif
