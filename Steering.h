#ifndef STEERING_H
#define STEERING_H

#include <AS5600.h>

class Steering {
  public:
    Steering();
    bool begin();
    float getAngle();  // returns calibrated angle

  private:
    AS5600 encoder;
};

#endif
