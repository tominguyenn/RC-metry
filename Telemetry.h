#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

class Telemetry {
  public:
    Telemetry();
    void begin(const char* deviceName = "ESP32 Telemetry");
    void update(float velocity, float yaw, float steeringAngle);

  private:
    void initBLE(const char* deviceName);
};

#endif
