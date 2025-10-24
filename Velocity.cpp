#include "Velocity.h"
#include <Arduino.h>

Velocity::Velocity() {
  velocityY = 0.0;
  yaw = 0.0;
  lastTime = 0;
  accY = 0.0;
  accY_offset = 0.0;  // calibration offset
  gyroZ_offset = 0.0;
}

bool Velocity::begin() {
  Wire.begin();
  mpu.initialize();
  lastTime = millis();
  if (!mpu.testConnection()) return false;

  // ---- Calibration ----
  long accY_sum = 0;
  long gyroZ_sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accY_sum += ay;
    gyroZ_sum += gz;
    delay(5);
  }

  accY_offset = (accY_sum / (float)samples) / 16384.0 * 9.81; // m/s²
  gyroZ_offset = (gyroZ_sum / (float)samples) / 131.0;        // deg/s

  Serial.println("Calibration done");
  Serial.print("accY_offset = "); Serial.println(accY_offset);
  Serial.print("gyroZ_offset = "); Serial.println(gyroZ_offset);

  return true;

}

void Velocity::update() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  // Convert raw gyro to deg/sec (250 dps range -> 131 LSB/dps)
  float gyroZ = gz / 131.0;

  // Time step
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Integrate gyro for yaw
  yaw = gyroZ * dt;  // accumulate yaw

  // Convert raw accel to g
  float accY_g = ay / 16384.0;

  // Convert to m/s² and subtract calibration offset
  float accY_ms2 = accY_g * 9.81 - accY_offset;

  accY = accY_ms2;

  // Integrate acceleration to velocity (accumulate)
  velocityY = accY_ms2 * dt;
}

float Velocity::getVelocity() {
  return velocityY;
}

float Velocity::getYaw() {
  return yaw;
}

float Velocity::getAccelY() {
  return accY;
}
