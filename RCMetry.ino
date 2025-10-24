#include <Arduino.h>
#include "Steering.h"
#include "Velocity.h"
#include "Telemetry.h"

Steering steering;
Velocity velocity;
Telemetry telemetry;

void setup() {
  Serial.begin(115200);

  if (!steering.begin()) {
    Serial.println("AS5600 not detected!");
    while (1);
  }

  if (!velocity.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }

  telemetry.begin("ESP32 Telemetry");
}

void loop() {
  velocity.update();

  float vel = velocity.getVelocity();
  float yaw = velocity.getYaw();
  float angle = steering.getAngle();
  float acc_y = velocity.getAccelY();

  telemetry.update(vel, yaw, angle);

  Serial.print("Angle: "); Serial.print(angle);
  Serial.print("° | Velocity: "); Serial.print(vel);
  Serial.print(" m/s | Yaw: "); Serial.println(yaw);
  Serial.print("acceleration for Y : "); Serial.println(acc_y);

  delay(100);
}
