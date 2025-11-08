#include <Arduino.h>
#include "Steering.h"
#include "Velocity.h"
#include "Telemetry.h"

Steering steering;
Velocity velocity;
Telemetry telemetry;

// Toggle between Serial Monitor (text) and Serial Plotter (graphs)
// Set to 'true' for graphs, 'false' for text readout
bool useSerialPlotter = true;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection
  delay(1000);

  if (!steering.begin()) {
    Serial.println("AS5600 not detected!");
    while (1);
  }

  if (!velocity.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }

  telemetry.begin("ESP32 Telemetry");
  
  // Print startup message
  if (!useSerialPlotter) {
    Serial.println("\n=== RC Drift Car Telemetry ===");
    Serial.println("System Ready!\n");
  }
  
  delay(1000);
}

void loop() {
  // Update velocity sensor every loop
  velocity.update();

  // Get all sensor readings
  float velX = velocity.getVelocityX();      // Forward speed
  float velY = velocity.getVelocityY();      // Sideways speed (drift!)
  float totalVel = velocity.getVelocity();   // Combined speed
  float yaw = velocity.getYaw();             // Yaw angle
  float angle = steering.getAngle();         // Steering angle
  float accX = velocity.getAccelX();         // Forward acceleration
  float accY = velocity.getAccelY();         // Sideways acceleration

  // Send to Bluetooth
  telemetry.update(totalVel, yaw, angle);
  
  // ===== OUTPUT TO SERIAL =====
  if (useSerialPlotter) {
    // SERIAL PLOTTER MODE - Graphs
    // Format: "Label:value " (space between each)
    // Use println ONLY on the last value
    
    Serial.print("Steering:");
    Serial.print(angle);
    Serial.print(" ");
    
    Serial.print("VelX:");
    Serial.print(velX);
    Serial.print(" ");
    
    Serial.print("VelY:");
    Serial.print(velY);
    Serial.print(" ");
    
    Serial.print("VelTotal:");
    Serial.print(totalVel);
    Serial.print(" ");
    
    Serial.print("Yaw:");
    Serial.print(yaw / 10.0);  // Scaled down for better visualization
    Serial.print(" ");
    
    Serial.print("AccX:");
    Serial.print(accX);
    Serial.print(" ");
    
    Serial.print("AccY:");
    Serial.println(accY);  // println on last value
    
  } else {
    // SERIAL MONITOR MODE - Text readout
    Serial.print("Steering Angle: "); Serial.print(angle);
    Serial.print(" | Vel X: "); Serial.print(velX);
    Serial.print(" | Vel Y: "); Serial.print(velY);
    Serial.print(" | Total: "); Serial.print(totalVel);
    Serial.print(" | Yaw: "); Serial.println(yaw);
  }
  
  delay(50);  // 20 Hz update rate
}
