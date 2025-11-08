#include "Velocity.h"
#include <Arduino.h>

// Constructor - initializes all member variables to zero/defaults
Velocity::Velocity() {
  velocityX = 0.0;        // Forward/backward velocity in m/s
  velocityY = 0.0;        // Sideways velocity in m/s (for drift detection)
  yaw = 0.0;              // Accumulated yaw angle in degrees
  lastTime = 0;           // Timestamp of last update in milliseconds
  accX = 0.0;             // Current X acceleration in m/s²
  accY = 0.0;             // Current Y acceleration in m/s²
  accX_offset = 0.0;      // Calibration offset for X acceleration
  accY_offset = 0.0;      // Calibration offset for Y acceleration
  gyroZ_offset = 0.0;     // Calibration offset for Z gyroscope (yaw rate)
}

// Initialize the MPU6050 sensor and perform calibration
bool Velocity::begin() {
  Wire.begin();           // Initialize I2C communication bus
  mpu.initialize();       // Initialize the MPU6050 sensor with default settings
  lastTime = millis();    // Record the current time as starting point
  
  // Test if the sensor is connected and responding
  if (!mpu.testConnection()) {
    return false;         // Return false if sensor not detected
  }

  // ===== CALIBRATION PHASE =====
  // The sensor has inherent bias/drift even when stationary
  // We measure this bias and subtract it from all future readings
  
  Serial.println("Starting calibration - keep device STILL!");
  delay(1000);            // Give user time to stop moving the device
  
  long accX_sum = 0;      // Accumulator for X acceleration readings
  long accY_sum = 0;      // Accumulator for Y acceleration readings
  long gyroZ_sum = 0;     // Accumulator for Z gyroscope readings
  const int samples = 1000; // Take 1000 samples for accurate calibration (increased from 500)

  // Collect samples while device is stationary
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz; // Raw 16-bit signed values from sensor
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read all 6 axes at once
    
    accX_sum += ax;       // Accumulate X acceleration
    accY_sum += ay;       // Accumulate Y acceleration
    gyroZ_sum += gz;      // Accumulate Z rotation rate
    
    delay(3);             // Small delay between samples (reduced for faster calibration)
  }

  // Calculate average offsets and convert to real units
  // MPU6050 accelerometer has ±2g range with 16384 LSB/g sensitivity
  // Convert: (raw_sum / samples) / 16384.0 = average in g's
  // Then multiply by 9.81 to get m/s²
  accX_offset = (accX_sum / (float)samples) / 16384.0 * 9.81;
  accY_offset = (accY_sum / (float)samples) / 16384.0 * 9.81;
  
  // MPU6050 gyroscope has ±250°/s range with 131 LSB/(°/s) sensitivity
  // Convert: (raw_sum / samples) / 131.0 = average in °/s
  gyroZ_offset = (gyroZ_sum / (float)samples) / 131.0;

  // Display calibration results for verification
  Serial.println("Calibration complete!");
  Serial.print("accX_offset = "); Serial.print(accX_offset); Serial.println(" m/s²");
  Serial.print("accY_offset = "); Serial.print(accY_offset); Serial.println(" m/s²");
  Serial.print("gyroZ_offset = "); Serial.print(gyroZ_offset); Serial.println(" °/s");

  return true;            // Initialization successful
}

// Update function - call this every loop iteration to get new sensor data
void Velocity::update() {
  // ===== READ SENSOR DATA =====
  int16_t ax, ay, az;     // Accelerometer raw values (X, Y, Z axes)
  int16_t gx, gy, gz;     // Gyroscope raw values (X, Y, Z axes)
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read all 6 DOF (degrees of freedom)

  // ===== CALCULATE TIME STEP =====
  // Integration requires knowing how much time passed since last measurement
  unsigned long now = millis();           // Current time in milliseconds
  float dt = (now - lastTime) / 1000.0;   // Time step in seconds
  lastTime = now;                         // Update last time for next iteration
  
  // Safety check: if time step is too large (device was paused), skip this update
  //if (dt > 1.0 || dt <= 0.0) {
  //  return;                               // Ignore this reading to prevent velocity spikes
  //}

  // ===== PROCESS GYROSCOPE (YAW RATE) =====
  // Gyroscope measures rotation rate in degrees per second
  float gyroZ_raw = gz / 131.0;           // Convert raw value to °/s (131 LSB per °/s)
  float gyroZ = gyroZ_raw - gyroZ_offset; // Subtract calibration offset to remove bias
  
  // INTEGRATION: yaw angle = previous yaw + (rotation rate × time)
  // This accumulates the total rotation over time
  yaw += gyroZ * dt;                      // Add rotation change to accumulated yaw
  
  // Optional: keep yaw in 0-360° range (uncomment if desired)
  // while (yaw >= 360.0) yaw -= 360.0;
  // while (yaw < 0.0) yaw += 360.0;

  // ===== PROCESS ACCELEROMETER (X AXIS - FORWARD/BACKWARD) =====
  float accX_g = ax / 16384.0;            // Convert raw to g's (16384 LSB per g)
  float accX_ms2 = accX_g * 9.81 - accX_offset; // Convert to m/s² and remove bias
  accX = accX_ms2;                        // Store for external access
  
  // INTEGRATION: velocity = previous velocity + (acceleration × time)
  // This accumulates velocity changes over time
  velocityX = accX_ms2 * dt;             // Update forward/backward velocity
  
  // Apply decay factor to counteract drift (sensor integration error accumulates)
  // Without decay, small errors cause velocity to drift even when stopped
  velocityX *= 0.98;                      // 2% decay per update (tune this value)

  // ===== PROCESS ACCELEROMETER (Y AXIS - SIDEWAYS) =====
  float accY_g = ay / 16384.0;            // Convert raw to g's
  float accY_ms2 = accY_g * 9.81 - accY_offset; // Convert to m/s² and remove bias
  accY = accY_ms2;                        // Store for external access
  
  // INTEGRATION: same as X axis
  velocityY = accY_ms2 * dt;             // Update sideways velocity (important for drift!)
  
  // Apply decay factor
  velocityY *= 0.98;                      // 2% decay per update
  
  // Optional: Zero out very small velocities to reduce noise when stopped
  if (abs(velocityX) < 0.01) velocityX = 0.0; // Threshold: 0.01 m/s = 1 cm/s
  if (abs(velocityY) < 0.01) velocityY = 0.0;
}

// Get the current forward/backward velocity in m/s
float Velocity::getVelocityX() {
  return velocityX;
}

// Get the current sideways velocity in m/s (useful for detecting drift)
float Velocity::getVelocityY() {
  return velocityY;
}

// Get the total (combined) velocity magnitude in m/s
// Uses Pythagorean theorem: speed = √(vx² + vy²)
float Velocity::getVelocity() {
  return sqrt(velocityX * velocityX + velocityY * velocityY);
}

// Get the accumulated yaw angle in degrees
float Velocity::getYaw() {
  return yaw;
}

// Get the current X acceleration in m/s² (forward/backward)
float Velocity::getAccelX() {
  return accX;
}

// Get the current Y acceleration in m/s² (sideways)
float Velocity::getAccelY() {
  return accY;
}

// Reset all velocity and yaw values to zero
// Useful when you want to restart measurements
void Velocity::reset() {
  velocityX = 0.0;
  velocityY = 0.0;
  yaw = 0.0;
  lastTime = millis(); // Reset time reference
}
