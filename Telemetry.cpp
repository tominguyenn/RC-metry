#include "Telemetry.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs for BLE Service + Characteristics
#define SERVICE_UUID           "12345678-1234-1234-1234-1234567890ab"
#define VELOCITY_CHAR_UUID     "12345678-1234-1234-1234-1234567890ac"
#define YAW_CHAR_UUID          "12345678-1234-1234-1234-1234567890ad"
#define STEERING_CHAR_UUID     "12345678-1234-1234-1234-1234567890ae"

// BLE characteristic pointers
static BLECharacteristic *velocityChar;
static BLECharacteristic *yawChar;
static BLECharacteristic *steeringChar;

Telemetry::Telemetry() {}

void Telemetry::begin(const char* deviceName) {
  initBLE(deviceName);
}

void Telemetry::initBLE(const char* deviceName) {
  BLEDevice::init(deviceName);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Velocity
  velocityChar = pService->createCharacteristic(
    VELOCITY_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  velocityChar->addDescriptor(new BLE2902());

  // Yaw
  yawChar = pService->createCharacteristic(
    YAW_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  yawChar->addDescriptor(new BLE2902());

  // Steering
  steeringChar = pService->createCharacteristic(
    STEERING_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  steeringChar->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE Telemetry ready!");
}

void Telemetry::update(float velocity, float yaw, float steeringAngle) {
  char buffer[16];
 if (steeringChar) {
    snprintf(buffer, sizeof(buffer), "%.2f", steeringAngle);
    steeringChar->setValue(buffer);
    steeringChar->notify();
  }
  if (velocityChar) {
    snprintf(buffer, sizeof(buffer), "%.2f", velocity);
    velocityChar->setValue(buffer);
    velocityChar->notify();
  }
  if (yawChar) {
    snprintf(buffer, sizeof(buffer), "%.2f", yaw);
    yawChar->setValue(buffer);
    yawChar->notify();
  }
 
}
