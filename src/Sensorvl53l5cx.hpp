#pragma once
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "vl53l5cx_api.h"
#include <memory>

class Sensorvl53l5cx {
  std::shared_ptr<i2c_inst_t> i2c_instance;
  uint8_t lpn_pin;
  int16_t latestDistance; // TODO: maybe take into account more than 1 reading
  VL53L5CX_Configuration config;

public:
  Sensorvl53l5cx(std::shared_ptr<i2c_inst_t> i2c_instance, uint8_t lpn_pin);
  ~Sensorvl53l5cx();

  bool initialize();
  bool setAddress(uint8_t address);
  void pollForData();
  int16_t getLatestDistance();

private:
  void setLpnPinState(uint8_t state);
};