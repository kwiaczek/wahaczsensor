#pragma once
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include <cstdint>

struct SlaveI2cData
{
    uint8_t address;
    uint8_t memory[0xFF + 1];
    bool address_set;
};

class RPController
{
    static uint8_t sampleTime;

  public:
    static void i2cHandler(i2c_inst_t *i2c, i2c_slave_event_t event);
    static void setSamplePollTime(uint8_t newSampleTime);
    static uint8_t currentSamplePollTime();
    static void parseMeassuements(int16_t leftSensorData, int16_t rightSensorData);
};