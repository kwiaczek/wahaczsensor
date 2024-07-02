#include "RPController.hpp"
#include <stdio.h>

SlaveI2cData slaveI2cData;
static uint8_t sampleTime;

void RPController::i2cHandler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event)
    {
    case I2C_SLAVE_RECEIVE:
        if (!slaveI2cData.address_set)
        {
            slaveI2cData.address = i2c_read_byte_raw(i2c);
            slaveI2cData.address_set = true;
        }
        else
        {
            slaveI2cData.memory[slaveI2cData.address] = i2c_read_byte_raw(i2c);
            RPController::setSamplePollTime(slaveI2cData.memory[slaveI2cData.address]);
        }
        break;
    case I2C_SLAVE_REQUEST:
        i2c_write_byte_raw(i2c, slaveI2cData.memory[slaveI2cData.address++]);
        break;
    case I2C_SLAVE_FINISH:
        slaveI2cData.address_set = false;
        slaveI2cData.address = 0;
        break;
    default:
        printf("Unrecognised request: %d\n", event);
        break;
    }
}

uint8_t RPController::currentSamplePollTime()
{
    return sampleTime;
}

void RPController::parseMeassuements(int16_t leftSensorData, int16_t rightSensorData)
{
    slaveI2cData.memory[0] = leftSensorData >> 8;
    slaveI2cData.memory[1] = leftSensorData & 0xFF;
    slaveI2cData.memory[2] = rightSensorData >> 8;
    slaveI2cData.memory[3] = rightSensorData & 0xFF;
    //printf("Acquired sample %d %d\n", leftSensorData, rightSensorData);
}

void RPController::setSamplePollTime(uint8_t newSampleTime)
{
    sampleTime = newSampleTime;
}