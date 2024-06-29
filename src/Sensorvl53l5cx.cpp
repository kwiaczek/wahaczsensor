#include "Sensorvl53l5cx.hpp"
#include "pico/stdlib.h"
#include "vl53l5cx_api.h"
#include <stdio.h>

Sensorvl53l5cx::Sensorvl53l5cx(std::shared_ptr<i2c_inst_t> i2c_instance,
                               uint8_t lpn_pin)
    : i2c_instance(i2c_instance), lpn_pin(lpn_pin), latestDistance(0)
{
    gpio_init(lpn_pin);
    gpio_set_dir(lpn_pin, GPIO_OUT);
    gpio_put(lpn_pin, 0);

    config.platform.address =
        (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    config.platform.i2c = i2c_instance;
}

Sensorvl53l5cx::~Sensorvl53l5cx() { vl53l5cx_stop_ranging(&config); }

bool Sensorvl53l5cx::initialize()
{
    gpio_put(lpn_pin, 1);

    uint8_t status, isAlive;
    status = vl53l5cx_is_alive(&config, &isAlive);
    status |= vl53l5cx_init(&config);
    status |= vl53l5cx_start_ranging(&config);

    bool success = status == VL53L5CX_STATUS_OK && isAlive;

    if (!success)
    {
        printf("initialize has failed at sensor %d\n", lpn_pin);
    }
    return success;
}

bool Sensorvl53l5cx::setAddress(uint8_t newAddress)
{
    uint8_t status;

    gpio_put(lpn_pin, 1);
    status = vl53l5cx_set_i2c_address(&config, newAddress);
    config.platform.address = (uint8_t)((newAddress >> 1) & 0xFF);

    bool success = status == VL53L5CX_STATUS_OK;

    if (!success)
    {
        printf("setAddress has failed on %d sensor!\n", lpn_pin);
    }
    return success;
}

void Sensorvl53l5cx::pollForData()
{
    VL53L5CX_ResultsData results;
    uint8_t isReady;
    vl53l5cx_check_data_ready(&config, &isReady);

    if (isReady)
    {
        vl53l5cx_get_ranging_data(&config, &results);

        // TODO: better algorithm for getting meassuremnt
        latestDistance = results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * 7];
    }
}

int16_t Sensorvl53l5cx::getLatestDistance() { return latestDistance; }