#include <cstring>
#include <platform.h>
#include <stdio.h>
#include <unistd.h>
#include <vl53l5cx_api.h>

#include "RPController.hpp"
#include "Sensorvl53l5cx.hpp"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/i2c_slave.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#define I2C_SDA 0
#define I2C_SCL 1
#define PIN_LPn 2
#define PIN_LPn2 3
#define SI2C_SDA 14
#define SI2C_SCL 15
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

static std::shared_ptr<Sensorvl53l5cx> leftSensor;
static std::shared_ptr<Sensorvl53l5cx> rightSensor;

static void createSampleCollectInterrupt();

static void handleSampleCollectInterrupt()
{
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    leftSensor->pollForData();
    rightSensor->pollForData();

    RPController::parseMeassuements(leftSensor->getLatestDistance(), rightSensor->getLatestDistance());

    createSampleCollectInterrupt();
}

void createSampleCollectInterrupt()
{
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, handleSampleCollectInterrupt);
    irq_set_enabled(ALARM_IRQ, true);
    uint64_t target = timer_hw->timerawl + 1000000;
    timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

int main(void)
{
    // pico settings
    stdio_init_all();
    std::shared_ptr<i2c_inst_t> vl53l5cx_i2c = std::make_shared<i2c_inst_t>();
    vl53l5cx_i2c->hw = i2c0_hw;
    vl53l5cx_i2c->restart_on_next = false;

    i2c_init(vl53l5cx_i2c.get(), 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    i2c_init(i2c1, 100000);
    gpio_set_function(SI2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SI2C_SCL, GPIO_FUNC_I2C);
    i2c_slave_init(i2c1, 0x21, &RPController::i2cHandler);

    sleep_ms(3000);
    puts("SwingArm Sensors have started....\n");

    leftSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn);
    rightSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn2);


    if (leftSensor->setAddress((uint8_t)((0x32 >> 1) & 0xFF)))
    {
        printf("URRRA!!!!");

    }

    if (!(leftSensor->initialize() && rightSensor->initialize()))
    {
        return -1;
    }

    printf("successfully initialized!");

    createSampleCollectInterrupt();

    while (true)
    {
    }
}
