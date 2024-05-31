#include <platform.h>
#include <stdio.h>
#include <unistd.h>
#include <vl53l5cx_api.h>

#include "Sensorvl53l5cx.hpp"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#define I2C_SDA 0
#define I2C_SCL 1
#define PIN_LPn 2
#define PIN_LPn2 3

std::shared_ptr<Sensorvl53l5cx> leftSensor;
std::shared_ptr<Sensorvl53l5cx> rightSensor;

void poll() {
  while (true) {
    leftSensor->pollForData();
    rightSensor->pollForData();
    sleep_ms(1000);
  }
}

int main(void) {
  // pico settings
  stdio_init_all();
  std::shared_ptr<i2c_inst_t> vl53l5cx_i2c = std::make_shared<i2c_inst_t>();
  vl53l5cx_i2c->hw = i2c0_hw;
  vl53l5cx_i2c->restart_on_next = false;

  i2c_init(vl53l5cx_i2c.get(), 400 * 1000); // 400kHz
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

  sleep_ms(3000);
  puts("SwingArm Sensors have started....\n");

  leftSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn);
  rightSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn2);

  leftSensor->setAddress(0x16);

  if (!(leftSensor->initialize() && rightSensor->initialize())) {
    return -1;
  }

  printf("successfully initialized!");

  multicore_launch_core1(poll);

  while (true) {
    printf("%d --- %d\n", leftSensor->getLatestDistance(),
           rightSensor->getLatestDistance());
    sleep_ms(500);
  }
}
