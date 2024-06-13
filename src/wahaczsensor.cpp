#include <platform.h>
#include <stdio.h>
#include <unistd.h>
#include <vl53l5cx_api.h>
#include <cstring>

#include "Sensorvl53l5cx.hpp"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"

#define I2C_SDA 0
#define I2C_SCL 1
#define PIN_LPn 2
#define PIN_LPn2 3
#define SI2C_SDA 14
#define SI2C_SCL 15

std::shared_ptr<Sensorvl53l5cx> leftSensor;
std::shared_ptr<Sensorvl53l5cx> rightSensor;

void poll() {
  while (true) {
    leftSensor->pollForData();
    rightSensor->pollForData();
    sleep_ms(1000);
  }
}

uint8_t tx[4];

static void i2cHandler(i2c_inst_t *i2c, i2c_slave_event_t event) {
  if (event == I2C_SLAVE_REQUEST)
  {
    printf("REQUEST FULLFIELED! CALLBACK FROM HELL!\n %d %d \n", (I2C_SLAVE_REQUEST == event), event);
	  i2c_write_raw_blocking(i2c, tx, 4);
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


	i2c_init(i2c1, 100000);
  gpio_set_function(SI2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(SI2C_SCL, GPIO_FUNC_I2C);
	i2c_slave_init(i2c1, 0x21, &i2cHandler);


  sleep_ms(3000);
  puts("SwingArm Sensors have started....\n");

  leftSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn);
  rightSensor = std::make_shared<Sensorvl53l5cx>(vl53l5cx_i2c, PIN_LPn2);

  //leftSensor->setAddress(0x16);


  leftSensor->initialize();
  rightSensor->initialize();
  //if (!(leftSensor->initialize() && rightSensor->initialize())) {
  //  return -1;
  //}

  printf("successfully initialized!");

  multicore_launch_core1(poll);

  while (true) {
    /*
    printf("%d --- %d\n", leftSensor->getLatestDistance(),
           rightSensor->getLatestDistance());
    */
    
    auto x = leftSensor->getLatestDistance();
    auto y = rightSensor->getLatestDistance();

    tx[0] = 1;
    tx[1] = 2;
    tx[2] = 3;
    tx[3] = 4;


    //std::memcpy(tx, &x, 2);
    //std::memcpy(tx+2, &y, 2);

  }
}
