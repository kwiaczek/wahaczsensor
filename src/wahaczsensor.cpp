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

static struct{
    uint8_t address;
    uint8_t memory[0xFF + 1];
    bool address_set;
} slave_i2c_data;

static void i2cHandler(i2c_inst_t *i2c, i2c_slave_event_t event) {
  /*if (event == I2C_SLAVE_REQUEST)
  {
    printf("REQUEST FULLFIELED! CALLBACK FROM HELL!\n %d %d \n", (I2C_SLAVE_REQUEST == event), event);
	  i2c_write_raw_blocking(i2c, tx, 4);
  }*/
  switch (event)
  {
  case I2C_SLAVE_RECEIVE:
    if (!slave_i2c_data.address_set) {
        slave_i2c_data.address = i2c_read_byte_raw(i2c);
        slave_i2c_data.address_set = true;
      }
    else {
        slave_i2c_data.memory[slave_i2c_data.address] = i2c_read_byte_raw(i2c);
      }
    break;
  case I2C_SLAVE_REQUEST: 
        i2c_write_byte_raw(i2c, slave_i2c_data.memory[slave_i2c_data.address++]);
    break;
  case I2C_SLAVE_FINISH:
        slave_i2c_data.address_set = false;
        slave_i2c_data.address = 0;
        break;
  default:
    printf("Unrecognised request: %d\n", event);
    break;
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
    
    int16_t x = leftSensor->getLatestDistance();
    int16_t y = rightSensor->getLatestDistance();

    i2c_slave_data.memory[0] = x >> 8;
    i2c_slave_data.memory[1] = x & 0xF;
    i2c_slave_data.memory[2] = y >> 8;
    i2c_slave_data.memory[3] = y & 0xF;


    //std::memcpy(tx, &x, 2);
    //std::memcpy(tx+2, &y, 2);

  }
}
