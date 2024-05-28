#include <stdio.h>
#include "pico/stdlib.h"
#include <platform.h>
#include <unistd.h>
#include <vl53l5cx_api.h>

#include "hardware/i2c.h"



#define I2C_SDA 0
#define I2C_SCL 1
#define PIN_LPn 2
#define PIN_LPn2 3
i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};

int main(void) {
    // pico settings
    stdio_init_all();
    sleep_ms(3000);
	puts("eloooo\n");

    i2c_init(&vl53l5cx_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

	gpio_init(PIN_LPn);
    gpio_set_dir(PIN_LPn, GPIO_OUT);
    gpio_put(PIN_LPn, 0);

	gpio_init(PIN_LPn2);
    gpio_set_dir(PIN_LPn2, GPIO_OUT);
    gpio_put(PIN_LPn2, 1);

	uint8_t 				status;
	VL53L5CX_Configuration 	config;			/* Sensor configuration */
	VL53L5CX_Configuration 	config2;			/* Sensor configuration */
	VL53L5CX_ResultsData 	results;		/* Results data from VL53L5CX */

	uint16_t secondSensorAddress = 0x16;
	config.platform.address = (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    config.platform.i2c     = &vl53l5cx_i2c; // set i2c bus
	config2.platform.address = (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    config2.platform.i2c     = &vl53l5cx_i2c; // set i2c bus

	status = vl53l5cx_set_i2c_address(&config2, secondSensorAddress);

	if (status == VL53L5CX_STATUS_OK)
	{
		printf("ZMIENIONO ADRES");
	}
    gpio_put(PIN_LPn, 1);

	config2.platform.address = (uint8_t)((secondSensorAddress >> 1) & 0xFF);


    uint8_t isAlive;
	status = vl53l5cx_is_alive(&config, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX was not detected\n");
		return status;
	}
	status = vl53l5cx_is_alive(&config2, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX (2) was not detected\n");
		return status;
	}

	status = vl53l5cx_init(&config);
	if(status)
	{
		printf("VL53L5CX could not be initalized\n");
		return status;
	}
	status = vl53l5cx_init(&config2);
	if(status)
	{
		printf("VL53L5CX (2) could not be initalized\n");
		return status;
	}

	printf("VL53L5CX initalized!\n");

	status = vl53l5cx_start_ranging(&config);
	status = vl53l5cx_start_ranging(&config2);

    uint8_t isReady;
	while(true)
	{
		status = vl53l5cx_check_data_ready(&config, &isReady);

		if(isReady)
		{
		printf("sensor adin\n");
			vl53l5cx_get_ranging_data(&config, &results);

			for(int i = 0; i < 16; i++)
			{
				printf("%4d \t",
					results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
                if ((i+1)%4==0 && i > 0)
                    printf("\n");
			}
            printf("\n");
		}

		status = vl53l5cx_check_data_ready(&config2, &isReady);

		if(isReady)
		{
		printf("sensor dva\n");
			vl53l5cx_get_ranging_data(&config2, &results);

			for(int i = 0; i < 16; i++)
			{
				printf("%4d \t",
					results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
                if ((i+1)%4==0 && i > 0)
                    printf("\n");
			}
            printf("\n");
		}

		WaitMs(&(config.platform), 1000);
	}

	status = vl53l5cx_stop_ranging(&config);
	status = vl53l5cx_stop_ranging(&config2);
	return status;
}
