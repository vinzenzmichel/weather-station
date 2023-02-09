#ifndef I2C_FUNCTIONS
#define I2C_FUNCTIONS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "i2c_funtions.h"
esp_err_t init_i2c();
uint8_t i2c_register_read (i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr );
esp_err_t i2c_register_write (i2c_port_t port_num, uint8_t dev_addr , uint8_t reg_addr , uint8_t data );
esp_err_t i2c_multi_register_read ( i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr , uint8_t * data ,size_t size );

#endif