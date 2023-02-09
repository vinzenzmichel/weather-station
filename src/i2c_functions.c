#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "i2c_funtions.h"

// define section
#define I2C_MASTER_PORT 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_MASTER_FREQ_HZ 1000000 // I2C frequency
int i2c_master_port = 0;

esp_err_t init_i2c();
uint8_t i2c_register_read (i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr );
esp_err_t i2c_register_write (i2c_port_t port_num, uint8_t dev_addr , uint8_t reg_addr , uint8_t data );
esp_err_t i2c_multi_register_read ( i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr , uint8_t * data ,size_t size );

// void app_main () {
//     esp_err_t err = init_i2c();
//     if (err != ESP_OK) {
//         printf("Init was not successfull\n");
//     }
//     while (1)
//     {
//         volatile uint8_t tsl_id = lux_read_sensor_id();
//         printf ("The ID is 0x%X\n", tsl_id);
//         lux_read_sensor_value();
//         vTaskDelay(200);
//     }
// }

esp_err_t init_i2c() {
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
    };
    i2c_param_config ( I2C_MASTER_PORT , &conf ) ;
    return i2c_driver_install ( I2C_MASTER_PORT , conf.mode , 0 , 0 , 0) ;
}



esp_err_t i2c_register_write (i2c_port_t port_num, uint8_t dev_addr , uint8_t reg_addr , uint8_t data ) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_write_byte(cmd, (dev_addr<<1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_write_byte(cmd, data, true);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_stop (cmd);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_cmd_begin (port_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return err;
    }
    i2c_cmd_link_delete (cmd);
    return err;
}

uint8_t i2c_register_read (i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr ) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        return 0xff;
    }
    err = i2c_master_write_byte(cmd, (dev_addr<<1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        return 0xff;
    }
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        return 0xff;
    }
    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        return 0xff;
    }
    err = i2c_master_write_byte(cmd, (dev_addr<<1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        return 0xff;
    }
    uint8_t data = 0;
    err = i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    if (err != ESP_OK) {
        return 0xff;
    }
    err = i2c_master_stop (cmd);
    if (err != ESP_OK) {
        return 0xff;
    }

    err = i2c_master_cmd_begin (port_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return 0xff;
    }
    i2c_cmd_link_delete (cmd);
    return data;
}

esp_err_t i2c_multi_register_read ( i2c_port_t port_num , uint8_t dev_addr , uint8_t reg_addr , uint8_t * data ,size_t size ) {
    if ( size < 1) {
        printf("Error: size argument must be bigger than 1");
        return 0;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_write_byte(cmd, (dev_addr<<1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        return err;
    }
    
    err = i2c_master_write_byte(cmd, (dev_addr<<1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        return err;
    }
    if (size > 1) {
        err = i2c_master_read(cmd, data, size-1, I2C_MASTER_ACK);
        if (err != ESP_OK) {
            return err;
        }
    }
    err = i2c_master_read_byte(cmd, (data+size-1), I2C_MASTER_NACK);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_stop (cmd);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_cmd_begin (port_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return err;
    }
    i2c_cmd_link_delete (cmd);
    return err;
}






