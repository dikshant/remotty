/**
 * Wrapper module for source code compatibility with esp-open-rtos.
 */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include <sys/time.h>
#include <string.h>

#include "driver/i2c.h"

#include "esp8266_wrapper.h"

// esp-open-rtos SDK function wrapper

uint32_t sdk_system_get_time ()
{
    struct timeval time;
    gettimeofday(&time,0);
    return time.tv_sec*1e6 + time.tv_usec;
}


// esp-open-rtos I2C interface wrapper

#define I2C_ACK_VAL  0x0
#define I2C_NACK_VAL 0x1

void i2c_init (int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    conf.clk_flags = 0;
    i2c_param_config(bus, &conf);
    i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
}

int i2c_slave_write (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                     uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    if (reg)
        i2c_master_write_byte(cmd, *reg, true);
    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return err;
}

int i2c_slave_read (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                    uint8_t *data, uint32_t len)
{
    if (len == 0) return true;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (reg)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, *reg, true);
        if (!data)
            i2c_master_stop(cmd);
    }
    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_READ, true);
        if (len > 1) i2c_master_read(cmd, data, len-1, I2C_ACK_VAL);
        i2c_master_read_byte(cmd, data + len-1, I2C_NACK_VAL);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

#endif  // ESP32 (ESP-IDF)

