#include <stdio.h>
#include "esp_qmc5883l.h"

static const char* TAG = "QMC5883L";


/**
 * @brief send data to QMC5883L register
 * 
 * @param qmc struct with QMC5883L parameters
 * @param reg register to write to
 * @param data pointer data to write
 * @param data_len length of data to write
*/
static void i2c_send(qmc5883l_conf_t qmc, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    err = i2c_master_start(cmd);
    assert(err == ESP_OK);

    err = i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_WRITE, true);
    assert(err == ESP_OK);

    for (uint32_t i = 0; i < data_len; i++)
    {
        err = i2c_master_write_byte(cmd, reg + i, true);
        assert(err == ESP_OK);

        err = i2c_master_write_byte(cmd, *(data + i), true);
        assert(err == ESP_OK);
    }

    err = i2c_master_stop(cmd);
    assert(err == ESP_OK);

    err = i2c_master_cmd_begin(qmc.i2c_port, cmd, QMC5883L_TIMEOUT);
    assert(err == ESP_OK);

    i2c_cmd_link_delete(cmd);
}


/**
 * @brief read data from QMC5883L register
 * 
 * @param qmc struct with QMC5883L parameters
 * @param reg register to read from
 * @param data pointer to data to read
 * @param data_len length of data to read
*/
static void i2c_read(qmc5883l_conf_t qmc, uint8_t reg, uint8_t* data, uint32_t data_len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    err = i2c_master_start(cmd);
    assert(err == ESP_OK);

    err = i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_WRITE, true);
    assert(err == ESP_OK);

    err = i2c_master_write_byte(cmd, reg, true);
    assert(err == ESP_OK);

    err = i2c_master_start(cmd);
    assert(err == ESP_OK);

    err = i2c_master_write_byte(cmd, (QMC5883L_ADDR << 1) | I2C_MASTER_READ, true);
    assert(err == ESP_OK);

    err = i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    assert(err == ESP_OK);

    err = i2c_master_stop(cmd);
    assert(err == ESP_OK);

    err = i2c_master_cmd_begin(qmc.i2c_port, cmd, QMC5883L_TIMEOUT);
    assert(err == ESP_OK);

    i2c_cmd_link_delete(cmd);
}


/**
 * @brief initialize QMC5883L
 * 
 * @param bmp struct with BMP280 parameters
*/
void qmc5883l_init(qmc5883l_conf_t qmc)
{
    if (qmc.drdy_pin != -1)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << qmc.drdy_pin;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);


    }

    if (qmc.i2c_freq > QMC5883L_MAX_FREQ)
    {
        qmc.i2c_freq = QMC5883L_MAX_FREQ;
        ESP_LOGW(TAG, "I2C frequency too high, set to value: %d Hz", QMC5883L_MAX_FREQ);
    }

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = qmc.sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = qmc.scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = qmc.i2c_freq,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(qmc.i2c_port, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(qmc.i2c_port, i2c_config.mode, 0, 0, 0));
}


/**
 * @brief reset QMC5883L
 * 
 * @param qmc struct with QMC5883L parameters
*/
void qmc5883l_soft_reset(qmc5883l_conf_t qmc)
{
    uint8_t reset = QMC_RESET;
    i2c_send(qmc, QMC_CONTROL_2_REG, &reset, 1);

    vTaskDelay(10 / portTICK_PERIOD_MS);
}


/**
 * @brief configure QMC5883L
 * 
 * @param qmc struct with QMC5883L parameters
 * @param over_sample_ratio over sample ratio
 * @param full_scale full scale
 * @param output_rate data output rate
 * @param mode measurement mode
*/
void qmc5883l_configure(qmc5883l_conf_t qmc, enum qmc5883l_over_sample_ratio over_sample_ratio,
                            enum qmc5883l_full_scale full_scale, enum qmc5883l_data_output_rate output_rate,
                            enum qmc5883l_mode mode)
{
    uint8_t period = 0x01;
    i2c_send(qmc, QMC_PERIOD_REG, &period, 1);

    uint8_t control = (uint8_t)over_sample_ratio << 6 | (uint8_t)full_scale << 4 | (uint8_t)output_rate << 2 | (uint8_t)mode;

    i2c_send(qmc, QMC_CONTROL_1_REG, &control, 1);
}


/**
 * @brief read magnetometer data
 * 
 * @param qmc struct with QMC5883L parameters
 * @param x pointer to x axis data
 * @param y pointer to y axis data
 * @param z pointer to z axis data
*/
void qmc5883l_read_magnetometer(qmc5883l_conf_t qmc, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t data[6];
    i2c_read(qmc, QMC_DATA_OUT_X_LSB_REG, data, 6);

    *x = (int16_t)data[1] << 8 | (int16_t)data[0];
    *y = (int16_t)data[3] << 8 | (int16_t)data[2];
    *z = (int16_t)data[5] << 8 | (int16_t)data[4];
}