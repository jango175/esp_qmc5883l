#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define QMC5883L_MAX_FREQ      400000
#define QMC5883L_TIMEOUT       (100 / portTICK_PERIOD_MS)
#define QMC5883L_ADDR          0x0D

// register map
#define QMC_DATA_OUT_X_LSB_REG 0x00 // Data Output X LSB Register Read
#define QMC_DATA_OUT_X_MSB_REG 0x01 // Data Output X MSB Register Read
#define QMC_DATA_OUT_Y_LSB_REG 0x02 // Data Output Y LSB Register Read
#define QMC_DATA_OUT_Y_MSB_REG 0x03 // Data Output Y MSB Register Read
#define QMC_DATA_OUT_Z_LSB_REG 0x04 // Data Output Z LSB Register Read
#define QMC_DATA_OUT_Z_MSB_REG 0x05 // Data Output Z MSB Register Read
#define QMC_STATUS_REG         0x06 // Status Register Read
#define QMC_TEMP_DATA_LSB_REG  0x07 // Temperature Data LSB Register Read
#define QMC_TEMP_DATA_MSB_REG  0x08 // Temperature Data MSB Register Read
#define QMC_CONTROL_1_REG      0x09 // Control Register 1 Read/Write
#define QMC_CONTROL_2_REG      0x0A // Control Register 2 Read/Write
#define QMC_PERIOD_REG         0x0B // SET/RESET Period Register Read/Write
#define QMC_CHIP_ID_REG        0x0D // Chip ID Register Read

#define QMC_RESET              0x80
#define QMC_PERIOD             0x01

enum qmc5883l_mode
{
    QMC5883L_STANDBY_MODE = 0,
    QMC5883L_CONTINUOUS_MODE
};

enum qmc5883l_data_output_rate
{
    QMC5883L_DATA_OUTPUT_RATE_10 = 0,
    QMC5883L_DATA_OUTPUT_RATE_50,
    QMC5883L_DATA_OUTPUT_RATE_100,
    QMC5883L_DATA_OUTPUT_RATE_200
};

enum qmc5883l_full_scale
{
    QMC5883L_FULL_SCALE_2G = 0,
    QMC5883L_FULL_SCALE_8G
};

enum qmc5883l_over_sample_ratio
{
    QMC5883L_OVER_SAMPLE_RATIO_512 = 0,
    QMC5883L_OVER_SAMPLE_RATIO_256,
    QMC5883L_OVER_SAMPLE_RATIO_128,
    QMC5883L_OVER_SAMPLE_RATIO_64
};

enum qmc5883l_pointer_rollover_function
{
    QMC5883L_POINTER_ROLLOVER_FUNCTION_NORMAL = 0,
    QMC5883L_POINTER_ROLLOVER_FUNCTION_ENABLE
};

enum qmc5883l_interrupt
{
    QMC5883L_INTERRUPT_ENABLE = 0,
    QMC5883L_INTERRUPT_DISABLE
};

// QMC5883L configuration struct
typedef struct qmc5883l_conf_t
{
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
    gpio_num_t drdy_pin;
} qmc5883l_conf_t;


void qmc5883l_init(qmc5883l_conf_t qmc);

void qmc5883l_soft_reset(qmc5883l_conf_t qmc);

void qmc5883l_read_magnetometer(qmc5883l_conf_t qmc, int16_t* x, int16_t* y, int16_t* z);

void qmc5883l_read_status(qmc5883l_conf_t qmc, uint8_t* status);

void qmc5883l_read_termometer(qmc5883l_conf_t qmc, int16_t* temp);

void qmc5883l_write_control(qmc5883l_conf_t qmc, enum qmc5883l_over_sample_ratio over_sample_ratio,
                            enum qmc5883l_full_scale full_scale, enum qmc5883l_data_output_rate output_rate,
                            enum qmc5883l_mode mode, enum qmc5883l_pointer_rollover_function rol_pnt,
                            enum qmc5883l_interrupt int_enable);

void qmc5883l_read_control_1(qmc5883l_conf_t qmc, uint8_t* control_1);

void qmc5883l_read_control_2(qmc5883l_conf_t qmc, uint8_t* control_2);

void qmc5883l_write_set_reset_period_FBR(qmc5883l_conf_t qmc);

void qmc5883l_read_chip_id(qmc5883l_conf_t qmc, uint8_t* chip_id);
