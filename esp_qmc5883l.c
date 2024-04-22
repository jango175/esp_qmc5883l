/**
 * @file esp_qmc5883l.c
 * @author JanG175
 * @brief ESP-IDF component for QMC5883L magnetometer
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_qmc5883l.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool measurement_ready = false;

static bool compensation = false;

#ifdef QMC5883L_I2C_INIT
static i2c_master_bus_handle_t bus_handle;
#else
extern i2c_master_bus_handle_t bus_handle;
#endif
static i2c_master_dev_handle_t dev_handle;

static const char* TAG = "QMC5883L";


/**
 * @brief interrupt handler for magnetometer readings
 * 
 * @param arg pointer to QMC5883L parameters
*/
static void isr_handler(void* arg)
{
    portENTER_CRITICAL(&mux);
    measurement_ready = true;
    portEXIT_CRITICAL(&mux);
}


/**
 * @brief initialize QMC5883L
 * 
 * @param qmc struct with QMC5883L parameters
*/
void qmc5883l_init(qmc5883l_conf_t qmc)
{
#ifdef QMC5883L_I2C_INIT
    if (qmc.i2c_freq > QMC5883L_MAX_FREQ)
    {
        qmc.i2c_freq = QMC5883L_MAX_FREQ;
        ESP_LOGW(TAG, "I2C frequency too high, set to value: %d Hz", QMC5883L_MAX_FREQ);
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = qmc.i2c_port,
        .scl_io_num = qmc.scl_pin,
        .sda_io_num = qmc.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
#endif

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883L_ADDR,
        .scl_speed_hz = qmc.i2c_freq
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // interrupt mode
    if (qmc.drdy_pin != -1)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << qmc.drdy_pin;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_isr_handler_add(qmc.drdy_pin, isr_handler, (void*)&qmc));

        qmc5883l_write_control(qmc, QMC5883L_OVER_SAMPLE_RATIO_512, QMC5883L_FULL_SCALE_8G,
                                QMC5883L_DATA_OUTPUT_RATE_200, QMC5883L_CONTINUOUS_MODE, 
                                QMC5883L_POINTER_ROLLOVER_FUNCTION_NORMAL, QMC5883L_INTERRUPT_ENABLE);
    }
}


/**
 * @brief reset QMC5883L
 * 
 * @param qmc struct with QMC5883L parameters
*/
void qmc5883l_soft_reset(qmc5883l_conf_t qmc)
{
    uint8_t data[2] = {QMC_CONTROL_2_REG, QMC_RESET};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, QMC5883L_TIMEOUT_MS));

    vTaskDelay(10 / portTICK_PERIOD_MS);
}


/**
 * @brief read magnetometer raw data
 * 
 * @param qmc struct with QMC5883L parameters
 * @param x pointer to x axis data
 * @param y pointer to y axis data
 * @param z pointer to z axis data
*/
void qmc5883l_read_raw_magnetometer(qmc5883l_conf_t qmc, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t data[6];
    uint8_t reg = QMC_DATA_OUT_X_LSB_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 6, QMC5883L_TIMEOUT_MS));

    *x = (int16_t)data[1] << 8 | (int16_t)data[0];
    *y = (int16_t)data[3] << 8 | (int16_t)data[2];
    *z = (int16_t)data[5] << 8 | (int16_t)data[4];

    if (qmc.drdy_pin != -1)
    {
        portENTER_CRITICAL(&mux);
        measurement_ready = false;
        portEXIT_CRITICAL(&mux);
    }
}


/**
 * @brief read QMC5883L status
 * 
 * @param qmc struct with QMC5883L parameters
 * @param status pointer to status data
*/
void qmc5883l_read_status(qmc5883l_conf_t qmc, uint8_t* status)
{
    uint8_t reg = QMC_STATUS_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, status, 1, QMC5883L_TIMEOUT_MS));

    uint8_t dor = (*status & (1 << 2)) >> 2;
    uint8_t ovl = (*status & (1 << 1)) >> 1;
    uint8_t drdy = *status & 1;

    if (drdy == 1)
        ESP_LOGI(TAG, "New data available");
    else
        ESP_LOGI(TAG, "No new data available");

    if (ovl == 1)
        ESP_LOGI(TAG, "Data overflow");
    else
        ESP_LOGI(TAG, "Normal - no data overflow");

    if (dor == 1)
        ESP_LOGI(TAG, "Data skipped for reading");
    else
        ESP_LOGI(TAG, "Normal - no data skipped for reading");
}


/**
 * @brief read QMC5883L termometer data
 * 
 * @param qmc struct with QMC5883L parameters
 * @param temp pointer to temperature data
*/
void qmc5883l_read_termometer(qmc5883l_conf_t qmc, int16_t* temp)
{
    uint8_t data[2];
    uint8_t reg = QMC_TEMP_DATA_LSB_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, QMC5883L_TIMEOUT_MS));

    *temp = (int16_t)data[1] << 8 | (int16_t)data[0];
}


/**
 * @brief configure QMC5883L
 * 
 * @param qmc struct with QMC5883L parameters
 * @param over_sample_ratio over sample ratio
 * @param full_scale full scale
 * @param output_rate data output rate
 * @param mode measurement mode
 * @param rol_pnt pointer rollover function
 * @param int_enable interrupt enable
*/
void qmc5883l_write_control(qmc5883l_conf_t qmc, enum qmc5883l_over_sample_ratio over_sample_ratio,
                            enum qmc5883l_full_scale full_scale, enum qmc5883l_data_output_rate output_rate,
                            enum qmc5883l_mode mode, enum qmc5883l_pointer_rollover_function rol_pnt,
                            enum qmc5883l_interrupt int_enable)
{
    qmc5883l_write_set_reset_period_FBR(qmc);

    // set control register 1

    uint8_t control = (uint8_t)over_sample_ratio << 6 | (uint8_t)full_scale << 4 | (uint8_t)output_rate << 2 | (uint8_t)mode;
    uint8_t data[2] = {QMC_CONTROL_1_REG, control};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, QMC5883L_TIMEOUT_MS));

    vTaskDelay(1);

    // set control register 2

    uint8_t reg = QMC_CONTROL_2_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, &control, 1, QMC5883L_TIMEOUT_MS));

    vTaskDelay(1);

    if (rol_pnt == QMC5883L_POINTER_ROLLOVER_FUNCTION_NORMAL)
        control &= ~(1 << 6);
    else if (rol_pnt == QMC5883L_POINTER_ROLLOVER_FUNCTION_ENABLE)
        control |= (1 << 6);

    if (int_enable == QMC5883L_INTERRUPT_ENABLE)
        control &= ~1;
    else if (int_enable == QMC5883L_INTERRUPT_DISABLE)
        control |= 1;

    data[0] = QMC_CONTROL_2_REG;
    data[1] = control;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, QMC5883L_TIMEOUT_MS));
}


/**
 * @brief read QMC5883L control 1 register
 * 
 * @param qmc struct with QMC5883L parameters
 * @param control_1 pointer to control 1 data
*/
void qmc5883l_read_control_1(qmc5883l_conf_t qmc, uint8_t* control_1)
{
    uint8_t reg = QMC_CONTROL_1_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, control_1, 1, QMC5883L_TIMEOUT_MS));

    uint8_t osr = *control_1 >> 6;
    uint8_t rng  = *control_1 << 2;
    rng = rng >> 6;
    uint8_t odr = *control_1 << 4;
    odr = odr >> 6;
    uint8_t mode = *control_1 << 6;
    mode = mode >> 6;

    switch (osr)
    {
        case QMC5883L_OVER_SAMPLE_RATIO_512:
            ESP_LOGI(TAG, "Over sample ratio: 512");
            break;
        case QMC5883L_OVER_SAMPLE_RATIO_256:
            ESP_LOGI(TAG, "Over sample ratio: 256");
            break;
        case QMC5883L_OVER_SAMPLE_RATIO_128:
            ESP_LOGI(TAG, "Over sample ratio: 128");
            break;
        case QMC5883L_OVER_SAMPLE_RATIO_64:
            ESP_LOGI(TAG, "Over sample ratio: 64");
            break;
    }

    switch (rng)
    {
        case QMC5883L_FULL_SCALE_2G:
            ESP_LOGI(TAG, "Full scale: 2G");
            break;
        case QMC5883L_FULL_SCALE_8G:
            ESP_LOGI(TAG, "Full scale: 8G");
            break;
    }

    switch (odr)
    {
        case QMC5883L_DATA_OUTPUT_RATE_10:
            ESP_LOGI(TAG, "Data output rate: 10 Hz");
            break;
        case QMC5883L_DATA_OUTPUT_RATE_50:
            ESP_LOGI(TAG, "Data output rate: 50 Hz");
            break;
        case QMC5883L_DATA_OUTPUT_RATE_100:
            ESP_LOGI(TAG, "Data output rate: 100 Hz");
            break;
        case QMC5883L_DATA_OUTPUT_RATE_200:
            ESP_LOGI(TAG, "Data output rate: 200 Hz");
            break;
    }

    switch (mode)
    {
        case QMC5883L_STANDBY_MODE:
            ESP_LOGI(TAG, "Mode: Standby");
            break;
        case QMC5883L_CONTINUOUS_MODE:
            ESP_LOGI(TAG, "Mode: Continuous");
            break;
    }
}


/**
 * @brief read QMC5883L control 2 register
 * 
 * @param qmc struct with QMC5883L parameters
 * @param control_2 pointer to control 2 data
*/
void qmc5883l_read_control_2(qmc5883l_conf_t qmc, uint8_t* control_2)
{
    uint8_t reg = QMC_CONTROL_2_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, control_2, 1, QMC5883L_TIMEOUT_MS));

    uint8_t rol = (*control_2 & (1 << 6)) >> 6;
    uint8_t int_en = *control_2 & 1;

    if (rol == 1)
        ESP_LOGI(TAG, "Pointer rollover function enabled");
    else
        ESP_LOGI(TAG, "Pointer rollover function: normal");

    if (int_en == 1)
        ESP_LOGI(TAG, "Interrupt disable");
    else
        ESP_LOGI(TAG, "Interrupt enable");
}


/**
 * @brief read QMC5883L set/reset period register
 * 
 * @param qmc struct with QMC5883L parameters
*/
void qmc5883l_write_set_reset_period_FBR(qmc5883l_conf_t qmc)
{
    uint8_t data[2] = {QMC_PERIOD_REG, QMC_PERIOD};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, QMC5883L_TIMEOUT_MS));
}


/**
 * @brief read QMC5883L chip id
 * 
 * @param qmc struct with QMC5883L parameters
 * @param chip_id pointer to chip id data
*/
void qmc5883l_read_chip_id(qmc5883l_conf_t qmc, uint8_t* chip_id)
{
    uint8_t reg = QMC_CHIP_ID_REG;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, chip_id, 1, QMC5883L_TIMEOUT_MS));
}


/**
 * @brief calibrate QMC5883L (10 seconds of stable measurements)
 * 
 * @param qmc struct with QMC5883L parameters
*/
void qmc5883l_calibrate(qmc5883l_conf_t qmc)
{
    portENTER_CRITICAL(&mux);
    compensation = true;
    portEXIT_CRITICAL(&mux);

    int16_t mag_cal[3][2]; // 0 - min, 1 - max
    for (uint32_t i = 0; i < 3; i++)
    {
        mag_cal[i][0] = 32767;
        mag_cal[i][1] = -32768;
    }

    int16_t mag_val[3] = {0, 0, 0};
    bool is_changed[3] = {false, false, false};
    uint32_t count = 0;

    ESP_LOGW(TAG, "Calibration started - move the sensor in all directions...");

    while (count < 1000) // while measurements are not stable for 10 seconds
    {
        qmc5883l_read_raw_magnetometer(qmc, mag_val, mag_val + 1, mag_val + 2);

        for (uint8_t i = 0; i < 3; i++)
        {
            if (mag_val[i] < mag_cal[i][0])
            {
                mag_cal[i][0] = mag_val[i];
                is_changed[i] = true;
            }
            else
                is_changed[i] = false;

            if (mag_val[i] > mag_cal[i][1])
            {
                mag_cal[i][1] = mag_val[i];
                is_changed[i] = true;
            }
            else
                is_changed[i] = false;
        }

        if (is_changed[0] == false && is_changed[1] == false && is_changed[2] == false)
            count++;
        else
            count = 0;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // calculate compensation values
    float avg_delta[3];
    float cal_offset[3];
    float cal_scale[3];

    for (uint32_t i = 0; i < 3; i++)
        avg_delta[i] = (float)(mag_cal[i][1] - mag_cal[i][0]) / 2.0f;

    float total_avg_delta = (avg_delta[0] + avg_delta[1] + avg_delta[2]) / 3.0f;

    for (uint32_t i = 0; i < 3; i++)
    {
        cal_offset[i] = (mag_cal[i][0] + mag_cal[i][1]) / 2.0f;
        cal_scale[i] = total_avg_delta / avg_delta[i];
    }

    ESP_LOGW(TAG, "Calibration ended");

    ESP_LOGI(TAG, "Paste these values to esp_qmc5883l.h compensation values section:");
    ESP_LOGI(TAG, "#define QMC5883L_X_OFFSET            %f", cal_offset[0]);
    ESP_LOGI(TAG, "#define QMC5883L_Y_OFFSET            %f", cal_offset[1]);
    ESP_LOGI(TAG, "#define QMC5883L_Z_OFFSET            %f", cal_offset[2]);

    ESP_LOGI(TAG, "#define QMC5883L_X_SCALE             %f", cal_scale[0]);
    ESP_LOGI(TAG, "#define QMC5883L_Y_SCALE             %f", cal_scale[1]);
    ESP_LOGI(TAG, "#define QMC5883L_Z_SCALE             %f", cal_scale[2]);
}


/**
 * @brief get azimuth (XY plane) from magnetometer readings
 *
 * @param qmc struct with QMC5883L parameters
 *
 * @return azimuth
*/
int32_t qmc5883l_get_azimuth(qmc5883l_conf_t qmc)
{
    float x, y, z;
    qmc5883l_read_magnetometer(qmc, &x, &y, &z);

    float heading = atan2(y, x) * 180.0 / M_PI;
    heading += QMC5883L_MAG_DEC_DEG;

    return (int32_t)heading % 360;
}


/**
 * @brief read magnetometer compensated data
 * 
 * @param qmc struct with QMC5883L parameters
 * @param x pointer to x axis data
 * @param y pointer to y axis data
 * @param z pointer to z axis data
*/
void qmc5883l_read_magnetometer(qmc5883l_conf_t qmc, float* x, float* y, float* z)
{
    int16_t x_raw, y_raw, z_raw;
    qmc5883l_read_raw_magnetometer(qmc, &x_raw, &y_raw, &z_raw);

    *x = ((float)x_raw - QMC5883L_X_OFFSET) * QMC5883L_X_SCALE;
    *y = ((float)y_raw - QMC5883L_Y_OFFSET) * QMC5883L_Y_SCALE;
    *z = ((float)z_raw - QMC5883L_Z_OFFSET) * QMC5883L_Z_SCALE;

    if (qmc.drdy_pin != -1)
    {
        portENTER_CRITICAL(&mux);
        measurement_ready = false;
        portEXIT_CRITICAL(&mux);
    }
}