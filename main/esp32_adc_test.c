#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "driver/i2c.h"

#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_sleep.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include <stdlib.h>
#include <string.h>

#define TEST_VOLT_MAX       3.3
#define LTC2606_REF_VOLT        4.096

#define ADC_VREF            1100            // ADC calibration data
#define ADC_CH              ADC1_CHANNEL_4  // GPIO 32
#define I2C_SDA             GPIO_NUM_33
#define I2C_SCL             GPIO_NUM_25
#define I2C_FREQ            100000           // 100kHz

#define TAG "adc_test"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

typedef struct atten_conf {
    adc_atten_t atten;
    float threshold;
} atten_conf_t;

atten_conf_t ATTEN_CONF[] = {
    { ADC_ATTEN_DB_11,  2.0     },
    { ADC_ATTEN_DB_6,   1.34    },
    { ADC_ATTEN_DB_2_5, 1.0     },
    { ADC_ATTEN_DB_0,   0       },
};

//////////////////////////////////////////////////////////////////////
// LTC2606
static void dac_set_value(float volt)
{
    i2c_cmd_handle_t cmd;
    uint32_t reg_val;

    reg_val = (uint32_t)(0xFFFF * volt) / LTC2606_REF_VOLT;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x20|I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x30, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_val >> 8, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_val & 0xFF, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    /* ESP_LOGI(TAG, "DAC Set voltage = %.2f", volt); */

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));

    i2c_cmd_link_delete(cmd);
}

//////////////////////////////////////////////////////////////////////
// ADS1015
static float ads1015_get_value()
{
    i2c_cmd_handle_t cmd;
    uint8_t reg_val[2];
    uint16_t val;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x90|I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x01, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xc3, 1)); // 0x83
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(100 / portTICK_RATE_MS); // wait 100ms

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x90|I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x90|I2C_MASTER_READ, 1));
    ESP_ERROR_CHECK(i2c_master_read(cmd, (uint8_t *)&reg_val, 2 , 0));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    return ((reg_val[0] << 4) | (reg_val[1] >> 4)) * 2 / 1000.0;
}

//////////////////////////////////////////////////////////////////////
static float adc_get_value_impl()
{
    float val;
    esp_adc_cal_characteristics_t adc_char;

    adc1_config_width(ADC_WIDTH_BIT_12);

    for (uint32_t i = 0 ; i < ARRAY_SIZE(ATTEN_CONF); i++) {
        adc1_config_channel_atten(ADC_CH, ATTEN_CONF[i].atten);
        esp_adc_cal_get_characteristics(
            ADC_VREF, ATTEN_CONF[i].atten, ADC_WIDTH_BIT_12, &adc_char
        );

        val = adc1_to_voltage(ADC_CH, &adc_char) / 1000.0;

        if (val > ATTEN_CONF[i].threshold) {
            break;
        }
    }
    return val;
}

static float adc_get_value(uint32_t count)
{
    float val = 0;

    for (uint32_t i = 0; i < count; i++) {
        val += adc_get_value_impl();
    }
    return val / count;
}

//////////////////////////////////////////////////////////////////////
static void init_io()
{
    i2c_config_t conf;
    // I2C
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void app_main()
{
    float dac_volt;
    float adc_char_one_volt = 0;
    float adc_char_ave_volt = 0;
    float adc_raw_ave_volt = 0;

    init_io();

    dac_volt = 0.0;
    while (dac_volt <= TEST_VOLT_MAX) {
        dac_set_value(dac_volt);

        vTaskDelay(500 / portTICK_RATE_MS); // wait 500ms

        adc_char_one_volt = adc_get_value(1);
        adc_char_ave_volt = adc_get_value(256);

        adc_raw_ave_volt = adc1_get_raw(ADC_CH) * 3.9 /4095;

        printf("DAC, ADC_char, ADC_ave, ADS1015 = (%.4f, %.4f, %.4f, %.4f)\n",
               dac_volt, adc_char_one_volt, adc_char_ave_volt,
               ads1015_get_value()
            );

        vTaskDelay(500 / portTICK_RATE_MS); // wait 500ms

        dac_volt += 0.05;
    }
}
