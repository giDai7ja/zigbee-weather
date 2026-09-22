#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>

#define I2C_NODE DT_NODELABEL(i2c0)
#define BMP280_NODE DT_NODELABEL(bmp280)
#define ADC_NODE DT_NODELABEL(adc)

#define AHT20_ADDR 0x38
#define AHT20_CMD_TRIGGER 0xAC

#define ADC_RESOLUTION 14

static int read_aht20(const struct device *i2c)
{
    uint8_t command[3] = {
        AHT20_CMD_TRIGGER,
        0x33,
        0x00
    };

    uint8_t raw[7];

    int ret = i2c_write(i2c, command, sizeof(command), AHT20_ADDR);
    if (ret) {
        return ret;
    }

    k_msleep(120);

    ret = i2c_read(i2c, raw, sizeof(raw), AHT20_ADDR);
    if (ret) {
        return ret;
    }

    uint32_t raw_humidity =
        ((uint32_t)raw[1] << 12) |
        ((uint32_t)raw[2] << 4) |
        ((uint32_t)raw[3] >> 4);

    uint32_t raw_temperature =
        ((uint32_t)(raw[3] & 0x0F) << 16) |
        ((uint32_t)raw[4] << 8) |
        raw[5];

    uint32_t humidity_x100 =
        ((uint64_t)raw_humidity * 10000ULL) / 1048576ULL;

    int32_t temperature_x100 =
        ((uint64_t)raw_temperature * 20000ULL) / 1048576ULL
        - 5000;

    printk("AHT20: T=%d.%02d C, RH=%u.%02u %%\n",
           temperature_x100 / 100,
           temperature_x100 % 100,
           humidity_x100 / 100,
           humidity_x100 % 100);

    return 0;
}

static int read_bmp280(const struct device *bmp280)
{
    struct sensor_value temperature;
    struct sensor_value pressure;

    int ret = sensor_sample_fetch(bmp280);
    if (ret) {
        return ret;
    }

    ret = sensor_channel_get(
        bmp280,
        SENSOR_CHAN_AMBIENT_TEMP,
        &temperature
    );
    if (ret) {
        return ret;
    }

    ret = sensor_channel_get(
        bmp280,
        SENSOR_CHAN_PRESS,
        &pressure
    );
    if (ret) {
        return ret;
    }

    printk("BMP280: T=%d.%02d C, P=%d.%03d kPa\n",
           temperature.val1,
           temperature.val2 / 10000,
           pressure.val1,
           pressure.val2 / 1000);

    return 0;
}

static int read_vdd(const struct device *adc)
{
    struct adc_channel_cfg channel_cfg = {
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 0,
        .input_positive = NRF_SAADC_VDD,
    };

    int ret = adc_channel_setup(adc, &channel_cfg);
    if (ret) {
        return ret;
    }

    int16_t sample = 0;

    struct adc_sequence sequence = {
        .channels = BIT(channel_cfg.channel_id),
        .buffer = &sample,
        .buffer_size = sizeof(sample),
        .resolution = ADC_RESOLUTION,
    };

    ret = adc_read(adc, &sequence);
    if (ret) {
        return ret;
    }

    int32_t vdd_mv =
        ((int32_t)sample * 3600) / (1 << ADC_RESOLUTION);

    printk("VDD: %d.%03d V\n",
           vdd_mv / 1000,
           vdd_mv % 1000);

    return 0;
}

int main(void)
{
    const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
    const struct device *bmp280 = DEVICE_DT_GET(BMP280_NODE);
    const struct device *adc = DEVICE_DT_GET(ADC_NODE);

    while (1) {
        read_vdd(adc);
        read_aht20(i2c);
        read_bmp280(bmp280);

        printk("\n");

        k_sleep(K_SECONDS(5));
    }

    return 0;
}