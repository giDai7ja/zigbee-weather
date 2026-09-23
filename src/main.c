#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_nrf_platform.h>
#include <zigbee/zigbee_app_utils.h>

#define I2C_NODE DT_NODELABEL(i2c0)
#define BMP280_NODE DT_NODELABEL(bmp280)
#define ADC_NODE DT_NODELABEL(adc)

#define AHT20_ADDR 0x38
#define AHT20_CMD_TRIGGER 0xAC

#define ADC_RESOLUTION 14

#define WEATHER_ENDPOINT 1
#define WEATHER_IN_CLUSTER_COUNT 5
#define WEATHER_OUT_CLUSTER_COUNT 0

/*
 * Keep Zigbee network parameters after reboot.
 * Set to ZB_TRUE only when a full factory reset is required.
 */
#define ERASE_PERSISTENT_CONFIG ZB_FALSE

static zb_int16_t temperature;
static zb_int16_t pressure;
static zb_uint16_t humidity;

static zb_uint8_t battery_voltage;
static zb_uint8_t battery_size;
static zb_uint8_t battery_quantity;
static zb_uint8_t battery_rated_voltage;
static zb_uint8_t battery_alarm_mask;
static zb_uint8_t battery_voltage_min_threshold;

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(
    basic_attr_list,
    ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE
);

ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(
    temperature_attr_list,
    &temperature,
    NULL,
    NULL,
    NULL
);

ZB_ZCL_DECLARE_PRESSURE_MEASUREMENT_ATTRIB_LIST(
    pressure_attr_list,
    &pressure,
    NULL,
    NULL,
    NULL
);

ZB_ZCL_DECLARE_REL_HUMIDITY_MEASUREMENT_ATTRIB_LIST(
    humidity_attr_list,
    &humidity,
    NULL,
    NULL
);

ZB_ZCL_DECLARE_POWER_CONFIG_ATTRIB_LIST(
    power_config_attr_list,
    &battery_voltage,
    &battery_size,
    &battery_quantity,
    &battery_rated_voltage,
    &battery_alarm_mask,
    &battery_voltage_min_threshold
);

static zb_zcl_cluster_desc_t cluster_list[] = {
    ZB_ZCL_CLUSTER_DESC(
        ZB_ZCL_CLUSTER_ID_BASIC,
        ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),
        basic_attr_list,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_MANUF_CODE_INVALID
    ),

    ZB_ZCL_CLUSTER_DESC(
        ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ZB_ZCL_ARRAY_SIZE(temperature_attr_list, zb_zcl_attr_t),
        temperature_attr_list,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_MANUF_CODE_INVALID
    ),

    ZB_ZCL_CLUSTER_DESC(
        ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ZB_ZCL_ARRAY_SIZE(pressure_attr_list, zb_zcl_attr_t),
        pressure_attr_list,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_MANUF_CODE_INVALID
    ),

    ZB_ZCL_CLUSTER_DESC(
        ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ZB_ZCL_ARRAY_SIZE(humidity_attr_list, zb_zcl_attr_t),
        humidity_attr_list,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_MANUF_CODE_INVALID
    ),

    ZB_ZCL_CLUSTER_DESC(
        ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        ZB_ZCL_ARRAY_SIZE(power_config_attr_list, zb_zcl_attr_t),
        power_config_attr_list,
        ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ZCL_MANUF_CODE_INVALID
    )
};

typedef ZB_PACKED_PRE struct weather_simple_desc_s {
    zb_uint8_t endpoint;
    zb_uint16_t app_profile_id;
    zb_uint16_t app_device_id;
    zb_bitfield_t app_device_version : 4;
    zb_bitfield_t reserved : 4;
    zb_uint8_t app_input_cluster_count;
    zb_uint8_t app_output_cluster_count;
    zb_uint16_t app_cluster_list[
        WEATHER_IN_CLUSTER_COUNT + WEATHER_OUT_CLUSTER_COUNT
    ];
} ZB_PACKED_STRUCT weather_simple_desc_t;

static weather_simple_desc_t simple_desc_weather = {
    WEATHER_ENDPOINT,
    ZB_AF_HA_PROFILE_ID,
    ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
    0,
    0,
    WEATHER_IN_CLUSTER_COUNT,
    WEATHER_OUT_CLUSTER_COUNT,
    {
        ZB_ZCL_CLUSTER_ID_BASIC,
        ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        ZB_ZCL_CLUSTER_ID_POWER_CONFIG
    }
};

ZB_AF_DECLARE_ENDPOINT_DESC(
    weather_endpoint,
    WEATHER_ENDPOINT,
    ZB_AF_HA_PROFILE_ID,
    0,
    NULL,
    ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),
    cluster_list,
    (zb_af_simple_desc_1_1_t *)&simple_desc_weather,
    0,
    NULL,
    0,
    NULL
);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(
    weather_device_ctx,
    weather_endpoint
);

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
        ((uint64_t)raw_temperature * 20000ULL) / 1048576ULL - 5000;

    printk(
        "AHT20: T=%d.%02d C, RH=%u.%02u %%\n",
        temperature_x100 / 100,
        temperature_x100 % 100,
        humidity_x100 / 100,
        humidity_x100 % 100
    );

    /*
     * Zigbee Temperature Measurement:
     * 0.01 degree C units.
     */
    temperature = (zb_int16_t)temperature_x100;

    /*
     * Zigbee Relative Humidity Measurement:
     * 0.01 % units.
     */
    humidity = (zb_uint16_t)humidity_x100;

    return 0;
}

static int read_bmp280(const struct device *bmp280)
{
    struct sensor_value temperature_value;
    struct sensor_value pressure_value;

    int ret = sensor_sample_fetch(bmp280);
    if (ret) {
        return ret;
    }

    ret = sensor_channel_get(
        bmp280,
        SENSOR_CHAN_AMBIENT_TEMP,
        &temperature_value
    );
    if (ret) {
        return ret;
    }

    ret = sensor_channel_get(
        bmp280,
        SENSOR_CHAN_PRESS,
        &pressure_value
    );
    if (ret) {
        return ret;
    }

    printk(
        "BMP280: T=%d.%02d C, P=%d.%03d kPa\n",
        temperature_value.val1,
        temperature_value.val2 / 10000,
        pressure_value.val1,
        pressure_value.val2 / 1000
    );

    /*
     * Convert pressure to 0.1 hPa units.
     *
     * sensor_value for pressure is in Pa.
     * 98555 Pa -> 9855 (0.1 hPa).
     */
    int32_t pressure_hpa_x10 =
        pressure_value.val1 * 10 +
        pressure_value.val2 / 100000;

    pressure = (zb_int16_t)pressure_hpa_x10;

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

    printk(
        "VDD: %d.%03d V\n",
        vdd_mv / 1000,
        vdd_mv % 1000
    );

    /*
     * Battery voltage is represented in 100 mV units.
     *
     * Example:
     * 3.2 V -> 32
     */
    battery_voltage = (zb_uint8_t)(vdd_mv / 100);

    return 0;
}

/*
 * Zigbee stack signal handler.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    zb_zdo_app_signal_hdr_t *sig_hndler = NULL;

    zb_zdo_app_signal_type_t sig =
        zb_get_app_signal(bufid, &sig_hndler);

    zb_ret_t status =
        ZB_GET_APP_SIGNAL_STATUS(bufid);

    printk(
        "Zigbee signal: %d, status: %d\n",
        sig,
        status
    );

    switch (sig) {
    case ZB_BDB_SIGNAL_STEERING:
        if (status == RET_OK) {
            printk("Zigbee joined network\n");
        } else {
            printk("Zigbee steering failed, retrying\n");

            bdb_start_top_level_commissioning(
                ZB_BDB_NETWORK_STEERING
            );
        }
        break;

    case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (status == RET_OK) {
            printk("Zigbee network restored\n");
        } else {
            zigbee_default_signal_handler(bufid);
        }
        break;

    default:
        zigbee_default_signal_handler(bufid);
        break;
    }

    if (bufid) {
        zb_buf_free(bufid);
    }
}

int main(void)
{
    const struct device *i2c =
        DEVICE_DT_GET(I2C_NODE);

    const struct device *bmp280 =
        DEVICE_DT_GET(BMP280_NODE);

    const struct device *adc =
        DEVICE_DT_GET(ADC_NODE);

    if (!device_is_ready(i2c)) {
        printk("ERROR: I2C device is not ready\n");
        return 0;
    }

    if (!device_is_ready(bmp280)) {
        printk("ERROR: BMP280 device is not ready\n");
        return 0;
    }

    if (!device_is_ready(adc)) {
        printk("ERROR: ADC device is not ready\n");
        return 0;
    }

    printk("Zigbee weather sensor starting\n");
    printk("BMP280 address: 0x77\n");
    printk("AHT20 address: 0x38\n");

    /*
     * Do not erase persistent Zigbee network configuration.
     */
    zigbee_erase_persistent_storage(
        ERASE_PERSISTENT_CONFIG
    );

    /*
     * Register our endpoint before starting Zigbee.
     */
    ZB_AF_REGISTER_DEVICE_CTX(
        &weather_device_ctx
    );

    /*
     * Start Nordic Zigbee application.
     */
    zigbee_enable();

    printk("Zigbee stack started\n");

    while (1) {
        read_vdd(adc);
        read_aht20(i2c);
        read_bmp280(bmp280);

        printk("\n");

        k_sleep(K_SECONDS(5));
    }

    return 0;
}