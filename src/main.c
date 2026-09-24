#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/logging/log.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_nrf_platform.h>

#include <zcl/zb_zcl_common.h>
#include <zcl/zb_zcl_basic.h>
#include <zcl/zb_zcl_temp_measurement.h>
#include <zcl/zb_zcl_pressure_measurement.h>
#include <zcl/zb_zcl_rel_humidity_measurement.h>
#include <zcl/zb_zcl_power_config.h>
#include <zcl/zb_zcl_reporting.h>

#include <zigbee/zigbee_app_utils.h>

#include <string.h>

LOG_MODULE_REGISTER(weather, LOG_LEVEL_INF);

/* --------------------------------------------------------------------------
 * Hardware
 * -------------------------------------------------------------------------- */

#define I2C_NODE DT_NODELABEL(i2c0)
#define BMP280_NODE DT_NODELABEL(bmp280)
#define ADC_NODE DT_NODELABEL(adc)

#define AHT20_ADDR 0x38
#define AHT20_CMD_TRIGGER 0xAC

#define ADC_RESOLUTION 14

/* --------------------------------------------------------------------------
 * Zigbee
 * -------------------------------------------------------------------------- */

#define WEATHER_ENDPOINT 1
#define WEATHER_IN_CLUSTER_COUNT 5
#define WEATHER_OUT_CLUSTER_COUNT 0

/* --------------------------------------------------------------------------
 * Application
 * -------------------------------------------------------------------------- */

#define MEASUREMENT_INTERVAL_SECONDS 15

/* --------------------------------------------------------------------------
 * Battery
 * -------------------------------------------------------------------------- */

#define BATTERY_LOW_WARNING_MV 2700
#define BATTERY_LOW_WARNING_ZCL 27

struct battery_level_point {
	int32_t voltage_mv;
	uint8_t percentage;
};

static const struct battery_level_point battery_table[] = {
	{ 3400, 100 },
	{ 3350,  95 },
	{ 3330,  90 },
	{ 3310,  85 },
	{ 3290,  80 },
	{ 3270,  75 },
	{ 3250,  70 },
	{ 3230,  65 },
	{ 3210,  60 },
	{ 3190,  55 },
	{ 3170,  50 },
	{ 3150,  45 },
	{ 3130,  40 },
	{ 3110,  35 },
	{ 3090,  30 },
	{ 3070,  25 },
	{ 3050,  20 },
	{ 3000,  15 },
	{ 2900,  10 },
	{ 2800,   5 },
	{ 2700,   0 },
};

#define BATTERY_TABLE_SIZE \
	(sizeof(battery_table) / sizeof(battery_table[0]))

/* --------------------------------------------------------------------------
 * Persistent Zigbee configuration
 * -------------------------------------------------------------------------- */

#define ERASE_PERSISTENT_CONFIG ZB_FALSE

/* --------------------------------------------------------------------------
 * ZCL attributes
 * -------------------------------------------------------------------------- */

static zb_int16_t temperature;
static zb_int16_t pressure;
static zb_uint16_t humidity;

static zb_uint8_t battery_voltage;
static zb_uint8_t battery_percentage_remaining;

static zb_uint8_t battery_size = 0xFF;
static zb_uint8_t battery_quantity = 1;
static zb_uint8_t battery_rated_voltage = 33;
static zb_uint8_t battery_alarm_mask = 0;
static zb_uint8_t battery_voltage_min_threshold =
	BATTERY_LOW_WARNING_ZCL;

static zb_uint8_t battery_voltage_threshold1 = 0;
static zb_uint8_t battery_voltage_threshold2 = 0;
static zb_uint8_t battery_voltage_threshold3 = 0;
static zb_uint8_t battery_percentage_min_threshold = 0;
static zb_uint8_t battery_percentage_threshold1 = 0;
static zb_uint8_t battery_percentage_threshold2 = 0;
static zb_uint8_t battery_percentage_threshold3 = 0;
static zb_uint32_t battery_alarm_state = 0;

/* --------------------------------------------------------------------------
 * ZCL attribute lists
 * -------------------------------------------------------------------------- */

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

#define bat_num

ZB_ZCL_DECLARE_POWER_CONFIG_BATTERY_ATTRIB_LIST_EXT(
	power_config_attr_list,
	&battery_voltage,
	&battery_size,
	&battery_quantity,
	&battery_rated_voltage,
	&battery_alarm_mask,
	&battery_voltage_min_threshold,
	&battery_percentage_remaining,
	&battery_voltage_threshold1,
	&battery_voltage_threshold2,
	&battery_voltage_threshold3,
	&battery_percentage_min_threshold,
	&battery_percentage_threshold1,
	&battery_percentage_threshold2,
	&battery_percentage_threshold3,
	&battery_alarm_state
);

#undef bat_num

/* --------------------------------------------------------------------------
 * Cluster list
 * -------------------------------------------------------------------------- */

static zb_zcl_cluster_desc_t cluster_list[] = {
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_BASIC,
		ZB_ZCL_ARRAY_SIZE(
			basic_attr_list,
			zb_zcl_attr_t
		),
		basic_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),

	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
		ZB_ZCL_ARRAY_SIZE(
			temperature_attr_list,
			zb_zcl_attr_t
		),
		temperature_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),

	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
		ZB_ZCL_ARRAY_SIZE(
			pressure_attr_list,
			zb_zcl_attr_t
		),
		pressure_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),

	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
		ZB_ZCL_ARRAY_SIZE(
			humidity_attr_list,
			zb_zcl_attr_t
		),
		humidity_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	),

	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
		ZB_ZCL_ARRAY_SIZE(
			power_config_attr_list,
			zb_zcl_attr_t
		),
		power_config_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID
	)
};

/* --------------------------------------------------------------------------
 * Zigbee simple descriptor
 * -------------------------------------------------------------------------- */

typedef ZB_PACKED_PRE struct weather_simple_desc_s {
	zb_uint8_t endpoint;
	zb_uint16_t app_profile_id;
	zb_uint16_t app_device_id;
	zb_bitfield_t app_device_version : 4;
	zb_bitfield_t reserved : 4;
	zb_uint8_t app_input_cluster_count;
	zb_uint8_t app_output_cluster_count;
	zb_uint16_t app_cluster_list[
		WEATHER_IN_CLUSTER_COUNT +
		WEATHER_OUT_CLUSTER_COUNT
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

/* --------------------------------------------------------------------------
 * Zigbee reporting
 * -------------------------------------------------------------------------- */

#define WEATHER_REPORT_ATTR_COUNT 5

ZBOSS_DEVICE_DECLARE_REPORTING_CTX(
	weather_reporting_info,
	WEATHER_REPORT_ATTR_COUNT
);

/* --------------------------------------------------------------------------
 * Endpoint
 * -------------------------------------------------------------------------- */

ZB_AF_DECLARE_ENDPOINT_DESC(
	weather_endpoint,
	WEATHER_ENDPOINT,
	ZB_AF_HA_PROFILE_ID,
	0,
	NULL,
	ZB_ZCL_ARRAY_SIZE(
		cluster_list,
		zb_zcl_cluster_desc_t
	),
	cluster_list,
	(zb_af_simple_desc_1_1_t *)&simple_desc_weather,
	WEATHER_REPORT_ATTR_COUNT,
	weather_reporting_info,
	0,
	NULL
);

/* --------------------------------------------------------------------------
 * Device context
 * -------------------------------------------------------------------------- */

ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	weather_device_ctx,
	weather_endpoint
);

/* --------------------------------------------------------------------------
 * Battery percentage
 * -------------------------------------------------------------------------- */

static uint8_t battery_voltage_to_percentage(int32_t voltage_mv)
{
	size_t i;

	if (voltage_mv >= battery_table[0].voltage_mv) {
		return battery_table[0].percentage;
	}

	for (i = 0; i < BATTERY_TABLE_SIZE - 1; i++) {
		int32_t high_voltage = battery_table[i].voltage_mv;
		int32_t low_voltage = battery_table[i + 1].voltage_mv;

		uint8_t high_percentage = battery_table[i].percentage;
		uint8_t low_percentage = battery_table[i + 1].percentage;

		if (voltage_mv >= low_voltage) {
			int32_t voltage_range =
				high_voltage - low_voltage;

			int32_t percentage_range =
				high_percentage - low_percentage;

			int32_t percentage =
				low_percentage +
				((voltage_mv - low_voltage) *
				 percentage_range) /
				voltage_range;

			return (uint8_t)percentage;
		}
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * AHT20
 * -------------------------------------------------------------------------- */

static int read_aht20(const struct device *i2c)
{
	uint8_t command[3] = {
		AHT20_CMD_TRIGGER,
		0x33,
		0x00
	};

	uint8_t raw[7];

	int ret = i2c_write(
		i2c,
		command,
		sizeof(command),
		AHT20_ADDR
	);

	if (ret) {
		LOG_ERR("AHT20 write failed: %d", ret);
		return ret;
	}

	k_msleep(120);

	ret = i2c_read(
		i2c,
		raw,
		sizeof(raw),
		AHT20_ADDR
	);

	if (ret) {
		LOG_ERR("AHT20 read failed: %d", ret);
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
		((uint64_t)raw_humidity * 10000ULL) /
		1048576ULL;

	int32_t temperature_x100 =
		((int64_t)raw_temperature * 20000LL) /
		1048576LL -
		5000LL;

	temperature = (zb_int16_t)temperature_x100;
	humidity = (zb_uint16_t)humidity_x100;

	return 0;
}

/* --------------------------------------------------------------------------
 * BMP280
 * -------------------------------------------------------------------------- */

static int read_bmp280(const struct device *bmp280)
{
	struct sensor_value temperature_value;
	struct sensor_value pressure_value;

	int ret = sensor_sample_fetch(bmp280);

	if (ret) {
		LOG_ERR("BMP280 sample fetch failed: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(
		bmp280,
		SENSOR_CHAN_AMBIENT_TEMP,
		&temperature_value
	);

	if (ret) {
		LOG_ERR("BMP280 temperature read failed: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(
		bmp280,
		SENSOR_CHAN_PRESS,
		&pressure_value
	);

	if (ret) {
		LOG_ERR("BMP280 pressure read failed: %d", ret);
		return ret;
	}

	int64_t pressure_pa =
		(int64_t)pressure_value.val1 +
		pressure_value.val2 / 1000000;

	pressure = (zb_int16_t)(pressure_pa / 100);

	return 0;
}

/* --------------------------------------------------------------------------
 * VDD
 * -------------------------------------------------------------------------- */

static int read_vdd(const struct device *adc, int32_t *vdd_mv)
{
	struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1_6,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = 0,
		.input_positive = NRF_SAADC_VDD,
	};

	int ret = adc_channel_setup(
		adc,
		&channel_cfg
	);

	if (ret) {
		LOG_ERR("ADC channel setup failed: %d", ret);
		return ret;
	}

	int16_t sample = 0;

	struct adc_sequence sequence = {
		.channels = BIT(channel_cfg.channel_id),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = ADC_RESOLUTION,
	};

	ret = adc_read(
		adc,
		&sequence
	);

	if (ret) {
		LOG_ERR("ADC read failed: %d", ret);
		return ret;
	}

	*vdd_mv =
		((int32_t)sample * 3600) /
		(1 << ADC_RESOLUTION);

	battery_voltage =
		(zb_uint8_t)(*vdd_mv / 100);

	uint8_t percentage =
		battery_voltage_to_percentage(*vdd_mv);

	battery_percentage_remaining =
		(zb_uint8_t)(percentage * 2);

	if (*vdd_mv < BATTERY_LOW_WARNING_MV) {
		LOG_WRN(
			"Low battery voltage: %d.%03d V",
			*vdd_mv / 1000,
			*vdd_mv % 1000
		);
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * Configure Zigbee reporting
 * -------------------------------------------------------------------------- */

static void configure_reporting(void)
{
	zb_zcl_reporting_info_t reporting;

	/* Temperature */

	memset(&reporting, 0, sizeof(reporting));

	reporting.direction =
		ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting.ep = WEATHER_ENDPOINT;
	reporting.cluster_id =
		ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
	reporting.cluster_role =
		ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting.attr_id =
		ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
	reporting.dst.short_addr = 0x0000;
	reporting.dst.endpoint = 1;
	reporting.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting.u.send_info.min_interval = 0;
	reporting.u.send_info.max_interval =
		MEASUREMENT_INTERVAL_SECONDS;
	reporting.u.send_info.delta.u16 = 1;

	zb_zcl_put_reporting_info(
		&reporting,
		ZB_TRUE
	);

	/* Pressure */

	memset(&reporting, 0, sizeof(reporting));

	reporting.direction =
		ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting.ep = WEATHER_ENDPOINT;
	reporting.cluster_id =
		ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
	reporting.cluster_role =
		ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting.attr_id =
		ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID;
	reporting.dst.short_addr = 0x0000;
	reporting.dst.endpoint = 1;
	reporting.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting.u.send_info.min_interval = 0;
	reporting.u.send_info.max_interval =
		MEASUREMENT_INTERVAL_SECONDS;
	reporting.u.send_info.delta.u16 = 1;

	zb_zcl_put_reporting_info(
		&reporting,
		ZB_TRUE
	);

	/* Relative humidity */

	memset(&reporting, 0, sizeof(reporting));

	reporting.direction =
		ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting.ep = WEATHER_ENDPOINT;
	reporting.cluster_id =
		ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
	reporting.cluster_role =
		ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting.attr_id =
		ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
	reporting.dst.short_addr = 0x0000;
	reporting.dst.endpoint = 1;
	reporting.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting.u.send_info.min_interval = 0;
	reporting.u.send_info.max_interval =
		MEASUREMENT_INTERVAL_SECONDS;
	reporting.u.send_info.delta.u16 = 1;

	zb_zcl_put_reporting_info(
		&reporting,
		ZB_TRUE
	);

	/* Battery voltage */

	memset(&reporting, 0, sizeof(reporting));

	reporting.direction =
		ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting.ep = WEATHER_ENDPOINT;
	reporting.cluster_id =
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
	reporting.cluster_role =
		ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting.attr_id =
		ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID;
	reporting.dst.short_addr = 0x0000;
	reporting.dst.endpoint = 1;
	reporting.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting.u.send_info.min_interval = 0;
	reporting.u.send_info.max_interval =
		MEASUREMENT_INTERVAL_SECONDS;
	reporting.u.send_info.delta.u8 = 1;

	zb_zcl_put_reporting_info(
		&reporting,
		ZB_TRUE
	);

	/* Battery percentage */

	memset(&reporting, 0, sizeof(reporting));

	reporting.direction =
		ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	reporting.ep = WEATHER_ENDPOINT;
	reporting.cluster_id =
		ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
	reporting.cluster_role =
		ZB_ZCL_CLUSTER_SERVER_ROLE;
	reporting.attr_id =
		ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
	reporting.dst.short_addr = 0x0000;
	reporting.dst.endpoint = 1;
	reporting.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	reporting.u.send_info.min_interval = 0;
	reporting.u.send_info.max_interval =
		MEASUREMENT_INTERVAL_SECONDS;
	reporting.u.send_info.delta.u8 = 10;

	zb_zcl_put_reporting_info(
		&reporting,
		ZB_TRUE
	);

	LOG_INF(
		"Reporting configured: interval=%d s",
		MEASUREMENT_INTERVAL_SECONDS
	);
}

/* --------------------------------------------------------------------------
 * Zigbee signal handler
 * -------------------------------------------------------------------------- */

void zboss_signal_handler(zb_uint8_t param)
{
	zb_zdo_app_signal_hdr_t *sig_hndler = NULL;

	zb_zdo_app_signal_type_t sig =
		zb_get_app_signal(
			param,
			&sig_hndler
		);

	zb_ret_t status =
		ZB_GET_APP_SIGNAL_STATUS(param);

	switch (sig) {
	case ZB_BDB_SIGNAL_STEERING:

		if (status == RET_OK) {
			LOG_INF("Joined Zigbee network");
			configure_reporting();
		} else {
			LOG_WRN(
				"Network steering failed: %d",
				status
			);

			bdb_start_top_level_commissioning(
				ZB_BDB_NETWORK_STEERING
			);
		}

		break;

	case ZB_BDB_SIGNAL_DEVICE_REBOOT:

		if (status == RET_OK) {
			LOG_INF("Zigbee network restored");
			configure_reporting();
		} else {
			LOG_WRN(
				"Zigbee network restore failed: %d",
				status
			);

			zigbee_default_signal_handler(
				param
			);
		}

		break;

	default:

		zigbee_default_signal_handler(
			param
		);

		break;
	}
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(void)
{
	const struct device *i2c =
		DEVICE_DT_GET(I2C_NODE);

	const struct device *bmp280 =
		DEVICE_DT_GET(BMP280_NODE);

	const struct device *adc =
		DEVICE_DT_GET(ADC_NODE);

	if (!device_is_ready(i2c)) {
		LOG_ERR("I2C device is not ready");
		return 0;
	}

	if (!device_is_ready(bmp280)) {
		LOG_ERR("BMP280 device is not ready");
		return 0;
	}

	if (!device_is_ready(adc)) {
		LOG_ERR("ADC device is not ready");
		return 0;
	}

	LOG_INF("Zigbee weather sensor started");

	LOG_INF(
		"Measurement interval: %d s",
		MEASUREMENT_INTERVAL_SECONDS
	);

	LOG_INF(
		"Sensors: AHT20@0x38 BMP280@0x77"
	);

	LOG_INF(
		"Battery warning threshold: %d.%03d V",
		BATTERY_LOW_WARNING_MV / 1000,
		BATTERY_LOW_WARNING_MV % 1000
	);

	zigbee_erase_persistent_storage(
		ERASE_PERSISTENT_CONFIG
	);

	ZB_AF_REGISTER_DEVICE_CTX(
		&weather_device_ctx
	);

	zigbee_enable();

	LOG_INF("Zigbee stack started");

	while (1) {
		int32_t vdd_mv = 0;

		int vdd_ret;
		int aht20_ret;
		int bmp280_ret;

		vdd_ret = read_vdd(
			adc,
			&vdd_mv
		);

		aht20_ret = read_aht20(
			i2c
		);

		bmp280_ret = read_bmp280(
			bmp280
		);

		if (vdd_ret == 0 &&
		    aht20_ret == 0 &&
		    bmp280_ret == 0) {

			LOG_INF(
				"T=%d.%02d C RH=%u.%02u %% "
				"P=%d.%01d kPa "
				"VDD=%d.%03d V "
				"Battery=%u%%",
				temperature / 100,
				temperature % 100,
				humidity / 100,
				humidity % 100,
				pressure / 10,
				pressure % 10,
				vdd_mv / 1000,
				vdd_mv % 1000,
				battery_percentage_remaining / 2
			);

		} else {
			LOG_ERR(
				"Measurement failed: "
				"VDD=%d AHT20=%d BMP280=%d",
				vdd_ret,
				aht20_ret,
				bmp280_ret
			);
		}

		k_sleep(
			K_SECONDS(
				MEASUREMENT_INTERVAL_SECONDS
			)
		);
	}

	return 0;
}