/*
 * Copyright (c) 2025, gaiaochos.com
 * SPDX-License-Identifier: Apache 2.0
 *
 */

#define DT_DRV_COMPAT st_lis3dshtr

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LIS3DSHTR, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>


struct st_lis3dshtr_config {
	const struct spi_dt_spec spi_controller;
};

struct st_lis3dshtr_data {
	uint8_t sample_int;
	uint8_t sample_dec;
};

static int lis3dshtr_write_register(const struct device *dev, uint8_t reg, uint8_t value) {
	const struct st_lis3dshtr_config *cfg = dev->config;

	uint8_t tx_buf[2] = {reg,value};

	const struct spi_buf buf = {
		.buf = tx_buf,
		.len = sizeof(tx_buf)
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};


	return spi_write_dt(&cfg->spi_controller, &tx);
}

static int lis3dshtr_read_register(const struct device *dev, uint8_t reg, uint8_t *val)  {
	const struct st_lis3dshtr_config *cfg = dev->config;
	int ret;
	uint8_t tx_buf[1];
	uint8_t rx_buf[1];

	/*Write address to read*/
	tx_buf[0] = 0x80 | reg;

	const struct spi_buf tx_buf_arr = {
		.buf = tx_buf,
		.len = 1,
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf_arr,
		.count = 1,
	};
	const struct spi_buf rx_buf_arr = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf_arr,
		.count = 1,
	};

	ret = spi_transceive_dt(&cfg->spi_controller, &tx_set, &rx_set);

	if (ret != 0) {
		LOG_ERR("Error reading channel axis (%02X)", reg);
		return ret;
	}

	return  ret;
}

static int lis3dshtr_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct st_lis3dshtr_data *data = dev->data;
	int ret;

	if (chan != SENSOR_CHAN_ACCEL_X) {
		return -ENOTSUP;
	}

	ret = lis3dshtr_read_register(dev, 0x29, &data->sample_int);

	if (ret != 0) {
		LOG_ERR("Could not read channel HIGH byte");
		return ret;
	}

	ret = lis3dshtr_read_register(dev, 0x28, &data->sample_int);

	if (ret != 0) {
		LOG_ERR("Could not read channe LOW byte");
		return ret;
	}

	return ret;
}

static int lis3dshtr_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct st_lis3dshtr_data *data = dev->data;
	int16_t raw;
	int32_t micro_g;

	if(chan != SENSOR_CHAN_ACCEL_X) {
		return -ENOTSUP;
	}

	/*Combine high and low byte*/
	raw = (int16_t)((data->sample_int << 8) | data->sample_dec);

	/*Apply 2G scaling*/
	micro_g = raw * 60;

	/*Fill sensor val with integer and fractional parts*/
	val->val1 = micro_g/1000000;
	val->val2 = micro_g%1000000;

	return 0;
}


static DEVICE_API(sensor, st_lis3dshtr_api) = {
	.sample_fetch = lis3dshtr_sample_fetch,
	.channel_get = lis3dshtr_channel_get,
};

static int st_lis3dshtr_init(const struct device *dev)
{
	const struct st_lis3dshtr_config *config = dev->config;
	int ret = 0;
	uint8_t config_byte;

	if(!spi_is_ready_dt(&config->spi_controller)) {
		LOG_ERR("SPI Bus not ready");
		return -ENODEV;
	}

	/*Static bring up*/
	config_byte = 0x60 | 0x0 | 0x7;

	ret = lis3dshtr_write_register(dev, 0x20, config_byte);

	if (ret != 0) {
		LOG_ERR("Could not configure LIS3DSHTR with (%d)", ret);
		return ret;
	}

	return ret;
}

#define SENSOR_LIS3DSHTR_DEFINE(inst)							\
	static struct st_lis3dshtr_data lis3dshtr_data_##inst;				\
	static struct st_lis3dshtr_config lis3dshtr_config_##inst = {			\
		.spi_controller  = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),	\
	};										\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,						\
				     &st_lis3dshtr_init,				\
				     NULL,						\
				     &lis3dshtr_data_##inst,				\
				     &lis3dshtr_config_##inst,				\
				     POST_KERNEL,					\
				     CONFIG_SENSOR_INIT_PRIORITY,			\
				     &st_lis3dshtr_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_LIS3DSHTR_DEFINE)
