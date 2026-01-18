/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000


/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */

int main(void)
{
	int ret;
	printf("Does this work Rue?");

	// talk to expander
	const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if(i2c_dev == NULL || !device_is_ready(i2c_dev)) {
		printf("I2C peripheral not ready\n");
		return -1;
	}

	while(1) {
		ret = i2c_reg_write_byte(i2c_dev, 0x20, 0x7, (0x7 >> 8));
		if(ret != 0) {
			printf("Could not enable all 3 LEDs \n");
			return -1;
		}
		k_msleep(SLEEP_TIME_MS);

		ret = i2c_reg_write_byte(i2c_dev, 0x20, 0x0, (0x0 >> 8));
		if(ret != 0) {
			printf("Could not enable all 3 LEDs \n");
			return -1;
		}
	}

	return 0;
}
