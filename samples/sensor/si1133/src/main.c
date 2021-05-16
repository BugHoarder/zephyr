/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <sys/printk.h>
//#include <drivers/sensor/si1133.h>
#include <stdio.h>




void main(void)
{
	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, silabs_si1133)));
	//printk("hello\n");

	if (dev == NULL) {
		return;
	}
	//printk("waiting\n");

	k_msleep(1000);
	
	while (1) {
		//printk("hello2\n");

		struct sensor_value temp;

		sensor_sample_fetch(dev);
		//printk("hello3\n");

		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		//printk("hello4\n");

		//printk("temp: %u.%06d\n",temp.val1, temp.val2);
		//printk("hello5\n");

		k_msleep(1000);
	}
}
