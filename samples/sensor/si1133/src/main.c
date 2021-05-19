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

		struct sensor_value white, uv, ir;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &white);
		sensor_channel_get(dev, SENSOR_CHAN_IR, &ir);
		sensor_channel_get(dev, SENSOR_CHAN_UV, &uv);
	
		printk("large white : %u\n",white.val1);
		printk("Large IR: %u\n",ir.val1);
		printk("UV: %u\n",uv.val1);
		
	
		k_msleep(1000);
	}
}
