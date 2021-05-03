#include <zephyr.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "si1133.h"

#define DT_DRV_COMPAT silabs_si1133

LOG_MODULE_REGISTER(SI1133, CONFIG_SENSOR_LOG_LEVEL);

struct si1133_dev_data {
	uint16_t i2c_addr;
	const struct device *i2c_master;
};

static inline int si1133_rst_cmd_ctr(const struct si1133_dev_data *const data)
{
	return i2c_reg_write_byte(data->i2c_master, data->i2c_addr, SI1133_REG_COMMAND, SI1133_CMD_RST_CMD_CTR);	
}

static int si1133_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	return 0;
}

static int si1133_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	return 0;
}

static int si1133_init(const struct device *dev)
{
	struct si1133_dev_data *data = dev->data;
	const struct si1133_dev_config *cfg = dev->config;
	int ret;
	uint8_t part_id, response0;

	data->i2c_addr = cfg->i2c_addr;
	data->i2c_master = device_get_binding(cfg->i2c_master_name);
	if (!data->i2c_master) {
		LOG_ERR("i2c master not found");
		return -ENODEV;
	}

	const struct device *portf = device_get_binding("GPIO_F");
	gpio_pin_configure(portf, 9, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(portf, 9, 1);

	/* ensure the sensor is ready for i2c transactions */
	k_msleep(25);

	ret = 1;
	while (ret) {
		ret = i2c_reg_read_byte(data->i2c_master, data->i2c_addr, SI1133_REG_PART_ID, &part_id);
		if (ret != 0)
			printk("read part id failed with error number: %d\n", ret);
	}

	printk("part id is: %x\n", part_id);

	ret = 1;
	while (ret) {
		ret = i2c_reg_read_byte(data->i2c_master, data->i2c_addr, SI1133_REG_RESPONSE0, &response0);
		if (ret != 0)
			printk("read response0 failed\n");
	}
	printk("response0 is: %x\n", response0 & 0b1111);

	si1133_rst_cmd_ctr(data);

	ret = 1;
        while (ret) {
                ret = i2c_reg_read_byte(data->i2c_master, data->i2c_addr, SI1133_REG_RESPONSE0, &response0);
                if (ret != 0)
                        printk("read response0 failed\n");
        }

	printk("response0 now is: %x\n", response0 & 0b1111);

	LOG_DBG("si1133 init ok");

	return 0;
}

static const struct si1133_dev_config si1133_config = {
	.i2c_master_name = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
};

static const struct sensor_driver_api si1133_api_funcs = {
	.sample_fetch = si1133_sample_fetch,
	.channel_get = si1133_channel_get,
};

static struct si1133_dev_data si1133_data;

DEVICE_DT_INST_DEFINE(0, si1133_init, device_pm_control_nop, &si1133_data, &si1133_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &si1133_api_funcs);
