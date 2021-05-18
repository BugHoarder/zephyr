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
	uint16_t lux;
};

static inline int si1133_reg_cmd_write(const struct si1133_dev_data *const data, uint8_t add, uint8_t cmd)
{
	return i2c_reg_write_byte(data->i2c_master, data->i2c_addr, add, cmd);	
}

static inline int si1133_reg_read(const struct si1133_dev_data *const data, uint8_t add, uint8_t* output_value)
{
	return i2c_reg_read_byte(data->i2c_master, data->i2c_addr, add, output_value);		
}


static inline int si1133_rst_cmd_ctr(const struct si1133_dev_data *const data)
{
	return si1133_reg_cmd_write(data, SI1133_REG_COMMAND, SI1133_CMD_RST_CMD_CTR);	
}

static inline int si1133_rst_and_read_cmd_ctr(const struct si1133_dev_data *const data, uint8_t* cmd_ctr)
{
	 uint8_t cnt;
	 int ret;
	ret = si1133_rst_cmd_ctr(data);
	if (ret < 0) {
		return ret;
	}

	si1133_reg_read(data, SI1133_REG_RESPONSE0, &cnt);
	*cmd_ctr = (cnt & SI1133_CMD_CTR);
	return 0;
}

static inline int si1133_poll_till_ctr_increment(const struct si1133_dev_data *const data, uint8_t cmd_ctr)
{
	uint8_t ctr;
	int ret;
	while(true){
		ret =  si1133_reg_read(data, SI1133_REG_RESPONSE0, &ctr);
		if (ret)
			return ret;
		if(ctr & SI1133_CMD_ERR)
			return -EINVAL;
		if((ctr & SI1133_CMD_CTR) == cmd_ctr+1)
			break;
	}
	return 0;
}


static inline int si1133_param_set(const struct si1133_dev_data *const data, uint8_t add, uint8_t param_data )
{
	int ret;
	ret = si1133_reg_cmd_write(data, SI1133_REG_HOSTIN0, param_data);
	if (ret < 0) {
		printk("param write failed with rc=%d\n", ret);
	}
	uint8_t set_cmd = 0b10000000;
	uint8_t set_add = set_cmd | add;
	return si1133_reg_cmd_write(data, SI1133_REG_COMMAND, set_add);	
}

static int si1133_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct si1133_dev_data *data = dev->data;
	uint8_t status;
	uint8_t large_white[2];
	uint8_t cmd_ctr;
	int ret;
	//reset response0 and read cmd_ctr
	ret = si1133_rst_and_read_cmd_ctr(data, &cmd_ctr);
	if (ret) {
		printk("si1133_rst_and_read_cmd_ctr failed with rc=%d\n", ret);
	}

	//force command
	ret = si1133_reg_cmd_write(data, SI1133_REG_COMMAND, SI1133_CMD_FORCE);
	if (ret < 0) {
		printk("force cmd failed with rc=%d\n", ret);
	}

	//poll till counter increments
	ret = si1133_poll_till_ctr_increment(data, cmd_ctr);
	if (ret) {
		printk("si1133_poll_till_ctr_increment failed with rc=%d\n", ret);
	}

	while(!status){
		i2c_reg_read_byte(data->i2c_master, data->i2c_addr, SI1133_REG_IRQ_STATUS, &status);
	}
	
	ret = i2c_burst_read(data->i2c_master, data->i2c_addr, SI1133_REG_HOSTOUT0, large_white, 2);
	if (ret < 0) {
		printk("Samples read failed with rc=%d\n", ret);
	}

	data->lux =  ((large_white[0] << 8) | large_white[1]);
	printk("29 Sensor data is =%d\n", data->lux);

	return 0;
}
static int si1133_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{	
	struct si1133_dev_data *data = dev->data;
	val->val1 = data->lux;
	val->val2 = 0;
	return 0;
}

static int si1133_init(const struct device *dev)
{
	struct si1133_dev_data *data = dev->data;
	const struct si1133_dev_config *cfg = dev->config;
	int ret;
	//uint8_t part_id, response0, response1;

	data->i2c_addr = cfg->i2c_addr;
	data->i2c_master = device_get_binding(cfg->i2c_master_name);
	
	if (!data->i2c_master) {
		LOG_ERR("i2c master not found");
		return -ENODEV;
	}

	k_msleep(25);

//	power in board.c file
	const struct device *portf = device_get_binding("GPIO_F");
	gpio_pin_configure(portf, 9, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(portf, 9, 1);


	//channel setup
	//channel 0 is 0b00000001 at 0x01
	ret = si1133_param_set(data, SI1133_REG_CHANNEL_LIST, SI1133_CMD_ENABLE_CHANNEL0);
	if (ret != 0){
		printk("channel 0 param write failed with error number: %d\n", ret);
	}

	k_msleep(10);
	
	//channel 0 config
	//ADCCONFIG0 = 0b00001101
	ret = si1133_param_set(data, SI1133_REG_ADCCONFIG0, SI1133_CMD_ENABLE_LARGE_WHITE);
	if (ret != 0){
		printk("ADCCONFIG0 param set failed with error number: %d\n", ret);
	}
	k_msleep(10);

	
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
