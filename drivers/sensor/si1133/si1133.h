
#ifndef ZEPHYR_DRIVERS_SENSOR_SI1133_SI1133_H_
#define ZEPHYR_DRIVERS_SENSOR_SI1133_SI1133_H_

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <stdio.h>


#define SI1133_INIT_TIME_MS 		25

#define SI1133_VAL_PART_ID		0x33

#define SI1133_REG_COMMAND		0x0B
#define SI1133_REG_PART_ID		0x00
#define SI1133_REG_REV_ID		0x02
#define SI1133_REG_HW_ID		0x01

#define SI1133_REG_RESPONSE0		0x11
#define SI1133_REG_RESPONSE1		0x10
#define SI1133_REG_HOSTIN0			0x0A
#define SI1133_REG_HOSTOUT0			0x13
#define SI1133_REG_HOSTOUT1			0x14
#define SI1133_REG_HOSTOUT2			0x15

#define SI1133_REG_IRQ_STATUS		0x12
#define SI1133_REG_IRQ_ENABLE		0x0F


#define SI1133_REG_CHANNEL_LIST		0x01
#define SI1133_REG_ADCCONFIG0		0x02
#define SI1133_REG_ADCSENS0			0x03
#define SI1133_REG_ADCPOST0			0x04
#define SI1133_REG_MEASCONFIG0		0x05

#define SI1133_CMD_RST_CMD_CTR		0x00
#define SI1133_CMD_FORCE		0x11
#define SI1133_CMD_ENABLE_CHANNEL0		0x01
#define SI1133_CMD_ENABLE_LARGE_WHITE		0x0D
#define SI1133_CMD_CTR 		0x0F
#define SI1133_CMD_ERR 		0x10







struct si1133_dev_config {
	const char *i2c_master_name;
	uint16_t i2c_addr;
};


#endif