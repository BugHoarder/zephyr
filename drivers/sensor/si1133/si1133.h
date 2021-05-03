#include <stdio.h>

#define SI1133_INIT_TIME_MS 		25

#define SI1133_VAL_PART_ID		0x33

#define SI1133_REG_COMMAND		0x0B
#define SI1133_REG_PART_ID		0x00
#define SI1133_REG_RESPONSE0		0x11

#define SI1133_CMD_RST_CMD_CTR		0x00

struct si1133_dev_config {
	const char *i2c_master_name;
	uint16_t i2c_addr;
};
