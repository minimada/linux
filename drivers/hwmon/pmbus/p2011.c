// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for P2011 PSU
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/pmbus.h>
#include "pmbus.h"

static int p2011_read_byte_data(struct i2c_client *client, int page, int reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int p2011_write_byte(struct i2c_client *client, int page, u8 byte)
{
	return i2c_smbus_write_byte(client, byte);
}

static int p2011_read_word_data(struct i2c_client *client, int page,
				   int phase, int reg)
{
	int ret;

	switch (reg) {
	case PMBUS_FAN_COMMAND_4:
		return -ENOENT;
	case PMBUS_VIRT_FAN_TARGET_3:
		reg += PMBUS_FAN_COMMAND_3 - PMBUS_FAN_COMMAND_2 - 1;
	case PMBUS_VIRT_FAN_TARGET_1:
	case PMBUS_VIRT_FAN_TARGET_2:
		reg = (reg - PMBUS_VIRT_FAN_TARGET_1) + PMBUS_FAN_COMMAND_1;
	default:
		ret = i2c_smbus_read_word_data(client, reg);
		break;
	}

	return ret;
}

static int p2011_write_word_data(struct i2c_client *client, int page,
				    int reg, u16 word)
{
	int ret;

	switch (reg) {
	case PMBUS_VIRT_FAN_TARGET_3:
		reg += PMBUS_FAN_COMMAND_3 - PMBUS_FAN_COMMAND_2 - 1;
	case PMBUS_VIRT_FAN_TARGET_1:
	case PMBUS_VIRT_FAN_TARGET_2:
		reg = (reg - PMBUS_VIRT_FAN_TARGET_1) + PMBUS_FAN_COMMAND_1;
	default:
		ret = i2c_smbus_write_word_data(client, reg, word);
		break;
	}

	return ret;
}

static struct pmbus_driver_info p2011_info = {
	.pages = 1,

	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 0,
	.R[PSC_TEMPERATURE] = 0,

	.format[PSC_FAN] = direct,
	.m[PSC_FAN] = 1,
	.b[PSC_FAN] = 0,
	.R[PSC_FAN] = 0,

	.format[PSC_PWM] = direct,
	.m[PSC_PWM] = 1,
	.b[PSC_PWM] = 0,
	.R[PSC_PWM] = 2,

	.func[0] =	PMBUS_HAVE_FAN12 | PMBUS_HAVE_FAN34 |
			PMBUS_HAVE_TEMP | PMBUS_HAVE_TEMP2 | PMBUS_HAVE_TEMP3,
	.read_byte_data = p2011_read_byte_data,
	.write_byte = p2011_write_byte,
	.read_word_data = p2011_read_word_data,
	.write_word_data = p2011_write_word_data,
};

static int p2011_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pmbus_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct pmbus_platform_data),
				     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->flags = PMBUS_NO_WRITE_PROTECT | PMBUS_SKIP_STATUS_CHECK;
	dev->platform_data = pdata;

	return pmbus_do_probe(client, &p2011_info);
}

static const struct i2c_device_id p2011_id[] = {
	{"p2011", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, p2011_id);

static struct i2c_driver p2011_driver = {
	.driver = {
		   .name = "p2011",
		   },
	.probe_new = p2011_probe,
	.id_table = p2011_id,
};

module_i2c_driver(p2011_driver);

MODULE_DESCRIPTION("PMBus driver for P2011");
MODULE_LICENSE("GPL");
