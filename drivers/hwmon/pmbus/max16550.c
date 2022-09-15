// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for Maxim MAX16550
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include "pmbus.h"

#define MAX16550_MFR_VOUT_PEAK		0xfd
#define MAX16550_MFR_TEMPERATURE_PEAK	0xd4

static int max16550_read_word_data(struct i2c_client *client, int page,
				   int phase, int reg)
{
	int ret;

	switch (reg) {
	case PMBUS_VIRT_READ_VOUT_MAX:
		ret = pmbus_read_word_data(client, page, phase,
					   MAX16550_MFR_VOUT_PEAK);
		break;
	case PMBUS_VIRT_READ_TEMP_MAX:
		ret = pmbus_read_word_data(client, page, phase,
					   MAX16550_MFR_TEMPERATURE_PEAK);
		break;
	case PMBUS_VIRT_RESET_VOUT_HISTORY:
	case PMBUS_VIRT_RESET_TEMP_HISTORY:
		ret = 0;
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static int max16550_write_word_data(struct i2c_client *client, int page,
				    int reg, u16 word)
{
	int ret;

	switch (reg) {
	case PMBUS_VIRT_RESET_VOUT_HISTORY:
		ret = pmbus_write_word_data(client, page,
					    MAX16550_MFR_VOUT_PEAK, 0);
		break;
	case PMBUS_VIRT_RESET_TEMP_HISTORY:
		ret = pmbus_write_word_data(client, page,
					    MAX16550_MFR_TEMPERATURE_PEAK,
					    0xffff);
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static struct pmbus_driver_info max16550_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.m[PSC_VOLTAGE_IN] = 7578,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = -2,

	.format[PSC_VOLTAGE_OUT] = direct,
	.m[PSC_VOLTAGE_OUT] = 7578,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = -2,

	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_TEMPERATURE] = 199,
	.b[PSC_TEMPERATURE] = 7046,
	.R[PSC_TEMPERATURE] = -2,

	.format[PSC_CURRENT_OUT] = direct,
	.m[PSC_POWER] = 34416,
	.b[PSC_POWER] = -4300,
	.R[PSC_POWER] = -3,

	.format[PSC_POWER] = direct,
	.m[PSC_POWER] = 13425,
	.b[PSC_POWER] = -9100,
	.R[PSC_POWER] = -2,

	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN |
			PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
			PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
			PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
			PMBUS_HAVE_PIN  | PMBUS_HAVE_STATUS_INPUT,
	.read_word_data = max16550_read_word_data,
	.write_word_data = max16550_write_word_data,
};

static int max16550_probe(struct i2c_client *client)
{
	return pmbus_do_probe(client, &max16550_info);
}

static const struct i2c_device_id max16550_id[] = {
	{"max16550", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max16550_id);

/* This is the driver that will be inserted */
static struct i2c_driver max16550_driver = {
	.driver = {
		   .name = "max16550",
		   },
	.probe_new = max16550_probe,
	.id_table = max16550_id,
};

module_i2c_driver(max16550_driver);

MODULE_AUTHOR("Brian Ma");
MODULE_DESCRIPTION("PMBus driver for Maxim MAX16550");
MODULE_LICENSE("GPL");
