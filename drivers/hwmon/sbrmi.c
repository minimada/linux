// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sbrmi.c - hwmon driver for a SB-RMI mailbox
 *           compliant AMD SoC device.
 *
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include "sbrmi-common.h"

#define SOCK_0_ADDR	0x3C
#define SOCK_1_ADDR	0x38

/* Do not allow setting negative power limit */
#define SBRMI_PWR_MIN	0

/* SBRMI REVISION REG */
#define SBRMI_REV	0x0

enum sbrmi_msg_id {
	SBRMI_READ_PKG_PWR_CONSUMPTION = 0x1,
	SBRMI_WRITE_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_PWR_LIMIT,
	SBRMI_READ_PKG_MAX_PWR_LIMIT,
};

static int sbrmi_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };
	int ret;

	if (type != hwmon_power)
		return -EINVAL;

	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;

	switch (attr) {
	case hwmon_power_input:
		msg.cmd = SBRMI_READ_PKG_PWR_CONSUMPTION;
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap:
		msg.cmd = SBRMI_READ_PKG_PWR_LIMIT;
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		break;
	case hwmon_power_cap_max:
		msg.data_out.mb_out[RD_WR_DATA_INDEX] = rmi_dev->pwr_limit_max;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}
	if (ret < 0)
		return ret;
	/* hwmon power attributes are in microWatt */
	*val = (long)msg.data_out.mb_out[RD_WR_DATA_INDEX] * 1000;

	return ret;
}

static int sbrmi_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(dev);
	struct apml_message msg = { 0 };

	if (type != hwmon_power && attr != hwmon_power_cap)
		return -EINVAL;
	/*
	 * hwmon power attributes are in microWatt
	 * mailbox read/write is in mWatt
	 */
	val /= 1000;

	val = clamp_val(val, SBRMI_PWR_MIN, rmi_dev->pwr_limit_max);

	msg.cmd = SBRMI_WRITE_PKG_PWR_LIMIT;
	msg.data_in.mb_in[RD_WR_DATA_INDEX] = val;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 0;

	return rmi_mailbox_xfer(rmi_dev, &msg);
}

static umode_t sbrmi_is_visible(const void *data,
				enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_cap_max:
			return 0444;
		case hwmon_power_cap:
			return 0644;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_channel_info *sbrmi_info[] = {
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_CAP | HWMON_P_CAP_MAX),
	NULL
};

static const struct hwmon_ops sbrmi_hwmon_ops = {
	.is_visible = sbrmi_is_visible,
	.read = sbrmi_read,
	.write = sbrmi_write,
};

static const struct hwmon_chip_info sbrmi_chip_info = {
	.ops = &sbrmi_hwmon_ops,
	.info = sbrmi_info,
};

static int sbrmi_get_max_pwr_limit(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret;

	msg.cmd = SBRMI_READ_PKG_MAX_PWR_LIMIT;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = rmi_mailbox_xfer(rmi_dev, &msg);
	if (ret < 0)
		return ret;
	rmi_dev->pwr_limit_max = msg.data_out.mb_out[RD_WR_DATA_INDEX];

	return ret;
}

static int sbrmi_get_rev(struct apml_sbrmi_device *rmi_dev)
{
	struct apml_message msg = { 0 };
	int ret;

	msg.data_in.reg_in[REG_OFF_INDEX] = SBRMI_REV;
	msg.data_in.reg_in[RD_FLAG_INDEX] = 1;
	ret = regmap_read(rmi_dev->regmap,
			  msg.data_in.reg_in[REG_OFF_INDEX],
			  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
	if (ret < 0)
		return ret;

	rmi_dev->rev = msg.data_out.reg_out[RD_WR_DATA_INDEX];
	return 0;
}

static long sbrmi_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int __user *arguser = (int  __user *)arg;
	struct apml_message msg = { 0 };
	struct apml_sbrmi_device *rmi_dev;
	bool read = false;
	int ret = -EFAULT;

	if (copy_struct_from_user(&msg, sizeof(msg), arguser,
				  sizeof(struct apml_message)))
		return ret;

	rmi_dev = container_of(fp->private_data, struct apml_sbrmi_device,
			       sbrmi_misc_dev);
	if (!rmi_dev)
		return ret;

	/* Is this a read/monitor/get request */
	if (msg.data_in.reg_in[RD_FLAG_INDEX])
		read = true;

	switch (msg.cmd) {
	case 0 ... 0x999:
		ret = rmi_mailbox_xfer(rmi_dev, &msg);
		if (ret && ret != -EPROTOTYPE)
			return ret;
		break;
	case APML_CPUID:
		ret = apml_ops.rmi_cpuid_read(rmi_dev, &msg);
		if (ret && ret != -EPROTOTYPE)
			return ret;
		break;
	case APML_MCA_MSR:
		ret = apml_ops.rmi_mca_msr_read(rmi_dev, &msg);
		if (ret && ret != -EPROTOTYPE)
			return ret;
		break;
	case APML_REG:
		if (read) {
			ret = regmap_read(rmi_dev->regmap,
					  msg.data_in.reg_in[REG_OFF_INDEX],
					  &msg.data_out.mb_out[RD_WR_DATA_INDEX]);
			if (ret)
				return ret;
		} else {
			return regmap_write(rmi_dev->regmap,
					    msg.data_in.reg_in[REG_OFF_INDEX],
					    msg.data_in.reg_in[REG_VAL_INDEX]);
		}
		break;
	default:
		return ret;
	}

	/* Copy results back to user only for get/monitor commands and firmware failures */
	if (read || ret == -EPROTOTYPE) {
		if (copy_to_user(arguser, &msg, sizeof(struct apml_message)))
			return -EFAULT;
	}
	return ret;
}

static const struct file_operations sbrmi_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= sbrmi_ioctl,
	.compat_ioctl	= sbrmi_ioctl,
};

static int create_misc_rmi_device(struct apml_sbrmi_device *rmi_dev,
				  struct device *dev, int id)
{
	int ret;

	rmi_dev->sbrmi_misc_dev.name		= devm_kasprintf(dev, GFP_KERNEL, "apml_rmi%d", id);
	rmi_dev->sbrmi_misc_dev.minor		= MISC_DYNAMIC_MINOR;
	rmi_dev->sbrmi_misc_dev.fops		= &sbrmi_fops;
	rmi_dev->sbrmi_misc_dev.parent		= dev;
	rmi_dev->sbrmi_misc_dev.nodename	= devm_kasprintf(dev, GFP_KERNEL, "sbrmi%d", id);
	rmi_dev->sbrmi_misc_dev.mode		= 0600;

	ret = misc_register(&rmi_dev->sbrmi_misc_dev);
	if (ret)
		return ret;

	dev_info(dev, "register %s device\n", rmi_dev->sbrmi_misc_dev.name);
	return ret;
}

static int sbrmi_probe(struct i2c_client *client,
		       const struct i2c_device_id *rmi_id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct apml_sbrmi_device *rmi_dev;
	struct regmap_config sbrmi_i2c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	int id, ret;

	rmi_dev = devm_kzalloc(dev, sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;

	mutex_init(&rmi_dev->lock);
	rmi_dev->regmap = devm_regmap_init_i2c(client, &sbrmi_i2c_regmap_config);
	if (IS_ERR(rmi_dev->regmap))
		return PTR_ERR(rmi_dev->regmap);

	dev_set_drvdata(dev, (void *)rmi_dev);

	ret = sbrmi_get_rev(rmi_dev);
	if (ret < 0)
		return ret;

	rmi_set_apml_ops(rmi_dev->rev);

	/* Cache maximum power limit */
	ret = sbrmi_get_max_pwr_limit(rmi_dev);
	if (ret < 0)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 rmi_dev,
							 &sbrmi_chip_info,
							 NULL);

	if (!hwmon_dev)
		return PTR_ERR_OR_ZERO(hwmon_dev);

	if (client->addr == SOCK_0_ADDR)
		id = 0;
	if (client->addr == SOCK_1_ADDR)
		id = 1;

	return create_misc_rmi_device(rmi_dev, dev, id);
}

static void sbrmi_i2c_remove(struct i2c_client *client)
{
	struct apml_sbrmi_device *rmi_dev = dev_get_drvdata(&client->dev);

	if (rmi_dev)
		misc_deregister(&rmi_dev->sbrmi_misc_dev);

	dev_info(&client->dev, "Removed sbrmi driver\n");
}


static const struct i2c_device_id sbrmi_id[] = {
	{"sbrmi", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sbrmi_id);

static const struct of_device_id __maybe_unused sbrmi_of_match[] = {
	{
		.compatible = "amd,sbrmi",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sbrmi_of_match);


static struct i2c_driver sbrmi_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "sbrmi",
		.of_match_table = of_match_ptr(sbrmi_of_match),
	},
	.probe = sbrmi_probe,
	.remove = sbrmi_i2c_remove,
	.id_table = sbrmi_id,
};

module_i2c_driver(sbrmi_driver);

MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_DESCRIPTION("Hwmon driver for AMD SB-RMI emulated sensor");
MODULE_LICENSE("GPL");
