// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Nuvoton Technology Corporation.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>

#define SPMOD_MASK	GENMASK(2, 0)
static void __iomem *ctrl_reg;

static ssize_t sp_mode_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	u32 val = readl(ctrl_reg) + 1;

	return sprintf(buf, "%ld", val & SPMOD_MASK);
}

static ssize_t sp_mode_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 val;
	u8 mode;

	mode = buf[0] - 0x31;
	if (mode >= 0 && mode <= 6) {
		val = readl(ctrl_reg) & ~SPMOD_MASK;
		val |= mode;
		writel(mode, ctrl_reg);
	}

	return count;
}

static DEVICE_ATTR_RW(sp_mode);

static int npcm_sp_ctrl_probe(struct platform_device *pdev)
{
	int ret;

	ctrl_reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl_reg))
		return PTR_ERR(ctrl_reg);

	ret = sysfs_create_file(&pdev->dev.kobj,
				&dev_attr_sp_mode.attr);

	return ret;
}

static int npcm_sp_ctrl_remove(struct platform_device *pdev)
{
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_sp_mode.attr);

	return 0;
}

static const struct of_device_id npcm_sp_control_ids[] = {
	{ .compatible = "nuvoton,npcm-sp-ctrl", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, npcm_sp_control_ids);

static struct platform_driver npcm_sp_control_driver = {
	.driver = {
		.name = "npcm-sp-control",
		.of_match_table = npcm_sp_control_ids,
	},
	.probe = npcm_sp_ctrl_probe,
	.remove = npcm_sp_ctrl_remove,
};

module_platform_driver(npcm_sp_control_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver to configure serial port mode");
