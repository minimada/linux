// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved. */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/io.h>
#include <linux/reset-controller.h>
#include <linux/extcon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "ci.h"

#define INTCR3_OFFSET			0x9C

#define NPCM_INTCR3_USBPHYSW		GENMASK(13, 12)
#define NPCM845_INTCR3_USBPHYSW		GENMASK(15, 14)

struct ci_hdrc_usb2_npcm {
	struct platform_device	*ci;
	struct clk		*core_clk;
	struct ci_hdrc_platform_data pdata;
	enum usb_dr_mode dr_mode;
};

static int ci_hdrc_npcm_notify_event(struct ci_hdrc *ci, unsigned event)
{
	struct device *dev = ci->dev->parent;

	switch (event) {
	case CI_HDRC_CONTROLLER_RESET_EVENT:
		/* clear all mode bits */
		hw_write(ci, OP_USBMODE, 0xffffffff, 0);
		break;
	default:
		dev_dbg(dev, "unknown ci_hdrc event\n");
		break;
	}

	return 0;
}

static int ci_hdrc_npcm_probe(struct platform_device *pdev)
{	
	int ret, udc_id;
	struct regmap *gcr_regmap;
	struct ci_hdrc_usb2_npcm *ci;
	struct platform_device *plat_ci;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	ci = devm_kzalloc(&pdev->dev, sizeof(*ci), GFP_KERNEL);
	if (!ci)
		return -ENOMEM;
	platform_set_drvdata(pdev, ci);

	ci->core_clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(ci->core_clk))
		return PTR_ERR(ci->core_clk);

	ret = clk_prepare_enable(ci->core_clk);
	if (ret) {
		dev_err(dev, "failed to enable the clock: %d\n", ret);
		return ret;
	}

	ci->pdata.name = dev_name(dev);
	ci->pdata.capoffset = DEF_CAPOFFSET;
	ci->pdata.flags	= CI_HDRC_REGS_SHARED | 
		CI_HDRC_FORCE_VBUS_ACTIVE_ALWAYS;
	ci->pdata.phy_mode = USBPHY_INTERFACE_MODE_UTMI;
	ci->pdata.notify_event = ci_hdrc_npcm_notify_event;

	udc_id = of_alias_get_id(np, "udc");
	if (udc_id == 9) {
		gcr_regmap = syscon_regmap_lookup_by_phandle(np,
							     "nuvoton,sysgcr");
		if (IS_ERR(gcr_regmap)) {
			dev_err(dev, "Failed to find nuvoton,sysgcr\n");
			return IS_ERR(gcr_regmap);
		}
		regmap_update_bits(gcr_regmap, INTCR3_OFFSET, 
				   NPCM_INTCR3_USBPHYSW, NPCM_INTCR3_USBPHYSW);
	}

	if (udc_id == 8 && of_device_is_compatible(np, "nuvoton,npcm845-udc")) {
		pr_info("\t\t udc 8 change\n\n\n");
		gcr_regmap = syscon_regmap_lookup_by_phandle(np,
							     "nuvoton,sysgcr");
		if (IS_ERR(gcr_regmap)) {
			dev_err(dev, "Failed to find nuvoton,sysgcr\n");
			return IS_ERR(gcr_regmap);
		}
		regmap_update_bits(gcr_regmap, INTCR3_OFFSET,
				   NPCM845_INTCR3_USBPHYSW,
				   NPCM845_INTCR3_USBPHYSW);
	}

	plat_ci = ci_hdrc_add_device(dev, pdev->resource, pdev->num_resources,
				     &ci->pdata);
	if (IS_ERR(plat_ci)) {
		ret = PTR_ERR(plat_ci);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to register ci_hdrc platform device: %d\n", ret);
		goto clk_err;
	}

	platform_set_drvdata(pdev, ci);

	pm_runtime_no_callbacks(dev);
	pm_runtime_enable(dev);

	return 0;

clk_err:
	clk_disable_unprepare(ci->core_clk);
	return ret;
}

static int ci_hdrc_npcm_remove(struct platform_device *pdev)
{
	struct ci_hdrc_usb2_npcm *ci = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	ci_hdrc_remove_device(ci->ci);
	clk_disable_unprepare(ci->core_clk);

	return 0;
}

static const struct of_device_id npcm_ci_dt_match[] = {
	{ .compatible = "nuvoton,npcm750-udc", },
	{ .compatible = "nuvoton,npcm845-udc", },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_ci_dt_match);

static struct platform_driver ci_hdrc_npcm_driver = {
	.probe = ci_hdrc_npcm_probe,
	.remove = ci_hdrc_npcm_remove,
	.driver = {
		.name = "npcm_hsusb",
		.of_match_table = npcm_ci_dt_match,
	},
};

module_platform_driver(ci_hdrc_npcm_driver);

MODULE_ALIAS("platform:npcm_hsusb");
MODULE_ALIAS("platform:npcm");
MODULE_LICENSE("GPL v2");
