// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Nuvoton Technology corporation.
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define ESPI_ESPICFG		0x004
#define   ESPICFG_VWCHANEN	BIT(1)
#define   ESPICFG_VWCHN_SUPP	BIT(25)
#define ESPI_ESPISTS		0x008
#define   ESPISTS_VWUPD		BIT(8)
#define ESPI_ESPIIE		0x00C
#define   ESPIIE_VWUPDIE	BIT(8)
#define ESPI_VWGPSM(n)		(0x180 + (4*(n)))
#define   VWGPSM_INDEX_EN	BIT(15)
#define ESPI_VWGPMS(n)		(0x1C0 + (4*(n)))
#define   VWGPMS_INDEX_EN	BIT(15)
#define   VWGPMS_MODIFIED	BIT(16)
#define   VWGPMS_IE		BIT(18)
#define ESPI_VWCTL		0x2FC
#define   VWCTL_INTWIN(x)	FIELD_PREP(GENMASK(1, 0), (x))
#define   VWCTL_GPVWMAP(x)	FIELD_PREP(GENMASK(3, 2), (x))
#define   VWCTL_IRQD		BIT(4)
#define   VWCTL_NOVALID		BIT(5)

#define VWGPSM_INDEX_COUNT	16
#define VWGPMS_INDEX_COUNT	16
/* SM GPIO: 0~63, MS GPIO: 64~127 */
#define VM_SMGPIO_START		0
#define VW_SMGPIO_NUM		64
#define VM_MSGPIO_START		64
#define VW_MSGPIO_NUM		64

/* vwgpio_event type */
#define VW_GPIO_EVENT_EDGE_RISING	0
#define VW_GPIO_EVENT_EDGE_FALLING	1
#define VW_GPIO_EVENT_EDGE_BOTH		2
#define VW_GPIO_EVENT_LEVEL_HIGH	3
#define VW_GPIO_EVENT_LEVEL_LOW		4
struct vwgpio_event {
	u8 enable : 1;
	u8 state: 1;
	u8 type: 3;
	u8 flags: 3;
};

struct npcm_espi_vw {
	struct gpio_chip chip;
	struct device *dev;
	void __iomem *regs;
	struct vwgpio_event events[VW_MSGPIO_NUM];
	int irq;
};

static int vwgpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(gc);
	u32 wire = offset % 4;
	u32 index;
	u32 val;

	dev_dbg(gc->parent, "%s: offset=%u\n", __func__, offset);
	/* Accept MS GPIO only */
	if (offset < VM_MSGPIO_START || offset >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return -EINVAL;

	index = (offset - VM_MSGPIO_START) / 4;
	val = ioread32(espi_vw->regs + ESPI_VWGPMS(index));
	/* Check wire valid bit */
	if (!(val & BIT(wire + 4)))
		return -EIO;

	return !!(val & BIT(wire));
}

static void vwgpio_set_value(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(gc);
	u32 index = offset / 4;
	u32 wire = offset % 4;
	u32 reg;

	dev_dbg(gc->parent, "%s: offset=%u, val=%d\n", __func__,
		  offset, val);
	/* Accept SM GPIO only */
	if (offset >= VW_SMGPIO_NUM)
		return;

	reg = ioread32(espi_vw->regs + ESPI_VWGPSM(index));
	/* Set index enable & wire valid */
	reg |= BIT(wire + 4) | VWGPSM_INDEX_EN;
	if (val)
		reg |= BIT(wire);
	else
		reg &= ~BIT(wire);
	iowrite32(reg, espi_vw->regs + ESPI_VWGPSM(index));
}

static int vwgpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	return (offset >= VW_SMGPIO_NUM) ? GPIO_LINE_DIRECTION_IN
		: GPIO_LINE_DIRECTION_OUT;
}

static int vwgpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	return (offset >= VW_SMGPIO_NUM) ? 0 : -EINVAL;
}

static int vwgpio_direction_output(struct gpio_chip *gc, unsigned int offset, int val)
{
	if (offset < VW_SMGPIO_NUM) {
		gc->set(gc, offset, val);
		return 0;
	}
	return -EINVAL;
}

static void npcm_espi_vwgpms_config(struct npcm_espi_vw *espi_vw, int index,
			       bool enable, bool int_en)
{
	u32 val;

	val = ioread32(espi_vw->regs + ESPI_VWGPMS(index));
	if (enable)
		val |= VWGPMS_INDEX_EN;
	else
		val &= ~VWGPMS_INDEX_EN;

	if (int_en)
		val |= VWGPMS_IE;
	else
		val &= ~VWGPMS_IE;

	iowrite32(val, espi_vw->regs + ESPI_VWGPMS(index));
}

static void npcm_vwgpio_check_event(struct npcm_espi_vw *espi_vw,
				   unsigned int event_idx)
{
	struct gpio_chip *gc = &espi_vw->chip;
	struct vwgpio_event *event;
	bool raise_irq = false;
	unsigned int girq;
	u32 index = event_idx / 4;
	u32 wire = event_idx % 4;
	u8 new_state;
	u32 val;

	if (event_idx >= VW_MSGPIO_NUM)
		return;

	event = &espi_vw->events[event_idx];

	val = ioread32(espi_vw->regs + ESPI_VWGPMS(index));
	/* Clear MODIFIED bit */
	iowrite32(val | VWGPMS_MODIFIED, espi_vw->regs + ESPI_VWGPMS(index));

	/* Check event enable */
	if (!event->enable)
		return;

	/* Check wire valid bit */
	if (!(val & BIT(wire + 4)))
		return;

	new_state = !!(val & BIT(wire));
	switch (event->type) {
	case VW_GPIO_EVENT_EDGE_RISING:
		if (event->state == 0 && new_state == 1) {
			event->state = new_state;
			raise_irq = true;
		}
		break;
	case VW_GPIO_EVENT_EDGE_FALLING:
		if (event->state == 1 && new_state == 0) {
			event->state = new_state;
			raise_irq = true;
		}
		break;
	case VW_GPIO_EVENT_EDGE_BOTH:
		if ((event->state == 1 && new_state == 0) ||
		    (event->state == 0 && new_state == 1)) {
			event->state = new_state;
			raise_irq = true;
		}
		break;
	case VW_GPIO_EVENT_LEVEL_HIGH:
		if (new_state == 1) {
			event->state = new_state;
			raise_irq = true;
		}
		break;
	case VW_GPIO_EVENT_LEVEL_LOW:
		if (new_state == 0) {
			event->state = new_state;
			raise_irq = true;
		}
		break;
	}

	if (raise_irq) {
		girq = irq_find_mapping(gc->irq.domain,
					VM_MSGPIO_START + event_idx);
		generic_handle_irq(girq);
	}
}

static void npcm_vwgpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_chip *ic = irq_desc_get_chip(desc);
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(gc);
	u32 status;
	int i;

	chained_irq_enter(ic, desc);

	status = ioread32(espi_vw->regs + ESPI_ESPISTS);
	/* Clear status */
	iowrite32(status, espi_vw->regs + ESPI_ESPISTS);
	if (!(status & ESPISTS_VWUPD))
		goto exit;
	/* Check all events */
	for (i = 0; i < VW_MSGPIO_NUM; i++)
		npcm_vwgpio_check_event(espi_vw, i);
exit:
	chained_irq_exit(ic, desc);
}

static int npcm_vwgpio_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	unsigned int gpio = irqd_to_hwirq(d);
	unsigned int event_idx;

	pr_debug("%s: gpio %u, type %d\n", __func__, gpio, type);
	if (gpio < VM_MSGPIO_START || gpio >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return -EINVAL;
	event_idx = gpio - VM_MSGPIO_START;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		espi_vw->events[event_idx].type = VW_GPIO_EVENT_EDGE_RISING;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		espi_vw->events[event_idx].type = VW_GPIO_EVENT_EDGE_FALLING;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		espi_vw->events[event_idx].type = VW_GPIO_EVENT_EDGE_BOTH;
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		espi_vw->events[event_idx].type = VW_GPIO_EVENT_LEVEL_LOW;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		espi_vw->events[event_idx].type = VW_GPIO_EVENT_LEVEL_HIGH;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void npcm_vwgpio_irq_ack(struct irq_data *d)
{
}

static void npcm_vwgpio_irq_mask(struct irq_data *d)
{
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	unsigned int gpio = irqd_to_hwirq(d);
	bool int_enable = false;
	int index;
	int wire;

	pr_debug("%s: gpio %u\n", __func__, gpio);
	/* Accept MS GPIO only */
	if (gpio < VM_MSGPIO_START || gpio >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return;

	espi_vw->events[gpio - VM_MSGPIO_START].enable = 0;
	index = (gpio - VM_MSGPIO_START) / 4;
	/* Check all wires in the same VMGPIOMS index */
	for (wire = 0; wire < 4; wire++) {
		if (espi_vw->events[index * 4 + wire].enable) {
			int_enable = true;
			break;
		}
	}
	if (!int_enable)
		npcm_espi_vwgpms_config(espi_vw, index, true, false);
}

static void npcm_vwgpio_irq_unmask(struct irq_data *d)
{
	struct npcm_espi_vw *espi_vw = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	unsigned int gpio = irqd_to_hwirq(d);
	int index;

	pr_debug("%s: gpio %u\n", __func__, gpio);
	/* Accept MS GPIO only */
	if (gpio < VM_MSGPIO_START || gpio >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return;

	/* Get current state */
	espi_vw->events[gpio - VM_MSGPIO_START].state =
		vwgpio_get_value(&espi_vw->chip, gpio);

	/* Set event enable */
	espi_vw->events[gpio - VM_MSGPIO_START].enable = 1;
	index = (gpio - VM_MSGPIO_START) / 4;
	npcm_espi_vwgpms_config(espi_vw, index, true, true);
}

static struct irq_chip npcm_vwgpio_irqchip = {
	.name = "NPCM-VW-GPIO-IRQ",
	.irq_ack = npcm_vwgpio_irq_ack,
	.irq_unmask = npcm_vwgpio_irq_unmask,
	.irq_mask = npcm_vwgpio_irq_mask,
	.irq_set_type = npcm_vwgpio_set_irq_type,
	.flags = IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_IMMUTABLE,
};

static void npcm_espi_vwgpio_init(struct npcm_espi_vw *espi_vw)
{
	u32 val;
	int i;

	/* Get gpio initial state */
	memset(&espi_vw->events, 0, sizeof(espi_vw->events));
	for (i = 0; i < VW_MSGPIO_NUM; i++)
		espi_vw->events[i].state =
			vwgpio_get_value(&espi_vw->chip, VM_MSGPIO_START + i);

	/* enable VWUPD interrupt */
	val = ioread32(espi_vw->regs + ESPI_ESPIIE);
	val |= ESPIIE_VWUPDIE;
	iowrite32(val, espi_vw->regs + ESPI_ESPIIE);
}

static void npcm_espi_vw_config(struct npcm_espi_vw *espi_vw, u8 intwin,
				u8 gpvwmap, bool irq_dis, bool no_valid)
{
	u32 val = VWCTL_INTWIN(intwin) | VWCTL_GPVWMAP(gpvwmap);

	if (irq_dis)
		val |= VWCTL_IRQD;
	if (no_valid)
		val |= VWCTL_NOVALID;

	iowrite32(val, espi_vw->regs + ESPI_VWCTL);
}

static void npcm_espi_vw_index_en(struct npcm_espi_vw *espi_vw,
				  u32 idxenmap)
{
	u32 val;
	int i;

	for (i = 0; i < VWGPSM_INDEX_COUNT; i++) {
		if (!(idxenmap & BIT(i)))
			continue;
		val = ioread32(espi_vw->regs + ESPI_VWGPSM(i));
		val |= VWGPSM_INDEX_EN;
		iowrite32(val, espi_vw->regs + ESPI_VWGPSM(i));
	}
	for (i = 0; i < VWGPMS_INDEX_COUNT; i++) {
		if (!(idxenmap & BIT(i + VWGPSM_INDEX_COUNT)))
			continue;
		val = ioread32(espi_vw->regs + ESPI_VWGPMS(i));
		val |= VWGPMS_INDEX_EN;
		iowrite32(val, espi_vw->regs + ESPI_VWGPMS(i));
	}
}

static int npcm_espi_vw_probe(struct platform_device *pdev)
{
	struct npcm_espi_vw *espi_vw;
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *irq;
	u32 gpvwmap, intwin, idxenmap;
	int rc;

	espi_vw = devm_kzalloc(dev, sizeof(*espi_vw), GFP_KERNEL);
	if (!espi_vw)
		return -ENOMEM;

	espi_vw->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(espi_vw->regs))
		return PTR_ERR(espi_vw->regs);

	espi_vw->irq = platform_get_irq(pdev, 0);
	if (espi_vw->irq < 0)
		return espi_vw->irq;

	espi_vw->dev = dev;

	if (of_property_read_u32(pdev->dev.of_node, "npcm,gpio-control-map", &gpvwmap))
		gpvwmap = 0;
	if (of_property_read_u32(pdev->dev.of_node, "npcm,control-interrupt-map", &intwin))
		intwin = 0;
	if (of_property_read_u32(pdev->dev.of_node, "npcm,index-en-map", &idxenmap))
		idxenmap = 0;

	npcm_espi_vw_config(espi_vw, intwin, gpvwmap, false, false);
	npcm_espi_vw_index_en(espi_vw, idxenmap);

	if (of_find_property(pdev->dev.of_node, "gpio-controller", NULL)) {
		espi_vw->chip.of_node = pdev->dev.of_node;
		espi_vw->chip.label = "ESPI_VW_GPIO";
		espi_vw->chip.base = -1;
		espi_vw->chip.parent = dev;
		espi_vw->chip.owner = THIS_MODULE;
		espi_vw->chip.ngpio = 128;
		espi_vw->chip.can_sleep = 0;
		espi_vw->chip.get = vwgpio_get_value;
		espi_vw->chip.set = vwgpio_set_value;
		espi_vw->chip.direction_output = vwgpio_direction_output;
		espi_vw->chip.direction_input = vwgpio_direction_input;
		espi_vw->chip.get_direction = vwgpio_get_direction;

		irq = &espi_vw->chip.irq;
		irq->chip = &npcm_vwgpio_irqchip;
		irq->handler = handle_level_irq;
		irq->default_type = IRQ_TYPE_NONE;
		irq->parent_handler = npcm_vwgpio_irq_handler;
		irq->parents = &espi_vw->irq;
		irq->num_parents = 1;

		rc = devm_gpiochip_add_data(dev, &espi_vw->chip, espi_vw);
		if (rc) {
			pr_info("Error adding ESPI vw gpiochip\n");
			return rc;
		}
		npcm_espi_vwgpio_init(espi_vw);
	}

	pr_info("%s OK\n", __func__);

	return 0;
}

static int npcm_espi_vw_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id npcm_espi_vw_of_matches[] = {
	{ .compatible = "nuvoton,npcm7xx-espi-vw", },
	{ .compatible = "nuvoton,npcm8xx-espi-vw", },
	{ },
};

static struct platform_driver npcm_espi_vw_driver = {
	.driver = {
		.name = "npcm-espi-vw",
		.of_match_table = npcm_espi_vw_of_matches,
	},
	.probe = npcm_espi_vw_probe,
	.remove = npcm_espi_vw_remove,
};

module_platform_driver(npcm_espi_vw_driver);

MODULE_DESCRIPTION("Nuvoton NPCM eSPI Virtual Wire Driver");
MODULE_LICENSE("GPL v2");
