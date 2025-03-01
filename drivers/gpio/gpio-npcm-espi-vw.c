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
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/mfd/syscon.h>

#define ESPI_ESPICFG		0x004
#define   ESPICFG_VWCHANEN	BIT(1)
#define   ESPICFG_VWCHN_SUPP	BIT(25)
#define ESPI_ESPISTS		0x008
#define   ESPISTS_VWUPD		BIT(8)
#define ESPI_ESPIIE		0x00C
#define   ESPIIE_VWUPDIE	BIT(8)
#define ESPI_VWGPSM(n)		(0x180 + (4*(n)))
#define   VWGPSM_INDEX_EN	BIT(15)
#define ESPI_ESPIERR		0x03C
#define   ESPIERR_VWERR		BIT(11)
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

struct npcm_vwgpio {
	struct gpio_chip chip;
	struct device *dev;
	struct regmap *map;
	struct vwgpio_event events[VW_MSGPIO_NUM];
	int irq;
	raw_spinlock_t lock;
	u64 mswire_default;
};

static int vwgpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct npcm_vwgpio *vwgpio = gpiochip_get_data(gc);
	u32 wire = offset % 4;
	u32 index;
	u32 val;

	dev_dbg(gc->parent, "%s: offset=%u\n", __func__, offset);
	/* Accept SM/MS GPIO */
	if (offset >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return -EINVAL;

	if (offset >= VM_MSGPIO_START) {
		index = (offset - VM_MSGPIO_START) / 4;
		regmap_read(vwgpio->map, ESPI_VWGPMS(index), &val);
		/* Check wire valid bit, invalid return default value */
		if (!(val & BIT(wire + 4)))
			return !!(vwgpio->mswire_default &
				  BIT(offset - VM_MSGPIO_START));
	}
	else {
		index = offset / 4;
		regmap_read(vwgpio->map, ESPI_VWGPSM(index), &val);
		/* Check wire valid bit*/
		if (!(val & BIT(wire + 4)))
			return -EIO;
	}

	return !!(val & BIT(wire));
}

static void vwgpio_set_value(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct npcm_vwgpio *vwgpio = gpiochip_get_data(gc);
	unsigned long flags;
	u32 index = offset / 4;
	u32 wire = offset % 4;
	u32 reg;

	dev_dbg(gc->parent, "%s: offset=%u, val=%d\n", __func__,
		offset, val);
	/* Accept SM GPIO only */
	if (offset >= VW_SMGPIO_NUM)
		return;

	regmap_read(vwgpio->map, ESPI_VWGPSM(index), &reg);
	/* Set index enable & wire valid */
	reg |= BIT(wire + 4) | VWGPSM_INDEX_EN;
	if (val)
		reg |= BIT(wire);
	else
		reg &= ~BIT(wire);

	raw_spin_lock_irqsave(&vwgpio->lock, flags);

	regmap_write(vwgpio->map, ESPI_VWGPSM(index), reg);

	raw_spin_unlock_irqrestore(&vwgpio->lock, flags);
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

static int vwgpio_direction_output(struct gpio_chip *gc,
				   unsigned int offset, int val)
{
	gc->set(gc, offset, val);
	return 0;
}

static void npcm_vwgpio_check_event(struct npcm_vwgpio *vwgpio,
				    unsigned int event_idx)
{
	struct gpio_chip *gc = &vwgpio->chip;
	struct vwgpio_event *event;
	bool raise_irq = false;
	unsigned int girq;
	u32 index = event_idx / 4;
	u32 wire = event_idx % 4;
	unsigned long flags;
	u8 new_state;
	u32 val;

	if (event_idx >= VW_MSGPIO_NUM)
		return;

	regmap_read(vwgpio->map, ESPI_VWGPMS(index), &val);
	/* Clear MODIFIED bit */
	regmap_write(vwgpio->map, ESPI_VWGPMS(index), val | VWGPMS_MODIFIED);

	raw_spin_lock_irqsave(&vwgpio->lock, flags);

	event = &vwgpio->events[event_idx];

	/* Check event enable */
	if (!event->enable)
		goto out;

	/* Check wire valid bit */
	if (!(val & BIT(wire + 4)))
		goto out;

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
	default:
		break;
	}

out:
	raw_spin_unlock_irqrestore(&vwgpio->lock, flags);

	if (raise_irq)
		generic_handle_domain_irq(gc->irq.domain,
					  VM_MSGPIO_START + event_idx);
}

static irqreturn_t npcm_vwgpio_irq_handler(int irq, void *dev_id)
{
	struct npcm_vwgpio *vwgpio = dev_id;
	u32 status;
	int i;

	regmap_read(vwgpio->map, ESPI_ESPISTS, &status);
	if (!(status & ESPISTS_VWUPD))
		return IRQ_HANDLED;

	/* Clear ESPISTS_VWUPD status */
	regmap_write(vwgpio->map, ESPI_ESPISTS, ESPISTS_VWUPD);
	/* Check all events */
	for (i = 0; i < VW_MSGPIO_NUM; i++)
		npcm_vwgpio_check_event(vwgpio, i);

	return IRQ_HANDLED;
}

static int npcm_vwgpio_set_irq_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct npcm_vwgpio *vwgpio = gpiochip_get_data(gc);
	unsigned int gpio = irqd_to_hwirq(d);
	irq_flow_handler_t handler;
	unsigned int event_idx;
	unsigned long flags;
	u8 value;

	dev_dbg(vwgpio->dev, "gpio %u, type %d\n", gpio, type);
	if (gpio < VM_MSGPIO_START || gpio >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return -EINVAL;
	event_idx = gpio - VM_MSGPIO_START;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		value = VW_GPIO_EVENT_EDGE_RISING;
		fallthrough;
	case IRQ_TYPE_EDGE_FALLING:
		value = VW_GPIO_EVENT_EDGE_FALLING;
		fallthrough;
	case IRQ_TYPE_EDGE_BOTH:
		value = VW_GPIO_EVENT_EDGE_BOTH;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		value = VW_GPIO_EVENT_LEVEL_LOW;
		fallthrough;
	case IRQ_TYPE_LEVEL_HIGH:
		value = VW_GPIO_EVENT_LEVEL_HIGH;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&vwgpio->lock, flags);

	vwgpio->events[event_idx].type = value;

	raw_spin_unlock_irqrestore(&vwgpio->lock, flags);

	irq_set_handler_locked(d, handler);

	return 0;
}

static void npcm_vwgpio_irq_ack(struct irq_data *d)
{
}

static void npcm_vwgpio_irq_set_mask(struct irq_data *d, bool enable, bool set)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct npcm_vwgpio *vwgpio = gpiochip_get_data(gc);
	unsigned int gpio = irqd_to_hwirq(d);
	bool int_enable = false;
	unsigned long flags;
	int index, wire;
	u32 val;

	if (gpio < VM_MSGPIO_START || gpio >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return;

	index = (gpio - VM_MSGPIO_START) / 4;

	raw_spin_lock_irqsave(&vwgpio->lock, flags);

	if (!set) {
		vwgpio->events[gpio - VM_MSGPIO_START].enable = 0;
		/* Check all wires in the same VMGPIOMS index */
		for (wire = 0; wire < 4; wire++) {
			if (vwgpio->events[index * 4 + wire].enable) {
				int_enable = true;
				break;
			}
		}
	} else {
		/* Set event enable */
		vwgpio->events[gpio - VM_MSGPIO_START].enable = 1;
		/* Get current state */
		vwgpio->events[gpio - VM_MSGPIO_START].state =
			vwgpio_get_value(&vwgpio->chip, gpio);
	}

	if (!int_enable) {
		regmap_read(vwgpio->map, ESPI_VWGPMS(index), &val);

		if (enable)
			val |= VWGPMS_INDEX_EN;
		else
			val &= ~VWGPMS_INDEX_EN;

		if (set)
			val |= VWGPMS_IE;
		else
			val &= ~VWGPMS_IE;

		regmap_write(vwgpio->map, ESPI_VWGPMS(index), val);
	}

        raw_spin_unlock_irqrestore(&vwgpio->lock, flags);
}

static void npcm_vwgpio_irq_mask(struct irq_data *d)
{
	npcm_vwgpio_irq_set_mask(d, true, false);
}

static void npcm_vwgpio_irq_unmask(struct irq_data *d)
{
	npcm_vwgpio_irq_set_mask(d, true, true);
}

static struct irq_chip npcm_vwgpio_irqchip = {
	.name = "NPCM-VW-GPIO-IRQ",
	.irq_ack = npcm_vwgpio_irq_ack,
	.irq_unmask = npcm_vwgpio_irq_unmask,
	.irq_mask = npcm_vwgpio_irq_mask,
	.irq_set_type = npcm_vwgpio_set_irq_type,
	.flags = IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_IMMUTABLE,
};

static void npcm_vwgpio_init(struct npcm_vwgpio *vwgpio)
{
	int i;

	/* Get gpio initial state */
	memset(&vwgpio->events, 0, sizeof(vwgpio->events));
	for (i = 0; i < VW_MSGPIO_NUM; i++)
		vwgpio->events[i].state =
			vwgpio_get_value(&vwgpio->chip, VM_MSGPIO_START + i);

	/* enable VWUPD interrupt */
	regmap_set_bits(vwgpio->map, ESPI_ESPIIE, ESPIIE_VWUPDIE);
}

static void npcm_vwgpio_config(struct npcm_vwgpio *vwgpio, u8 intwin,
			       u8 gpvwmap, u32 idxenmap, u64 smwiremap)
{
	u32 val = VWCTL_INTWIN(intwin) | VWCTL_GPVWMAP(gpvwmap);
	u32 reg;
	int i, j;

	regmap_write(vwgpio->map, ESPI_VWCTL, val);

	for (i = 0; i < VWGPSM_INDEX_COUNT; i++) {
		regmap_read(vwgpio->map, ESPI_VWGPSM(i), &reg);
		if ((idxenmap & BIT(i))) {
			reg |= VWGPSM_INDEX_EN;
			for (j = 0; j < 4; j++) {
				reg |= BIT(j + 4);
				if (!(smwiremap & BIT((i * 4) + j)))
					reg &= ~BIT(j);
				else
					reg |= BIT(j);
			}
		}
		else {
			reg &= ~VWGPSM_INDEX_EN;
			for (j = 0; j < 4; j++) {
				reg &= ~BIT(j + 4);
			}
		}
		regmap_write(vwgpio->map, ESPI_VWGPSM(i), reg);
	}
	for (i = 0; i < VWGPMS_INDEX_COUNT; i++) {
		if (idxenmap & BIT(i + VWGPSM_INDEX_COUNT))
			regmap_set_bits(vwgpio->map, ESPI_VWGPMS(i),
					VWGPMS_INDEX_EN);
		else
			regmap_clear_bits(vwgpio->map, ESPI_VWGPMS(i),
					VWGPMS_INDEX_EN);
	}
}

static int npcm_vwgpio_probe(struct platform_device *pdev)
{
	struct npcm_vwgpio *vwgpio;
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *irq;
	u32 gpvwmap, intwin, idxenmap;
	u64 mswiremap, smwiremap;
	int rc;

	vwgpio = devm_kzalloc(dev, sizeof(*vwgpio), GFP_KERNEL);
	if (!vwgpio)
		return -ENOMEM;

	vwgpio->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(vwgpio->map)) {
		dev_err(dev, "Couldn't get regmap\n");
		return -ENODEV;
	}

	vwgpio->irq = platform_get_irq(pdev, 0);
	if (vwgpio->irq < 0)
		return vwgpio->irq;

	vwgpio->dev = dev;

	if (of_property_read_u32(pdev->dev.of_node,
				 "nuvoton,gpio-control-map", &gpvwmap))
		gpvwmap = 0;
	if (of_property_read_u32(pdev->dev.of_node,
				 "nuvoton,control-interrupt-map", &intwin))
		intwin = 0;
	if (of_property_read_u32(pdev->dev.of_node,
				 "nuvoton,index-en-map", &idxenmap))
		idxenmap = 0;
	if (of_property_read_u64(pdev->dev.of_node,
				 "nuvoton,vwgpms-wire-map", &mswiremap))
		vwgpio->mswire_default = 0;
	else
		vwgpio->mswire_default = mswiremap;
	if (of_property_read_u64(pdev->dev.of_node,
				 "nuvoton,vwgpsm-wire-map", &smwiremap))
		smwiremap = 0xFFFFFFFFFFFFFFFFULL;

	npcm_vwgpio_config(vwgpio, intwin, gpvwmap, idxenmap, smwiremap);

	if (of_find_property(pdev->dev.of_node, "gpio-controller", NULL)) {
		vwgpio->chip.of_node = pdev->dev.of_node;
		vwgpio->chip.label = "ESPI_VW_GPIO";
		vwgpio->chip.base = -1;
		vwgpio->chip.parent = dev;
		vwgpio->chip.owner = THIS_MODULE;
		vwgpio->chip.ngpio = VW_SMGPIO_NUM + VW_MSGPIO_NUM;
		vwgpio->chip.can_sleep = 0;
		vwgpio->chip.get = vwgpio_get_value;
		vwgpio->chip.set = vwgpio_set_value;
		vwgpio->chip.direction_output = vwgpio_direction_output;
		vwgpio->chip.direction_input = vwgpio_direction_input;
		vwgpio->chip.get_direction = vwgpio_get_direction;

		raw_spin_lock_init(&vwgpio->lock);

		irq = &vwgpio->chip.irq;
		irq->chip = &npcm_vwgpio_irqchip;
		irq->handler = handle_bad_irq;
		irq->default_type = IRQ_TYPE_NONE;
		irq->parent_handler = NULL;
		irq->parents = NULL;
		irq->num_parents = 0;

		rc = devm_gpiochip_add_data(dev, &vwgpio->chip, vwgpio);
		if (rc) {
			dev_err(dev, "Error adding ESPI vw gpiochip\n");
			return rc;
		}

		/* Clear ESPISTS_VWUPD status */
		regmap_write(vwgpio->map, ESPI_ESPISTS, ESPISTS_VWUPD);
		/* Clear ESPIERR_VWERR status */
		regmap_write(vwgpio->map, ESPI_ESPIERR, ESPIERR_VWERR);
		/* Disable VWUPD interrupt */
		regmap_clear_bits(vwgpio->map, ESPI_ESPIIE, ESPIIE_VWUPDIE);

		rc = devm_request_irq(dev, vwgpio->irq, npcm_vwgpio_irq_handler,
				      IRQF_SHARED, "espi-vw-gpio", vwgpio);
		if (rc) {
			dev_err(dev, "failed to request IRQ\n");
			return rc;
		}
		npcm_vwgpio_init(vwgpio);
	}

	return 0;
}

static int npcm_vwgpio_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id npcm_vwgpio_of_matches[] = {
	{ .compatible = "nuvoton,npcm845-espi-vwgpio", },
	{ },
};

static struct platform_driver npcm_vwgpio_driver = {
	.driver = {
		.name = "npcm-espi-vwgpio",
		.of_match_table = npcm_vwgpio_of_matches,
	},
	.probe = npcm_vwgpio_probe,
	.remove = npcm_vwgpio_remove,
};

module_platform_driver(npcm_vwgpio_driver);

MODULE_AUTHOR("Tomer Maimon <Tomer.Maimon@nuvoton.com>");
MODULE_AUTHOR("Ban Feng <kcfeng0@nuvoton.com>");
MODULE_AUTHOR("Stanley Chu <yschu@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NPCM eSPI Virtual Wire GPIO Driver");
MODULE_LICENSE("GPL v2");
