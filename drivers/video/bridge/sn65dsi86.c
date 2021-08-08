// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 * Copyright (C) 2021 Patrick Wildt <patrick@blueri.se>
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <i2c.h>
#include <edid.h>
#include <log.h>
#include <asm/gpio.h>
#include <display.h>
#include <video_bridge.h>
#include <linux/delay.h>

struct sn65dsi86_priv {
	struct gpio_desc enable;
	u8 edid[EDID_SIZE];
};

static int sn65dsi86_write(struct udevice *dev, unsigned char reg_addr,
			   unsigned char value)
{
	struct dm_i2c_chip *chip = dev_get_parent_plat(dev);
	uint8_t buf[2];
	struct i2c_msg msg;
	int ret;

	msg.addr = chip->chip_addr;
	msg.flags = 0;
	buf[0] = reg_addr;
	buf[1] = value;
	msg.buf = buf;
	msg.len = 2;
	ret = dm_i2c_xfer(dev, &msg, 1);
	if (ret) {
		debug("%s: write failed, reg=%#x, value=%#x, ret=%d\n",
		      __func__, reg_addr, value, ret);
		return ret;
	}

	return 0;
}

static int sn65dsi86_read(struct udevice *dev, unsigned char reg_addr,
			  unsigned char *value)
{
	struct dm_i2c_chip *chip = dev_get_parent_plat(dev);
	uint8_t addr, val;
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = chip->chip_addr;
	msg[0].flags = 0;
	addr = reg_addr;
	msg[0].buf = &addr;
	msg[0].len = 1;
	msg[1].addr = chip->chip_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = 1;
	ret = dm_i2c_xfer(dev, msg, 2);
	if (ret) {
		debug("%s: read failed, reg=%.2x, value=%p, ret=%d\n",
		      __func__, (int)reg_addr, value, ret);
		return ret;
	}
	*value = val;

	return 0;
}

static int sn65dsi86_set_backlight(struct udevice *dev, int percent)
{
	return -ENOSYS;
}

static int sn65dsi86_read_edid(struct udevice *dev, u8 *buf, int size)
{
	struct sn65dsi86_priv *priv = dev_get_priv(dev);

	if (size > EDID_SIZE)
		size = EDID_SIZE;
	memcpy(buf, priv->edid, size);

	return size;
}

static int sn65dsi86_attach(struct udevice *dev)
{
	/* No-op */
	return 0;
}

//static int sn65dsi86_enable(struct udevice *dev)
//{
//	u8 chipid, colordepth, lanes, data_rate, c;
//	int ret, i, bpp;
//	struct display_timing timing;
//	struct sn65dsi86_priv *priv = dev_get_priv(dev);
//	int ret;
//
//	/* Deassert reset and enable power */
//	ret = video_bridge_set_active(dev, true);
//	if (ret)
//		return ret;
//
//	return 0;
//}

int sn65dsi86_enable(struct udevice *dev, int panel_bpp,
                     const struct display_timing *timing)
{
	struct sn65dsi86_priv *priv = dev_get_priv(dev);
	uint8_t reg;
	int i, j, ret;

	dm_gpio_set_value(&priv->enable, 1);
	udelay(1000);

	printf("%s:%d\n", __func__, __LINE__);

	/* Set DSI clock to 486 MHz */
	sn65dsi86_write(dev, 0x0a, 0x06);
	/* Single Channel, 4 DSI lanes */
	sn65dsi86_write(dev, 0x10, 0x26);
	/* Enhanced framing and ASSR */
	sn65dsi86_write(dev, 0x5a, 0x05);
	/* DSI CLK FREQ */
	sn65dsi86_write(dev, 0x12, 0x61);
	/* 24 bpp */
	sn65dsi86_write(dev, 0x5b, 0x00);
	/* 2 DP lanes w/o SSC */
	sn65dsi86_write(dev, 0x93, 0x20);
	/* 2.7 Gbps DP data rate */
	sn65dsi86_write(dev, 0x94, 0x80);
	/* Enable PLL and confirm PLL is locked */
	sn65dsi86_write(dev, 0x0d, 0x01);
	for (i = 50; i > 0; i--) {
		ret = sn65dsi86_read(dev, 0x0a, &reg);
		if (ret) {
			printf("%s:%d: can't read\n", __func__, __LINE__);
			return ret;
		}
		if (reg & (1U << 7))
			break;
		udelay(1000);
	}
	if (i == 0)
		printf("%s:%d: %x\n", __func__, __LINE__, reg);

	/* Enable ASSR on display */
	sn65dsi86_write(dev, 0x64, 0x01);
	sn65dsi86_write(dev, 0x75, 0x01);
	sn65dsi86_write(dev, 0x76, 0x0a);
	sn65dsi86_write(dev, 0x77, 0x01);
	sn65dsi86_write(dev, 0x78, 0x81);
	/* Train link and confirm link is trained */
	for (i = 10; i > 0; i--) {
		sn65dsi86_write(dev, 0x96, 0x0a);
		for (j = 500; j > 0; j--) {
			ret = sn65dsi86_read(dev, 0x96, &reg);
			if (ret) {
				printf("%s:%d: can't read\n", __func__, __LINE__);
				return ret;
			}
			if (reg == 0x01)
				break;
			udelay(1000);
		}
		if (reg == 0x01)
			break;
	}
	if (i == 0)
		printf("%s:%d: %x\n", __func__, __LINE__, reg);

	/* Line length 1920 */
	sn65dsi86_write(dev, 0x20, 0x80);
	sn65dsi86_write(dev, 0x21, 0x07);
	/* Vertical display size 1080 */
	sn65dsi86_write(dev, 0x24, 0x38);
	sn65dsi86_write(dev, 0x25, 0x04);
	/* HSync pulse width 40 */
	sn65dsi86_write(dev, 0x2c, 0x28);
	/* VSync pulse width 4 */
	sn65dsi86_write(dev, 0x30, 0x04);
	/* Horizonal back porch 80 */
	sn65dsi86_write(dev, 0x34, 0x50);
	/* Vertical back porch 24 */
	sn65dsi86_write(dev, 0x36, 0x18);
	/* Horizonal front porch 40 */
	sn65dsi86_write(dev, 0x38, 0x28);
	/* Vertical front porch 4 */
	sn65dsi86_write(dev, 0x3a, 0x04);

//	/* Enable color bar */
//	sn65dsi86_write(dev, 0x3c, 0x10);
	/* Enable video stream, ASSR, enhanced framing */
	sn65dsi86_write(dev, 0x5a, 0x0d);

	return 0;
}

static int sn65dsi86_of_to_plat(struct udevice *dev)
{
	struct sn65dsi86_priv *priv = dev_get_priv(dev);
	int ret;

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT);
	if (ret) {
		debug("%s: Warning: cannot get enable GPIO: ret=%d\n",
		    __func__, ret);
		if (ret != -ENOENT)
			return log_ret(ret);
	}

	return 0;
}

static int sn65dsi86_probe(struct udevice *dev)
{
	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	return 0;
//	return sn65dsi86_enable(dev);
}

#if 0
struct video_bridge_ops sn65dsi86_ops = {
	.attach = sn65dsi86_attach,
	.set_backlight = sn65dsi86_set_backlight,
	.read_edid = sn65dsi86_read_edid,
};
#endif

static const struct dm_display_ops sn65dsi86_ops = {
	.enable = sn65dsi86_enable,
};

static const struct udevice_id sn65dsi86_ids[] = {
	{ .compatible = "ti,sn65dsi86", },
	{ }
};

U_BOOT_DRIVER(ti_sn65dsi86) = {
	.name	= "ti_sn65dsi86",
//	.id	= UCLASS_VIDEO_BRIDGE,
	.id	= UCLASS_DISPLAY,
	.of_match = sn65dsi86_ids,
	.of_to_plat = sn65dsi86_of_to_plat,
	.probe	= sn65dsi86_probe,
	.ops	= &sn65dsi86_ops,
	.priv_auto	= sizeof(struct sn65dsi86_priv),
};
