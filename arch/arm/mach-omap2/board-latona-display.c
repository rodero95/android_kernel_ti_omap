/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom-display.c for Samsung latona board.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/leds.h>
#include <linux/leds-omap-display.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include "mux.h"

#define LCD_PANEL_ENABLE_GPIO		(7 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_RESET_GPIO_PROD	96
#define LCD_PANEL_RESET_GPIO_PILOT	55
#define LCD_PANEL_QVGA_GPIO			56

int omap_mux_init_signal(const char *muxname, int val);

struct latona_dss_board_info {
	int gpio_flag;
};

static int latona_panel_power_enable(int enable)
{
	int ret;
	struct regulator *vdds_dsi_reg;

	vdds_dsi_reg = regulator_get(NULL, "vdds_dsi");
	if (IS_ERR(vdds_dsi_reg)) {
		pr_err("Unable to get vdds_dsi regulator\n");
		return PTR_ERR(vdds_dsi_reg);
	}

	if (enable)
		ret = regulator_enable(vdds_dsi_reg);
	else
		ret = regulator_disable(vdds_dsi_reg);

	return ret;
}

static int latona_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	int ret;
	struct latona_dss_board_info *pdata;
	struct display_led_data *b_led;

	ret = latona_panel_power_enable(1);
	if (ret < 0)
		return ret;
	pdata = dssdev->dev.platform_data;
	if (pdata->gpio_flag == 0) {
		ret = gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd enable");
		if (ret) {
			pr_err("Failed to get LCD_PANEL_ENABLE_GPIO.\n");
			return ret;
		}
		gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);
		pdata->gpio_flag = 1;
	} else {
		gpio_set_value(LCD_PANEL_ENABLE_GPIO, 1);
	}

	b_led = get_omap_led_info();
	if(b_led){
		mutex_lock(&b_led->pri_disp_lock);
		b_led->led_pdata->primary_display_set(
			b_led->pri_display_class_dev.brightness);
		mutex_unlock(&b_led->pri_disp_lock);
	}else{
		pr_err("unable to get backlight led data");
	}

	return 0;
}

static void latona_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	struct display_led_data *b_led;

	b_led = get_omap_led_info();
	if(b_led){
		mutex_lock(&b_led->pri_disp_lock);
		b_led->led_pdata->primary_display_set(0);
		mutex_unlock(&b_led->pri_disp_lock);
	}else{
		pr_err("unable to get backlight led data");
	}

	latona_panel_power_enable(0);
	gpio_set_value(LCD_PANEL_ENABLE_GPIO, 0);
}

static struct latona_dss_board_info latona_dss_lcd_data = {
	.gpio_flag = 0,
};

static struct omap_dss_device latona_lcd_device = {
	.name = "lcd",
	.driver_name = "nt35510_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = NULL,
	.platform_disable = NULL,
	.dev = {
		.platform_data = &latona_dss_lcd_data,
	},
};

static struct omap_dss_device *latona_dss_devices[] = {
	&latona_lcd_device,
	&latona_tv_device,
};

static struct omap_dss_board_info latona_dss_data = {
	.num_devices = ARRAY_SIZE(latona_dss_devices),
	.devices = latona_dss_devices,
	.default_device = &latona_lcd_device,
};

static struct omap2_mcspi_device_config dss_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info nt35510_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "nt35510_spi",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 375000,
		.controller_data        = &dss_lcd_mcspi_config,
	},
};

void __init latona_display_init(enum omap_dss_venc_type venc_type)
{
	omap_display_init(&latona_dss_data);
	spi_register_board_info(nt35510_spi_board_info,
				ARRAY_SIZE(nt35510_spi_board_info));
}

