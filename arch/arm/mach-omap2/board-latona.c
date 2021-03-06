/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/delay.h>
#include <asm/setup.h>
#include <asm/sizes.h>

#include <mach/board-latona.h>

#include <plat/common.h>
#include <plat/control.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/opp_twl_tps.h>
#include <plat/timer-gp.h>

#include "mux.h"
#include "sdram-qimonda-hyb18m512160af-6.h"
#include "smartreflex-class1p5.h"
#include "board-latona-wifi.h"
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

#define LATONA_McBSP3_BT_GPIO 164

#ifdef CONFIG_OMAP_MUX
extern struct omap_board_mux *latona_board_mux_ptr;
extern struct omap_board_mux *latona_board_wk_mux_ptr;
#else
#define latona_board_mux_ptr		NULL
#define latona_board_wk_mux_ptr		NULL
#endif

#ifdef CONFIG_PM
static struct omap_volt_vc_data vc_config = {
	/* MPU */
	.vdd0_on	= 1200000, /* 1.2v */
	.vdd0_onlp	= 1000000, /* 1.0v */
	.vdd0_ret	=  975000, /* 0.975v */
	.vdd0_off	=  600000, /* 0.6v */
	/* CORE */
	.vdd1_on	= 1150000, /* 1.15v */
	.vdd1_onlp	= 1000000, /* 1.0v */
	.vdd1_ret	=  975000, /* 0.975v */
	.vdd1_off	=  600000, /* 0.6v */

	.clksetup	= 0xff,
	.voltoffset	= 0xff,
	.voltsetup2	= 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
};

#ifdef CONFIG_TWL4030_CORE
static struct omap_volt_pmic_info omap_pmic_mpu = { /* and iva */
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x0, /* (vdd0) VDD1 -> VDD1_CORE -> VDD_MPU */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x14,
	.vp_vlimitto_vddmax = 0x44,
};

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x1, /* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x42,
};
#endif /* CONFIG_TWL4030_CORE */
#endif /* CONFIG_PM */

static void __init omap_latona_map_io(void)
{
	omap2_set_globals_36xx();
	omap34xx_map_common_io();
}

static struct omap_board_config_kernel latona_config[] __initdata = {
};

static void __init omap_latona_init_irq(void)
{
	omap_board_config = latona_config;
	omap_board_config_size = ARRAY_SIZE(latona_config);
	omap2_init_common_devices(hyb18m512160af6_sdrc_params,
		hyb18m512160af6_sdrc_params);
	omap2_gp_clockevent_set_gptimer(1);
	omap_init_irq();
}

static const struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1]		= OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2]		= OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset		= true,
	.reset_gpio_port[0]	= -EINVAL,
	.reset_gpio_port[1]	= 64,
	.reset_gpio_port[2]	= -EINVAL,
};
int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl128x BT, FM, GPS connectivity chip */
static int gpios[] = {109, 161, -1};

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 109,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};
static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *latona_devices[] __initdata = {

	&wl127x_device,
	&btwilink_device,
};

/* OPP MPU/IVA Clock Frequency */
struct opp_frequencies {
	unsigned long mpu;
	unsigned long iva;
	unsigned long ena;
};

static struct opp_frequencies opp_freq_add_table[] __initdata = {
  {
	.mpu = 800000000,
	.iva = 660000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP120_VDD1,
  },
  {
	.mpu = 1000000000,
	.iva =  800000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP1G_VDD1,
  },
  {
	.mpu = 1200000000,
	.iva =   65000000,
	.ena = OMAP3630_CONTROL_FUSE_OPP1_2G_VDD1,
  },

  { 0, 0, 0 },
};


/* Fix to prevent VIO leakage on wl127x */
static int wl127x_vio_leakage_fix(void)
{
	int ret = 0;

	pr_info(" wl127x_vio_leakage_fix\n");

	ret = gpio_request(gpios[0], "wl127x_bten");
	if (ret < 0) {
		pr_err("wl127x_bten gpio_%d request fail",
			gpios[0]);
		goto fail;
	}

	gpio_direction_output(gpios[0], 1);
	mdelay(10);
	gpio_direction_output(gpios[0], 0);
	udelay(64);

	gpio_free(gpios[0]);
fail:
	return ret;
}



static void __init omap_latona_init(void)
{
	printk("\n");
	printk("-----------------------------------------------------------\n");
	printk("  Latona board init \n");
	printk("\n");
	printk("  Mux init...\n");
	omap3_mux_init(latona_board_mux_ptr, OMAP_PACKAGE_CBP);
	latona_mux_init_gpio_out();
	latona_mux_set_wakeup_gpio();
	config_wlan_mux();
	
	printk("  Peripherals init...\n");
	latona_peripherals_init();
	printk("  Onenand init...\n");
	latona_onenand_init();
	printk("  Latona Display and OMAP DSS init...\n");
	latona_display_init(OMAP_DSS_VENC_TYPE_COMPOSITE);

	omap_mux_init_gpio(LATONA_EHCI_RESET_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(LATONA_McBSP3_BT_GPIO, OMAP_PIN_OUTPUT);
	printk("  USB init...\n");
	usb_uhhtll_init(&usbhs_pdata);
	sr_class1p5_init();

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif
	printk("  Latona Devices init...\n");
	platform_add_devices(latona_devices, ARRAY_SIZE(latona_devices));
	wl127x_vio_leakage_fix();
	printk("  Latona board init done.\n");
	printk("-----------------------------------------------------------\n");
}

static void __init omap_latona_fixup(struct machine_desc *desc,
				    struct tag *tags, char **cmdline,
				    struct meminfo *mi)
{
	mi->bank[0].start = 0x80000000;
	mi->bank[0].size = 256 * SZ_1M;	/* DDR_CS0 256MB */
	mi->bank[0].node = 0;

	mi->bank[1].start = 0x90000000;
	mi->bank[1].size = 256 * SZ_1M;	/* DDR_CS1 256MB */
	mi->bank[1].node = 0;

	mi->nr_banks = 2;
}

/* must be called after omap2_common_pm_init() */
static int __init latona_opp_init(void)
{
	struct omap_hwmod *mh, *dh;
	struct omap_opp *mopp, *dopp;
	struct device *mdev, *ddev;
	struct opp_frequencies *opp_freq;
	unsigned long hw_support;


	if (!cpu_is_omap3630())
		return 0;

	mh = omap_hwmod_lookup("mpu");
	if (!mh || !mh->od) {
		pr_err("%s: no MPU hwmod device.\n", __func__);
		return 0;
	}

	dh = omap_hwmod_lookup("iva");
	if (!dh || !dh->od) {
		pr_err("%s: no DSP hwmod device.\n", __func__);
		return 0;
	}

	mdev = &mh->od->pdev.dev;
	ddev = &dh->od->pdev.dev;

	/* add MPU and IVA clock frequencies */
	for (opp_freq = opp_freq_add_table; opp_freq->mpu; opp_freq++) {
		/* check enable/disable status of MPU frequecy setting */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, false);
		hw_support = omap_ctrl_readl(opp_freq->ena);

		if (IS_ERR(mopp))
			mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		if (IS_ERR(mopp) || !hw_support) {
			pr_err("%s: MPU does not support %lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* check enable/disable status of IVA frequency setting */
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, false);
		if (IS_ERR(dopp))
			dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (IS_ERR(dopp)) {
			pr_err("%s: DSP does not support %lu MHz\n", __func__, opp_freq->iva / 1000000);
			continue;
		}

		/* try to enable MPU frequency setting */
		if (opp_enable(mopp)) {
			pr_err("%s: OPP cannot enable MPU:%lu MHz\n", __func__, opp_freq->mpu / 1000000);
			continue;
		}

		/* try to enable IVA frequency setting */
		if (opp_enable(dopp)) {
			pr_err("%s: OPP cannot enable DSP:%lu MHz\n", __func__, opp_freq->iva / 1000000);
			opp_disable(mopp);
			continue;
		}

		/* verify that MPU and IVA frequency settings are available */
		mopp = opp_find_freq_exact(mdev, opp_freq->mpu, true);
		dopp = opp_find_freq_exact(ddev, opp_freq->iva, true);
		if (!mopp || !dopp) {
			pr_err("%s: OPP requested MPU: %lu MHz and DSP: %lu MHz not found\n",
				__func__, opp_freq->mpu / 1000000, opp_freq->iva / 1000000);
			continue;
		}

		dev_info(mdev, "OPP enabled %lu MHz\n", opp_freq->mpu / 1000000);
		dev_info(ddev, "OPP enabled %lu MHz\n", opp_freq->iva / 1000000);
	}

	return 0;
}
device_initcall(latona_opp_init);

/*
* Android expects the bootloader to pass the device serial number and
* the name of the board as parameters on the kernel command line.
* Use the last 16 bytes of the DIE id as device serial number.
*/
#define DIE_ID_REG_BASE		(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET	0x218

#define SERIALNO_PARAM		"androidboot.serialno"
#define HARDWARE_PARAM		"androidboot.hardware=latona"
static int __init latona_cmdline_set_serialno(void)
{
	unsigned int val[2] = { 0 };
	unsigned int reg;

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;

	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 0x4);

	/*
	 * The final cmdline will have 16 digits, two spaces, an =, and a trailing
	 * \0 as well as the contents of saved_command_line, SERIALNO_PARAM
	 * and HARDWARE_PARAM
	*/
	size_t len = strlen(saved_command_line) + strlen(SERIALNO_PARAM)
				+ 20 + strlen(HARDWARE_PARAM);
	char *buf = kmalloc(len, GFP_ATOMIC);
	if (buf) {
		snprintf(buf, len, "%s %s=%08X%08X %s",
			saved_command_line,
			SERIALNO_PARAM,
			val[1], val[0],
			HARDWARE_PARAM);
		/* XXX This leaks strlen(saved_command_line) bytes of memory
		 * Do we care? */
		saved_command_line = buf;
	}

	return 0;
}
device_initcall(latona_cmdline_set_serialno);

MACHINE_START(LATONA, "SAMSUNG LATONA")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.fixup		= omap_latona_fixup,
	.map_io		= omap_latona_map_io,
	.init_irq	= omap_latona_init_irq,
	.init_machine	= omap_latona_init,
	.timer		= &omap_timer,
MACHINE_END
