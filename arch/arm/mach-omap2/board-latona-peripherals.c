/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom-peripherals.c for Samsung latona board.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <linux/switch.h>

#ifdef CONFIG_LEDS_OMAP_DISPLAY
#include <linux/leds.h>
#include <linux/leds-omap-display.h>
#endif

#include <mach/board-latona.h>

#include "mux.h"
#include "hsmmc.h"
#include "twl4030.h"
#include <linux/wakelock.h>

/* Atmel Touchscreen */
#define OMAP_GPIO_TSP_INT 142

#include <media/v4l2-int-device.h>

#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
#include <media/imx046.h>
extern struct imx046_platform_data latona_imx046_platform_data;
#endif

#ifdef CONFIG_VIDEO_OMAP3
extern void latona_cam_init(void);
#else
#define latona_cam_init()	NULL
#endif

#if (defined(CONFIG_VIDEO_LV8093) || defined(CONFIG_VIDEO_LV8093_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
#include <media/lv8093.h>
extern struct imx046_platform_data latona_lv8093_platform_data;
#define LV8093_PS_GPIO		7
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF		1
#define LV8093_PWR_ON		(!LV8093_PWR_OFF)
#endif

#ifdef CONFIG_LEDS_OMAP_DISPLAY
/* PWM output/clock enable for LCD backlight*/
#define REG_INTBR_GPBR1				0xc
#define REG_INTBR_GPBR1_PWM1_OUT_EN		(0x1 << 3)
#define REG_INTBR_GPBR1_PWM1_OUT_EN_MASK	(0x1 << 3)
#define REG_INTBR_GPBR1_PWM1_CLK_EN		(0x1 << 1)
#define REG_INTBR_GPBR1_PWM1_CLK_EN_MASK	(0x1 << 1)

/* pin mux for LCD backlight*/
#define REG_INTBR_PMBR1				0xd
#define REG_INTBR_PMBR1_PWM1_PIN_EN		(0x3 << 4)
#define REG_INTBR_PMBR1_PWM1_PIN_MASK		(0x3 << 4)

#define MAX_CYCLES				0x7f
#define MIN_CYCLES				75
#define LCD_PANEL_BACKLIGHT_GPIO		(7 + OMAP_MAX_GPIO_LINES)
#endif

#define BLUETOOTH_UART	UART2

static struct wake_lock uart_lock;

/* Latona Volume up and Volume down configs from Samsung Kernel */
static int board_keymap[] = {
	KEY(2, 1, KEY_VOLUMEUP),
	KEY(1, 1, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data latona_kp_twl4030_data = {
	.keymap_data = &board_map_data,
	.rows = 5,
	.cols = 6,
	.rep = 0,
};

static struct resource board_power_key_resources[] = {
	[0] = {
	       // PWRON KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	       },
	[1] = {
	       // HOME KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	       },

};

static struct platform_device board_power_key_device = {
	.name = "power_key_device",
	.id = -1,
	.num_resources = ARRAY_SIZE(board_power_key_resources),
	.resource = board_power_key_resources,
};


static struct __initdata twl4030_power_data latona_t2scripts_data;
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
static struct resource board_ear_key_resource = {
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};
static struct platform_device board_ear_key_device = {
	.name = "sec_jack",
	.id = -1,
	.num_resources = 1,
	.resource = &board_ear_key_resource,
};
#endif
static struct regulator_consumer_supply latona_vdda_dac_supply = {
	.supply = "vdda_dac",
	
};
static struct regulator_consumer_supply latona_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply latona_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply latona_vmmc2_supply = {
	.supply		= "vmmc",
};
static struct regulator_consumer_supply latona_vaux1_supply = {
	.supply = "vaux1",
};

static struct regulator_consumer_supply latona_vaux2_supply = {
	.supply = "vaux2",
};

static struct regulator_consumer_supply latona_vaux3_supply = {
	.supply = "vaux3",
};

static struct regulator_consumer_supply latona_vaux4_supply = {
	.supply = "vaux4",
};

static struct regulator_consumer_supply latona_vpll2_supply = {
	.supply = "vpll2",
};
struct regulator_init_data latona_vdac = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &latona_vdda_dac_supply,
};
/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data latona_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &latona_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data latona_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &latona_vmmc2_supply,
};


/* VAUX1 for PL_SENSOR */
static struct regulator_init_data latona_aux1 = {
	.constraints = {
			.min_uV = 3000000,
			.max_uV = 3000000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &latona_vaux1_supply,
};

/* VAUX2 for PL_SENSOR */
static struct regulator_init_data latona_aux2 = {
	.constraints = {
			.min_uV = 2800000,
			.max_uV = 2800000,
			.apply_uV = true,
    #if 1  //TI patch to avoid leakage current in U404 chip(AP_SCL voltage correction for sensor )
			.always_on = true,
    #endif 
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &latona_vaux2_supply,
};


/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data latona_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &latona_vsim_supply,
};

static struct gpio_switch_platform_data headset_switch_data = {
	.name = "h2w",
	.gpio = OMAP_GPIO_DET_3_5,	/* Omap3430 GPIO_27 For samsung zeus */
};

static struct platform_device headset_switch_device = {
	.name = "switch-gpio",
	.dev = {
		.platform_data = &headset_switch_data,
		}
};

#ifdef CONFIG_LEDS_OMAP_DISPLAY
/* omap3 led display */
static void latona_pwm_config(u8 brightness)
{

	u8 pwm_off = 0;

	pwm_off = (MIN_CYCLES * (LED_FULL - brightness) +
		   MAX_CYCLES * (brightness - LED_OFF)) /
		(LED_FULL - LED_OFF);

	pwm_off = clamp(pwm_off, (u8)MIN_CYCLES, (u8)MAX_CYCLES);

	printk(KERN_DEBUG "PWM Duty cycles = %d\n", pwm_off);

	/* start at 0 */
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, 0, 0);
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, pwm_off, 1);
}

static void latona_pwm_enable(int enable)
{
	u8 gpbr1;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &gpbr1, REG_INTBR_GPBR1);
	gpbr1 &= ~REG_INTBR_GPBR1_PWM1_OUT_EN_MASK;
	gpbr1 |= (enable ? REG_INTBR_GPBR1_PWM1_OUT_EN : 0);
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, gpbr1, REG_INTBR_GPBR1);

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &gpbr1, REG_INTBR_GPBR1);
	gpbr1 &= ~REG_INTBR_GPBR1_PWM1_CLK_EN_MASK;
	gpbr1 |= (enable ? REG_INTBR_GPBR1_PWM1_CLK_EN : 0);
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, gpbr1, REG_INTBR_GPBR1);
}

void omap_set_primary_brightness(u8 brightness)
{
	u8 pmbr1;
	static int latona_pwm1_config;
	static int latona_pwm1_output_enabled;

	if (latona_pwm1_config == 0) {
		twl_i2c_read_u8(TWL4030_MODULE_INTBR, &pmbr1, REG_INTBR_PMBR1);

		pmbr1 &= ~REG_INTBR_PMBR1_PWM1_PIN_MASK;
		pmbr1 |=  REG_INTBR_PMBR1_PWM1_PIN_EN;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR, pmbr1, REG_INTBR_PMBR1);

		latona_pwm1_config = 1;
	}

	if (!brightness) {
		latona_pwm_enable(0);
		latona_pwm1_output_enabled = 0;
		return;
	}

	latona_pwm_config(brightness);
	if (latona_pwm1_output_enabled == 0) {
		latona_pwm_enable(1);
		latona_pwm1_output_enabled = 1;
	}

	printk(KERN_DEBUG "Latona LCD Backlight brightness = %d\n", brightness);
}

static struct omap_disp_led_platform_data omap_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.primary_display_set = omap_set_primary_brightness,
	.secondary_display_set = NULL,
};

static struct platform_device omap_disp_led = {
	.name   =       "display_led",
	.id     =       -1,
	.dev    = {
		.platform_data = &omap_disp_led_data,
	},
};
/* end led Display */
#endif

static struct platform_device *latona_board_devices[] __initdata = {
	&headset_switch_device,
#ifdef CONFIG_LEDS_OMAP_DISPLAY
	&omap_disp_led,
#endif
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
	&board_ear_key_device,
#endif
	&board_power_key_device,
};

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static struct regulator_consumer_supply latona_vpll2_supply = {
	.supply         = "vdds_dsi",
};

static struct regulator_consumer_supply latona_vdda_dac_supply = {
	.supply         = "vdda_dac",
};

static struct regulator_init_data latona_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &latona_vpll2_supply,
};

static struct regulator_init_data latona_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &latona_vdda_dac_supply,
};

static int latona_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	/* The controller that is connected to the 128x device
	 * should have the card detect gpio disabled. This is
	 * achieved by initializing it with a negative value
	 */
	mmc[CONFIG_TIWLAN_MMC_CONTROLLER - 1].gpio_cd = -EINVAL;
#endif

	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	latona_vmmc1_supply.dev = mmc[0].dev;
	latona_vsim_supply.dev = mmc[0].dev;
	latona_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

/* EXTMUTE callback function */
static void latona_set_hs_extmute(int mute)
{
	gpio_set_value(LATONA_HEADSET_EXTMUTE_GPIO, mute);
}

static int latona_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
4040,  3910,  3790,  3670,  3550
};

static struct twl4030_bci_platform_data latona_bci_data = {
	.battery_tmp_tbl	= latona_batt_table,
	.tblsize		= ARRAY_SIZE(latona_batt_table),
};

static struct twl4030_usb_data latona_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data latona_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= latona_twl_gpio_setup,
	.debounce	= 0x04,
};

static struct twl4030_madc_platform_data latona_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data latona_audio_data = {
	.audio_mclk	= 26000000,
	.ramp_delay_value = 3, /* 161 ms */
	.hs_extmute	= 1,
	.set_hs_extmute	= latona_set_hs_extmute,
};

static struct twl4030_codec_data latona_codec_data = {
	.audio_mclk = 26000000,
	.audio = &latona_audio_data,
};

static struct twl4030_platform_data latona_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &latona_bci_data,
	.madc		= &latona_madc_data,
	.usb		= &latona_usb_data,
	.gpio		= &latona_gpio_data,
	.keypad		= &latona_kp_twl4030_data,
	.power		= &latona_t2scripts_data,
	.codec		= &latona_codec_data,
	.vmmc1      = &latona_vmmc1,
	.vmmc2      = &latona_vmmc2,
	.vsim       = &latona_vsim,
    .vaux1      = &latona_aux1,
	.vaux2      = &latona_aux2,
	.vaux3      = &latona_aux3,
	.vaux4      = &latona_aux4,
	.vpll2		= &latona_vpll2,
	.vdac		= &latona_vdac,

};

static struct i2c_board_info __initdata latona_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl5030", 0x48),
		.flags		= I2C_CLIENT_WAKE,
		.irq		= INT_34XX_SYS_NIRQ,
		.platform_data	= &latona_twldata,
	},
};

#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
static inline void __init board_init_ear_key(void)
{
	board_ear_key_resource.start = gpio_to_irq(OMAP_GPIO_EAR_SEND_END);
	if (gpio_request(OMAP_GPIO_EAR_SEND_END, "ear_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for POWER KEY IRQ \n",
		       OMAP_GPIO_EAR_SEND_END);
		return;
	}
	gpio_direction_input(OMAP_GPIO_EAR_SEND_END);
}
#endif

static inline void __init board_init_power_key(void)
{
	board_power_key_resources[0].start = gpio_to_irq(OMAP_GPIO_KEY_PWRON);
	if (gpio_request(OMAP_GPIO_KEY_PWRON, "power_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for POWER KEY IRQ \n",
		       OMAP_GPIO_KEY_PWRON);
		return;
	}
	board_power_key_resources[1].start = gpio_to_irq(OMAP_GPIO_KEY_HOME);
	if (gpio_request(OMAP_GPIO_KEY_HOME, "home_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for VOLDN KEY IRQ \n",
		       OMAP_GPIO_KEY_HOME);
		return;
	}
	gpio_direction_input(OMAP_GPIO_KEY_PWRON);
	gpio_direction_input(OMAP_GPIO_KEY_HOME);
}


static void atmel_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	if (gpio_request(OMAP_GPIO_TSP_INT, "touch_atmel") < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_GPIO_TSP_INT);
	
}

static struct i2c_board_info __initdata latona_i2c_bus2_info[] = {
#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO(IMX046_NAME, IMX046_I2C_ADDR),
		.platform_data = &latona_imx046_platform_data,
	},
#endif
#if (defined(CONFIG_VIDEO_LV8093) || defined(CONFIG_VIDEO_LV8093_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &latona_lv8093_platform_data,
	},
#endif
};

static int __init omap_i2c_init(void)
{
	/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;

		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 100, NULL, latona_i2c_boardinfo,
			ARRAY_SIZE(latona_i2c_boardinfo));
	omap_register_i2c_bus(2, 100, NULL, latona_i2c_bus2_info,
			ARRAY_SIZE(latona_i2c_bus2_info));
	omap_register_i2c_bus(3, 400, NULL, latona_i2c_bus3_info,
			ARRAY_SIZE(latona_i2c_bus3_info));
	return 0;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void plat_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;

	/* Specific wakelock for bluetooth usecases */
	if ((up2->pdev->id == BLUETOOTH_UART)
			&& ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
		wake_lock_timeout(&uart_lock, 2*HZ);
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = plat_hold_wakelock,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
	},
	{
		.use_dma	= 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout	= DEFAULT_IDLE_TIMEOUT,
		.flags		= 1,
		.plat_hold_wakelock = NULL,
	},
	{
		.flags		= 0
	}
};

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

void __init latona_peripherals_init(void)
{
	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");

	twl4030_get_scripts(&latona_t2scripts_data);
	omap_i2c_init();
	platform_add_devices(latona_board_devices,
		ARRAY_SIZE(latona_board_devices));
	atmel_dev_init();
	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);
	board_init_power_key();
	enable_board_wakeup_source();
	latona_cam_init();
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
	board_init_ear_key();
#endif
}
