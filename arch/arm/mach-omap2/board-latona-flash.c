/*
 * board-latona-flash.c
 *
 * Modified from mach-omap2/board-zoom-flash.c for Samsung latona board.
 * 
 * Copyright (C) 2009 Texas Instruments Inc.
 * Vimal Singh <vimalsingh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <mach/board-latona.h>

static struct omap_onenand_platform_data latona_onenand_data = {
  .cs    = 0,
  .gpio_irq  = 73,
  .parts    = onenand_partitions,
  .nr_parts  = ARRAY_SIZE(onenand_partitions),
  .flags    = ONENAND_SYNC_READWRITE,
};

static void __init latona_onenand_init(void)
{
  gpmc_onenand_init(&latona_onenand_data);
}

