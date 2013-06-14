/*
 * Defines for latona board
 */
#include <plat/mux_latona_rev_r08.h>
#include <plat/display.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include "board-latona-mtd.h"

struct flash_partitions {
	struct mtd_partition *parts;
	int nr_parts;
};

extern void __init latona_peripherals_init(void);
extern void __init latona_display_init(enum omap_dss_venc_type venc_type);
extern void __init latona_onenand_init(void);

#define LATONA_HEADSET_EXTMUTE_GPIO	153
#define LATONA_NAND_CS    0
