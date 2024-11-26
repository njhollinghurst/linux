// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIO code for RP1 DPI.
 *
 * In progressive modes: if either the DRM_MODE_FLAG_CSYNC flag is set
 * or the "csync-gpio" property refers to an RP1 GPIO, PIO will generate
 * CSYNC alongside existing VSYNC and HSYNC signals, which must be mapped
 * as outputs. If "csync-gpio" is not specified, CSYNC defaults to GPIO 1.
 * CSYNC polarity will be negative unless DRM_MODE_FLAG_PCSYNC is set.
 *
 * In interlaced modes: if either the DRM_MODE_FLAG_CSYNC flag is set
 * or the "csync-gpio" property refers to an RP1 GPIO, PIO will generate
 * CSYNC alongside broken/modified versions of VSYNC and HSYNC, which
 * must be mapped as outputs and ignored (this is a DPI limitation).
 *
 * Note that fixed-up CSYNC outputs will not be synchronous to DPICLOCK
 * and is subject to +/-5ns of jitter. For this reason, CSync is usable
 * only at SDTV rates.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/pio_rp1.h>
#include <drm/drm_print.h>

#include "rp1_dpi.h"

/*
 * COMPOSITE SYNC FOR PROGRESSIVE
 *
 * Copies HSYNC pulses to CSYNC (adding 1 cycle); then when VSYNC
 * is asserted, extend each pulse by an additional Y + 1 cycles.
 *
 * The following time constant should be written to the FIFO:
 *    (htotal - 2 * hsync_width) * sys_clock / dpi_clock - 2.
 *
 * The default configuration is +HSync, +VSync, -CSync; other
 * polarities can be made by modifying the PIO program code.
 */

static int rp1dpi_pio_csync_prog(struct rp1_dpi *dpi,
				 struct drm_display_mode const *mode)
{
	unsigned i, tc, offset;
	u16 instructions[] = {  /* This is mutable */
		0x90a0, //  0: pull   block           side 1
		0x7040, //  1: out    y, 32           side 1
		//     .wrap_target
		0xb222, //  2: mov    x, y            side 1 [2]
		0x3083, //  3: wait   1 gpio, 3       side 1
		0xa322, //  4: mov    x, y            side 0 [3]
		0x2003, //  5: wait   0 gpio, 3       side 0
		0x00c7, //  6: jmp    pin, 7          side 0
		//     .wrap
		0x0047, //  7: jmp    x--, 7          side 0
		0x1002, //  8: jmp    2               side 1
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	int sm = pio_claim_unused_sm(dpi->pio, true);

	if (sm < 0)
		return -EBUSY;

	/* Adapt program code for sync polarity; configure program */
	pio_sm_set_enabled(dpi->pio, sm, false);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC) {
		instructions[6] = 0x00c2; /* jmp pin, 2 side 0 */
	}
	if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
		instructions[3] ^= 0x80;
		instructions[5] ^= 0x80;
	}
	if (mode->flags & DRM_MODE_FLAG_PCSYNC) {
		for(i = 0; i < ARRAY_SIZE(instructions); i++)
			instructions[i] ^= 0x1000;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + 2,
			   offset + ((mode->flags & DRM_MODE_FLAG_NVSYNC) ? 7 : 6));
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* VSync on GPIO 2 */

	pio_sm_init(dpi->pio, sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, sm, i, 1, true);

	/* Place time constant into the FIFO; start the SM */
	tc = mode->htotal - 2*(mode->hsync_end - mode->hsync_start);
	tc = (tc * (u64)clock_get_hz(clk_sys)) / (u64)(mode->clock * 1000u);
	pio_sm_put(dpi->pio, sm, tc - 2);
	pio_sm_set_enabled(dpi->pio, sm, true);

	return 0;
}

static int rp1dpi_pio_start_timers(struct rp1_dpi *dpi, u32 flags, int num, u32 const tc[])
{
	static const u16 instructions[2][4] = {
		{ 0xa022, 0x2083, 0x0042, 0xc010 }, /* posedge */
		{ 0xa022, 0x2003, 0x0042, 0xc010 }, /* negedge */
	};
	const struct pio_program prog = {
		.instructions = instructions[(flags & DRM_MODE_FLAG_NHSYNC) ? 1 : 0],
		.length = ARRAY_SIZE(instructions[0]),
		.origin = -1
	};
	int offset, i;

	i = pio_claim_sm_mask(dpi->pio, (2 << num) - 2); /* Claim [num] SMs, starting from #1 */
	if (i != 0)
		return -EBUSY;

	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	for(i = 0; i < num; i++) {
		pio_sm_config cfg = pio_get_default_sm_config();

		pio_sm_set_enabled(dpi->pio, i + 1, false);
		sm_config_set_wrap(&cfg, offset, offset + 3);
		pio_sm_init(dpi->pio, i + 1, offset, &cfg);

		pio_sm_put(dpi->pio, i + 1, tc[i] - 4);
		pio_sm_exec(dpi->pio, i + 1, pio_encode_pull(false, false));
		pio_sm_exec(dpi->pio, i + 1, pio_encode_out(pio_y, 32));
		pio_sm_set_enabled(dpi->pio, i + 1, true);
	}

	return 0;
}


/*
 * COMPOSITE SYNC FOR INTERLACED
 *
 * DPI VSYNC (GPIO2) must be a modified signal which is always active-low.
 * It should go low for 1 or 2 scanlines, 2.5 or 3 lines before Vsync-start
 * (in the case of 525/60i it should be 3 or 3.5 lines before VSync-start).
 * This is to allow time to generate "equalizing pulses" if required.
 *
 * Three PIO SMs will be configured as timers, to fire at the end of a left
 * broad pulse, the middle of a scanline, and the end of a right broad pulse.
 *
 * The remaining SM should have its Y register set to equalizing pulse width
 * minus 3 cycles, and its ISR register loaded with one of the following:
 * For 625/50i: 0000_0010_1010_1011_1111_1111_1010_1010 (0x02ABFFAA)
 * For 525/60i: 1010_1010_1011_1111_1111_1110_1010_1010 (0xAABFFEAA)
 *
 * HSYNC->CSYNC latency is about 4 cycles, with a jitter of up to 1 cycle.
 * To minimize jitter, PIO clock should be a multiple of twice the line rate.
 *
 * Default program is compiled for +HSync, -CSync. The program may be
 * modified for other polarities and to remove mid-line equalizing pulses.
 */

static int rp1dpi_pio_csync_ilace(struct rp1_dpi *dpi,
				  struct drm_display_mode const *mode)
{
	static const int wrap_target = 4;
	static const int wrap = 27;
	u16 instructions[] = {  /* This is mutable */
		0x3083, //  0: wait   1 gpio, 3       side 1
		0xa3e6, //  1: mov    osr, isr        side 0 [3]
		0x2003, //  2: wait   0 gpio, 3       side 0
		0x12c0, //  3: jmp    pin, 0          side 1 [2]
		//     .wrap_target
		0xd042, //  4: irq    clear 2         side 1
		0xd043, //  5: irq    clear 3         side 1
		0x7021, //  6: out    x, 1            side 1
		0x102b, //  7: jmp    !x, 11          side 1
		0x30c2, //  8: wait   1 irq, 2        side 1
		0x20c3, //  9: wait   1 irq, 3        side 0
		0x100e, // 10: jmp    14              side 1
		0x30c2, // 11: wait   1 irq, 2        side 1
		0xa122, // 12: mov    x, y            side 0 [1]
		0x004d, // 13: jmp    x--, 13         side 0
		0x7021, // 14: out    x, 1            side 1
		0x1020, // 15: jmp    !x, 0           side 1
		0xd041, // 16: irq    clear 1         side 1
		0xb022, // 17: mov    x, y            side 1
		0x3083, // 18: wait   1 gpio, 3       side 1
		0x0053, // 19: jmp    x--, 19         side 0
		0x6021, // 20: out    x, 1            side 0
		0x0037, // 21: jmp    !x, 23          side 0
		0x20c1, // 22: wait   1 irq, 1        side 0
		0x7021, // 23: out    x, 1            side 1
		0x1020, // 24: jmp    !x, 0           side 1
		0x10c4, // 25: jmp    pin, 4          side 1
		0xb0e6, // 26: mov    osr, isr        side 1
		0x7022, // 27: out    x, 2            side 1
		//     .wrap
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	u32 sysclk_khz = clock_get_hz(clk_sys) / 1000u;
	unsigned i, offset;
	u32 tc[3], tc_eq, magic;
	int sm = pio_claim_sm_mask(dpi->pio, 1);
	if (sm != 0)
		return -EBUSY;

	/* Compute mid-line and broad-sync time constants and start the 3 "timer" SMs */
	tc_eq = ((mode->hsync_end - mode->hsync_start) * sysclk_khz) / mode->clock;
	tc[1] = (mode->htotal * sysclk_khz) / (2 * mode->clock);
	tc[0] = tc[1] - tc_eq;
	tc[2] = tc[0] + tc[1];
	if (rp1dpi_pio_start_timers(dpi, mode->flags, 3, tc) < 0) {
		pio_sm_unclaim(dpi->pio, sm);
		return -EBUSY;
	}

	/* Configure VSync sequence and equalizing pulses (for SDTV modes only) */
	if (16 * mode->htotal <= mode->clock || 15 * mode->htotal >= mode->clock ||
	    ((mode->vtotal >> 1) != 262 && (mode->vtotal >> 1) != 312)) {
		magic = (0x80u << (2*(mode->vsync_end - mode->vsync_start))) - 0x56u;
		instructions[12] |= 0x1000; /* kill misaligned EQ pulses */
		instructions[13] |= 0x1000;
	} else if ((mode->vtotal >> 1) == 262) {
		tc_eq = sysclk_khz / 431u;  /* 2.32 us */
		magic = 0xAABFFEAA;         /* 5-6 short, 6 broad, 6 short */
	} else {
		tc_eq = sysclk_khz / 425u;  /* 2.35 us */
		magic = 0x02ABFFAA;         /* 4-5 short, 5 broad, 5 short */
	}

	/* Adapt program code according to CSync polarity; configure program */
	pio_sm_set_enabled(dpi->pio, sm, false);
	for(i = 0; i < ARRAY_SIZE(instructions); i++) {
		if (mode->flags & DRM_MODE_FLAG_PCSYNC)
			instructions[i] ^= 0x1000;
		if ((mode->flags & DRM_MODE_FLAG_NHSYNC) && (instructions[i] & 0xef7f) == 0x2003)
			instructions[i] ^= 0x0080;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	/* Configure pins and SM */
	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* "VSync helper" signal is always GPIO2 */
	pio_sm_init(dpi->pio, sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, sm, i, 1, true);

	/* Load constants into the ISR and Y registers; start the SM */
	pio_sm_put(dpi->pio, sm, magic);
	pio_sm_put(dpi->pio, sm, (tc_eq >= 3) ? (tc_eq - 3) : 0);
	pio_sm_exec(dpi->pio, sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, sm, pio_encode_out(pio_y, 32));
	pio_sm_exec(dpi->pio, sm, pio_encode_in(pio_y, 32));
	pio_sm_exec(dpi->pio, sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, sm, pio_encode_out(pio_y, 32));
	pio_sm_set_enabled(dpi->pio, sm, true);

	return 0;
}

int rp1dpi_pio_probe(struct rp1_dpi *dpi, struct device_node *np)
{
	struct of_phandle_args of_args = { 0 };

	/* Find which GPIO (if any) was requested */
	dpi->csync_gpio = -1;
	if (np && !of_parse_phandle_with_args(np, "csync-gpio", "#gpio-cells", 0, &of_args)) {
		bool is_rp1 = of_device_is_compatible(of_args.np, "raspberrypi,rp1-gpio");
		if (is_rp1 && of_args.args_count == 2) {
			dpi->csync_gpio = of_args.args[0];
		}
		of_node_put(of_args.np);
	}

	/* Always succeed, since the CSync pin is optional */
	return 0;
}

int rp1dpi_pio_start(struct rp1_dpi *dpi, struct drm_display_mode const * mode)
{
	int r;

	if (dpi->csync_gpio < 0 && !(mode->flags & DRM_MODE_FLAG_CSYNC))
		return 0;

	if (dpi->pio)
		pio_close(dpi->pio);

	dpi->pio = pio_open();
	if (IS_ERR(dpi->pio)) {
		drm_err(&dpi->drm, "Could not open PIO\n");
		dpi->pio = NULL;
		return -ENODEV;
	}

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		r = rp1dpi_pio_csync_ilace(dpi, mode);
	else
		r = rp1dpi_pio_csync_prog(dpi, mode);

	if (!r) {
		drm_info(&dpi->drm, "Start PIO to generate %cCSync on GPIO %d\n",
			 (mode->flags & DRM_MODE_FLAG_PCSYNC) ? '+' : '-',
			 (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1);
	} else {
		drm_err(&dpi->drm, "Failed to initialize PIO\n");
		rp1dpi_pio_stop(dpi);
	}

	return r;
}

void rp1dpi_pio_stop(struct rp1_dpi *dpi)
{
	if (dpi->pio) {
		pio_close(dpi->pio);
		dpi->pio = NULL;
	}
}
