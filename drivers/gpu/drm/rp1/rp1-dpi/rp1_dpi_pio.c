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
		0xb322, //  2: mov    x, y            side 1 [3]
		0x3083, //  3: wait   1 gpio, 3       side 1
		0xa422, //  4: mov    x, y            side 0 [4]
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

		pio_sm_put(dpi->pio, i + 1, tc[i] - 3);
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
 * It should go low for 1 or 2 scanlines, 1 or 1.5 lines before Vsync-start.
 * Desired VSync width minus 1 (in half-lines) should be written to the FIFO.
 *
 * Three PIO SMs will be configured as timers, to fire at the end of a left
 * broad pulse, the middle of a scanline, and the end of a right broad pulse.
 *
 * HSYNC->CSYNC latency is about 4 cycles, with a jitter of up to 1 cycle.
 * To minimize jitter, PIO clock should be a multiple of twice the line rate.
 *
 * Default program is compiled for +HSync, -CSync. The program may be
 * modified for other polarities. GPIO2 polarity is always active low.
 */

static int rp1dpi_pio_csync_ilace(struct rp1_dpi *dpi,
				  struct drm_display_mode const *mode)
{
	static const int wrap_target = 2;
	static const int wrap = 20;
	u16 instructions[] = {  /* This is mutable */
		0x90a0, //  0: pull   block           side 1
		0x7040, //  1: out    y, 32           side 1
		//     .wrap_target
		0x3083, //  2: wait   1 gpio, 3       side 1
		0xa442, //  3: nop                    side 0 [4]
		0x2003, //  4: wait   0 gpio, 3       side 0
		0x13c2, //  5: jmp    pin, 2          side 1 [3]
		0x3083, //  6: wait   1 gpio, 3       side 1
		0xa322, //  7: mov    x, y            side 0 [3]
		0xc041, //  8: irq    clear 1         side 0
		0x2003, //  9: wait   0 gpio, 3       side 0
		0x00d3, // 10: jmp    pin, 19         side 0
		0xd042, // 11: irq    clear 2         side 1
		0xd043, // 12: irq    clear 3         side 1
		0x30c2, // 13: wait   1 irq, 2        side 1
		0x20c3, // 14: wait   1 irq, 3        side 0
		0x1051, // 15: jmp    x--, 17         side 1
		0x1002, // 16: jmp    2               side 1
		0xd041, // 17: irq    clear 1         side 1
		0x3083, // 18: wait   1 gpio, 3       side 1
		0x20c1, // 19: wait   1 irq, 1        side 0
		0x104b, // 20: jmp    x--, 11         side 1
		//     .wrap
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned i, offset;
	u32 tc[3];
	u32 sysclk = clock_get_hz(clk_sys);
	int sm = pio_claim_sm_mask(dpi->pio, 1);
	if (sm != 0)
		return -EBUSY;

	/* Compute mid-line and broad-sync time constants and start the 3 "timer" SMs */
	tc[1] = (mode->htotal * (u64)sysclk) / (u64)(2000u * mode->clock);
	tc[0] = tc[1] - ((mode->hsync_end - mode->hsync_start) * (u64)sysclk) /
		(u64)(1000u * mode->clock);
	tc[2] = tc[0] + tc[1];
	if (rp1dpi_pio_start_timers(dpi, mode->flags, 3, tc) < 0) {
		pio_sm_unclaim(dpi->pio, sm);
		return -EBUSY;
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

	/* Configure pins and SM; set VSync width; start the SM */
	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* "VSync helper" signal is always GPIO2 */
	pio_sm_init(dpi->pio, sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, sm, i, 1, true);
	pio_sm_put(dpi->pio, sm, mode->vsync_end - mode->vsync_start - 1);
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
