// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIO code for Raspberry Pi RP1 DPI driver
 *
 * Copyright (c) 2024 Raspberry Pi Limited.
 */

/*
 * Use PIO to generate composite sync and fix up interlaced output.
 *
 * If the "csync-gpio" OF property is set, and refers to an RP1 GPIO,
 * PIO will generate CSYNC on that pin (which may not be GPIO 2 or 3).
 * CSYNC polarity will be negative unless DRM_MODE_FLAG_PCSYNC is set.
 * VSYNC will be mangled for interlaced modes and should be ignored.
 *
 * Otherwise, if DPI's "pinctrl-0" enables "gpio1" (DE):
 *  - If DRM_MODE_FLAG_CSYNC is set, DE is replaced by CSYNC as above;
 *  - Otherwise, VSYNC will be corrected for interlaced modes.
 *
 * (Interlaced modes will not work if neither CSYNC nor DE can be output.)
 * Note that CSYNC and corrected VSYNC outputs will not be synchronous to
 * DPICLOCK, will lag HSYNC by about 30ns and may suffer +/-5ns of jitter.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/pio_rp1.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <drm/drm_print.h>

#include "rp1_dpi.h"

/*
 * COMPOSITE SYNC FOR PROGRESSIVE
 *
 * Copy HSYNC pulses to CSYNC (adding 1 cycle); then when VSYNC
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
	unsigned int i, tc, offset;
	u16 instructions[] = {  /* This is mutable */
		0x90a0, //  0: pull   block      side 1
		0x7040, //  1: out    y, 32      side 1
		//     .wrap_target
		0xb322, //  2: mov    x, y       side 1 [3]
		0x3083, //  3: wait   1 gpio, 3  side 1
		0xa422, //  4: mov    x, y       side 0 [4]
		0x2003, //  5: wait   0 gpio, 3  side 0
		0x00c7, //  6: jmp    pin, 7     side 0    ; modify to flip VSync polarity
		//     .wrap                               ; modify to flip VSync polarity
		0x0047, //  7: jmp    x--, 7     side 0
		0x1002, //  8: jmp    2          side 1
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
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		instructions[6] = 0x00c2; /* jmp pin, 2 side 0 */
	if (mode->flags & DRM_MODE_FLAG_NHSYNC) {
		instructions[3] ^= 0x80;
		instructions[5] ^= 0x80;
	}
	if (mode->flags & DRM_MODE_FLAG_PCSYNC) {
		for (i = 0; i < ARRAY_SIZE(instructions); i++)
			instructions[i] ^= 0x1000;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	/* Configure pins and SM */
	dpi->pio_stole_gpio1 = (dpi->csync_gpio < 0);
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
	tc = mode->htotal - 2 * (mode->hsync_end - mode->hsync_start);
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

	for (i = 0; i < num; i++) {
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
		0x90a0, //  0: pull   block       side 1
		0x7040, //  1: out    y, 32       side 1
		//     .wrap_target                          ; while (true) {
		0x3083, //  2: wait   1 gpio, 3   side 1     ;   do { @HSync
		0xa442, //  3: nop                side 0 [4] ;     CSYNC short delay
		0x2003, //  4: wait   0 gpio, 3   side 0     ;     CSYNC HSync->CSync
		0x13c2, //  5: jmp    pin, 2      side 1 [3] ;   } while (VSync)
		0x3083, //  6: wait   1 gpio, 3   side 1     ;   @HSync
		0xa322, //  7: mov    x, y        side 0 [3] ;   CSYNC x = #broad pulses - 1
		0xc041, //  8: irq    clear 1     side 0     ;   CSYNC clear stale IRQs
		0x2003, //  9: wait   0 gpio, 3   side 0     ;   CSYNC HSync->CSync
		0x00d3, // 10: jmp    pin, 19     side 0     ;   CSYNC if (VSync) goto broad_left
		0xd042, // 11: irq    clear 2     side 1     ; right:
		0xd043, // 12: irq    clear 3     side 1     ;   clear stale IRQs
		0x30c2, // 13: wait   1 irq, 2    side 1     ;   @Midline
		0x20c3, // 14: wait   1 irq, 3    side 0     ;   CSYNC broad pulse
		0x1051, // 15: jmp    x--, 17     side 1     ;   if (x--) goto left
		0x1002, // 16: jmp    2           side 1     ;   continue;
		0xd041, // 17: irq    clear 1     side 1     ; left:
		0x3083, // 18: wait   1 gpio, 3   side 1     ;   @HSync
		0x20c1, // 19: wait   1 irq, 1    side 0     ;   broad_left: CSYNC broad pulse
		0x104b, // 20: jmp    x--, 11     side 1     ;   if (x--) goto right
		//     .wrap                                 ; }
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned int i, offset;
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
	for (i = 0; i < ARRAY_SIZE(instructions); i++) {
		if (mode->flags & DRM_MODE_FLAG_PCSYNC)
			instructions[i] ^= 0x1000;
		if ((mode->flags & DRM_MODE_FLAG_NHSYNC) && (instructions[i] & 0xe07f) == 0x2003)
			instructions[i] ^= 0x0080;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	/* Configure pins and SM; set VSync width; start the SM */
	dpi->pio_stole_gpio1 = (dpi->csync_gpio < 0);
	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* VSync "helper" signal is GPIO 2 */
	pio_sm_init(dpi->pio, sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, sm, i, 1, true);
	pio_sm_put(dpi->pio, sm, mode->vsync_end - mode->vsync_start - 1);
	pio_sm_set_enabled(dpi->pio, sm, true);

	return 0;
}

static int rp1dpi_pio_start_timer_both(struct rp1_dpi *dpi, u32 flags, u32 tc)
{
	static const u16 instructions[2][5] = {
		{ 0xa022, 0x2083, 0xc001, 0x0043, 0xc001 }, /* posedge */
		{ 0xa022, 0x2003, 0xc001, 0x0043, 0xc001 }, /* negedge */
	};
	const struct pio_program prog = {
		.instructions = instructions[(flags & DRM_MODE_FLAG_NHSYNC) ? 1 : 0],
		.length = ARRAY_SIZE(instructions[0]),
		.origin = -1
	};
	int offset, sm;

	sm = pio_claim_unused_sm(dpi->pio, true);
	if (sm < 0)
		return -EBUSY;

	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	pio_sm_config cfg = pio_get_default_sm_config();

	pio_sm_set_enabled(dpi->pio, sm, false);
	sm_config_set_wrap(&cfg, offset, offset + 4);
	pio_sm_init(dpi->pio, sm, offset, &cfg);

	pio_sm_put(dpi->pio, sm, tc - 4);
	pio_sm_exec(dpi->pio, sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, sm, pio_encode_out(pio_y, 32));
	pio_sm_set_enabled(dpi->pio, sm, true);

	return 0;
}

/*
 * VERTICAL SYNC FOR INTERLACE (WHERE CSYNC NOT USED)
 *
 * This time, because we are replacing VSYNC, we cannot also snoop on it!
 * Snoop on HSYNC, DE to count half-lines in the vertical blanking interval.
 *
 * At a suitable moment (which should be an odd number of half-lines since
 * the last active line), sample DE again to detect field phase.
 *
 * This version assumes VFP length is within 2..129 half-lines for any field
 * (one half-line delay is needed to sample DE; we always wait for the next
 * half-line boundary to improve VSync start accuracy).
 */

static int rp1dpi_pio_vsync_ilace(struct rp1_dpi *dpi,
				  struct drm_display_mode const *mode)
{
	static const int wrap_target = 14;
	static const int wrap = 26;
	u16 instructions[] = {  /* This is mutable */
		0xa0e6, //  0: mov    osr, isr    side 0     ; top: rewind parameters
		0x2081, //  1: wait   1 gpio, 1   side 0     ; main: while (!DE) wait;
		0x2783, //  2: wait   1 gpio, 3   side 0 [7] ;  do { @HSync
		0xc041, //  3: irq    clear 1     side 0     ;   flush stale IRQs
		0x20c1, //  4: wait   1 irq, 1    side 0     ;   @midline
		0x00c1, //  5: jmp    pin, 1      side 0     ;  } while (DE)
		0x0007, //  6: jmp    7           side 0     ;  <modify for -DE fixup>
		0x6027, //  7: out    x, 7        side 0     ;  x = VFPlen - 2
		0x000a, //  8: jmp    10          side 0     ;  while (x--) {
		0x20c1, //  9: wait   1 irq, 1    side 0     ;    @halfline
		0x0049, // 10: jmp    x--, 9      side 0     ;  }
		0x6021, // 11: out    x, 1        side 0     ;  test for aligned case
		0x003a, // 12: jmp    !x, 26      side 0     ;  if (!x) goto precise;
		0x20c1, // 13: wait   1 irq, 1    side 0     ;  @halfline
		//     .wrap_target                          ; vsjoin:
		0xb722, // 14: mov    x, y        side 1 [7] ;  VSYNC=1; x = VSyncLen
		0xd041, // 15: irq    clear 1     side 1     ;  VSYNC=1; flush stale IRQs
		0x30c1, // 16: wait   1 irq, 1    side 1     ;  VSYNC=1; do { @halfline
		0x1050, // 17: jmp    x--, 16     side 1     ;  VSYNC=1; } while (x--)
		0x6028, // 18: out    x, 8        side 0     ;  VSYNC=0; x = VBPLen
		0x0015, // 19: jmp    21          side 0     ;  while (x--) {
		0x20c1, // 20: wait   1 irq, 1    side 0     ;    @halfline
		0x0054, // 21: jmp    x--, 20     side 0     ;  }
		0x00c0, // 22: jmp    pin, 0      side 0     ;  if (DE) reset phase
		0x0018, // 23: jmp    24          side 0     ;  <modify for -DE fixup>
		0x00e1, // 24: jmp    !osre, 1    side 0     ;  if (!phase) goto main
		0x0000, // 25: jmp    0           side 0     ;  goto top
		0x2083, // 26: wait   1 gpio, 3   side 0     ; precise: @HSync
		//     .wrap                                 ;  goto vsjoin
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned int i, offset;
	u32 tc, vfp, vbp;
	u32 sysclk = clock_get_hz(clk_sys);
	int sm = pio_claim_unused_sm(dpi->pio, true);

	if (sm < 0)
		return -EBUSY;

	/* Compute mid-line time constant and start the timer SM */
	tc = (mode->htotal * (u64)sysclk) / (u64)(2000u * mode->clock);
	if (rp1dpi_pio_start_timer_both(dpi, mode->flags, tc) < 0) {
		pio_sm_unclaim(dpi->pio, sm);
		return -EBUSY;
	}

	/* Adapt program code according to DE and Sync polarity; configure program */
	pio_sm_set_enabled(dpi->pio, sm, false);
	if (dpi->de_inv) {
		instructions[1] ^= 0x0080;
		instructions[5]  = 0x00c7;
		instructions[6]  = 0x0001;
		instructions[22] = 0x00d8;
		instructions[23] = 0x0000;
	}
	for (i = 0; i < ARRAY_SIZE(instructions); i++) {
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			instructions[i] ^= 0x1000;
		if ((mode->flags & DRM_MODE_FLAG_NHSYNC) && (instructions[i] & 0xe07f) == 0x2003)
			instructions[i] ^= 0x0080;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY)
		return -EBUSY;

	/* Configure pins and SM */
	dpi->pio_stole_gpio2 = true;
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, 2);
	pio_gpio_init(dpi->pio, 2);
	sm_config_set_jmp_pin(&cfg, 1); /* "DE" is always GPIO1 */
	pio_sm_init(dpi->pio, sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, sm, 2, 1, true);

	/* Compute vertical times, remembering how we rounded vdisplay, vtotal */
	vfp = mode->vsync_start - (mode->vdisplay & ~1);
	vbp = (mode->vtotal | 1) - mode->vsync_end;
	if (vfp < 3) {
		vbp = (vbp > 3 - vfp) ? (vbp - 3 + vfp) : 0;
		vfp = 3;
	}
	pio_sm_put(dpi->pio, sm,
		   (vfp - 2) + ((vfp & 1) << 7) + (vbp << 8) +
		   ((vfp - 3) << 16) + (((~vfp) & 1) << 23) + ((vbp + 1) << 24));
	pio_sm_put(dpi->pio, sm, mode->vsync_end - mode->vsync_start - 1);
	pio_sm_exec(dpi->pio, sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, sm, pio_encode_out(pio_y, 32));
	pio_sm_exec(dpi->pio, sm, pio_encode_in(pio_y, 32));
	pio_sm_exec(dpi->pio, sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, sm, pio_encode_out(pio_y, 32));
	pio_sm_set_enabled(dpi->pio, sm, true);

	return 0;
}

int rp1dpi_pio_probe(struct rp1_dpi *dpi, const struct device_node *np)
{
	int i, j;
	struct of_phandle_args of_args = { 0 };

	dpi->csync_gpio = -1;
	dpi->gpio1_used = false;

	if (!np)
		return -EINVAL;

	/* Find which GPIO (if any) was explicitly assigned to CSYNC */
	if (!of_parse_phandle_with_args(np, "csync-gpio", "#gpio-cells", 0, &of_args)) {
		bool is_rp1 = of_device_is_compatible(of_args.np, "raspberrypi,rp1-gpio");

		of_node_put(of_args.np);
		if (is_rp1 && of_args.args_count == 2 && of_args.args[0] < 32u) {
			dpi->csync_gpio = of_args.args[0];
			return 0;
		}
	}

	/* Otherwise, check if PIO can snoop on or override DPI's GPIO1 */
	for (i = 0; !dpi->gpio1_used; i++) {
		u32 p = 0;
		const char *str = NULL;
		struct device_node *np1 = of_parse_phandle(np, "pinctrl-0", i);

		if (!np1)
			break;

		if (!of_property_read_string(np1, "function", &str) && !strcmp(str, "dpi")) {
			for (j = 0; !dpi->gpio1_used; j++) {
				if (of_property_read_string_index(np1, "pins", j, &str))
					break;
				if (!strcmp(str, "gpio1"))
					dpi->gpio1_used = true;
			}
			for (j = 0; !dpi->gpio1_used; j++) {
				if (of_property_read_u32_index(np1, "brcm,pins", j, &p))
					break;
				if (p == 1)
					dpi->gpio1_used = true;
			}
		}
		of_node_put(np1);
	}

	/* Succeed, since these features are optional */
	return 0;
}

int rp1dpi_pio_start(struct rp1_dpi *dpi, const struct drm_display_mode *mode)
{
	int r;

	if (dpi->csync_gpio < 0 &&
	    !(dpi->gpio1_used && (mode->flags & (DRM_MODE_FLAG_CSYNC | DRM_MODE_FLAG_INTERLACE))))
		return 0;

	if (dpi->pio)
		pio_close(dpi->pio);

	dpi->pio = pio_open();
	if (IS_ERR(dpi->pio)) {
		drm_err(&dpi->drm, "Could not open PIO\n");
		dpi->pio = NULL;
		return -ENODEV;
	}

	if (!(mode->flags & DRM_MODE_FLAG_INTERLACE))
		r = rp1dpi_pio_csync_prog(dpi, mode);
	else if (dpi->csync_gpio >= 0 || (mode->flags & DRM_MODE_FLAG_CSYNC))
		r = rp1dpi_pio_csync_ilace(dpi, mode);
	else
		r = rp1dpi_pio_vsync_ilace(dpi, mode);

	if (r) {
		drm_err(&dpi->drm, "Failed to initialize PIO\n");
		rp1dpi_pio_stop(dpi);
	}

	return r;
}

void rp1dpi_pio_stop(struct rp1_dpi *dpi)
{
	if (dpi->pio) {
		if (dpi->pio_stole_gpio1)
			pio_gpio_set_function(dpi->pio, 1, GPIO_FUNC_FSEL1);
		if (dpi->pio_stole_gpio2)
			pio_gpio_set_function(dpi->pio, 2, GPIO_FUNC_FSEL1);
		pio_close(dpi->pio);
		dpi->pio_stole_gpio1 = false;
		dpi->pio_stole_gpio2 = false;
		dpi->pio = NULL;
	}
}
