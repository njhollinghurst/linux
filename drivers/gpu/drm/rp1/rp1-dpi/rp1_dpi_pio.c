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
 * Copy HSYNC pulses to CSYNC (adding 1 cycle); then when VSYNC
 * is asserted, extend each pulse by an additional Y + 1 cycles.
 * The Y register should be initialized to:
 *    (htotal - 2 * hsync_width) * sys_clock / dpi_clock - 2.
 *
 * The default configuration is +HSync, +VSync, -CSync; but all
 * combinations of HSync, VSync and CSync polarity are handled.
 */

static int rp1dpi_pio_start_csync_progressive(struct rp1_dpi *dpi, u32 extend, u32 dpi_khz, u32 flags)
{
	u16 instructions[] = {  /* This is mutable */
		0x90a0, //  0: pull   block           side 1
		0x7040, //  1: out    y, 32           side 1
		//     .wrap_target
		0xb122, //  2: mov    x, y            side 1 [1]
		0x3083, //  3: wait   1 gpio, 3       side 1
		0xa122, //  4: mov    x, y            side 0 [1]
		0x2003, //  5: wait   0 gpio, 3       side 0
		0x00c7, //  6: jmp    pin, 7          side 0
		//     .wrap
		0x0047, //  7: jmp    x--, 7          side 0
		0x1002, //  8: jmp    2               side 1
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = 0
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned i, offset;

	/* Adapt program code according to sync polarity; configure program */
	pio_sm_set_enabled(dpi->pio, dpi->pio_sm, false);
	if (flags & DRM_MODE_FLAG_NVSYNC) {
		instructions[6] = 0x00c2; /* jmp pin, 2 side 0 */
	}
	if (flags & DRM_MODE_FLAG_NHSYNC) {
		instructions[3] ^= 0x80;
		instructions[5] ^= 0x80;
	}
	if (flags & DRM_MODE_FLAG_PCSYNC) {
		for(i = 0; i < ARRAY_SIZE(instructions); i++)
			instructions[i] ^= 0x1000;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY) {
		pio_close(dpi->pio);
		return -EBUSY;
	}

	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + 2, offset + ((flags & DRM_MODE_FLAG_NVSYNC) ? 7 : 6));
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* VSync on GPIO 2 */

	pio_sm_init(dpi->pio, dpi->pio_sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, dpi->pio_sm, i, 1, true);

	/* Place time constant into the FIFO; start the SM */
	extend = (extend * (clock_get_hz(clk_sys) / 1000u)) / dpi_khz;
	pio_sm_put(dpi->pio, dpi->pio_sm, extend - 2);
	pio_sm_set_enabled(dpi->pio, dpi->pio_sm, true);

	return 0;
}




/*
 * COMPOSITE SYNC FOR TV-STYLE INTERLACED OUTPUT
 *
 * We require HSYNC (GPIO3) to be a square wave, rising at the
 * start of each line and falling in the middle. VSYNC (GPIO2)
 * must be high except during lines 311, 623-624 (for 625/50),
 * i.e. the line following the last full line of image data,
 * with low pulses alternately 1 and 2 scanlines wide.
 *
 * ISR should be loaded with one of the following 32-bit words:
 * For 625/50: 0000_0010_1010_1011_1111_1111_1010_1010 (0x02ABFFAA)
 * For 525/60: 1010_1010_1011_1111_1111_1110_1010_1010 (0xAABFFEAA)
 *
 * Scratch register Y should be initialized to (2.35us - 8 cycles)
 * Narrow pulse width:   Y +  4 (2.33us at 200MHz)
 * Normal pulse width: 2*Y + 16 (4.7us)
 * Broad pulse width: 12*Y + 12 (27.78us at 200MHz)
 */

static int rp1dpi_pio_start_csync_tv(struct rp1_dpi *dpi, bool ntsc, bool pcsync)
{
	static const int wrap_target = 6;
	static const int wrap = 30;
	u16 instructions[] = {  /* This is mutable */
		0x3303, //  0: wait   0 gpio, 3       side 1 [3]
		0x3083, //  1: wait   1 gpio, 3       side 1
		0xad22, //  2: mov    x, y            side 0 [13]
		0x0143, //  3: jmp    x--, 3          side 0 [1]
		0x10c0, //  4: jmp    pin, 0          side 1
		0xb0e6, //  5: mov    osr, isr        side 1
		//     .wrap_target
		0x7021, //  6: out    x, 1            side 1
		0x102c, //  7: jmp    !x, 12          side 1
		0xb022, //  8: mov    x, y            side 1
		0x3003, //  9: wait   0 gpio, 3       side 1
		0x0a4a, // 10: jmp    x--, 10         side 0 [10]
		0x100f, // 11: jmp    15              side 1
		0x3003, // 12: wait   0 gpio, 3       side 1
		0xa122, // 13: mov    x, y            side 0 [1]
		0x004e, // 14: jmp    x--, 14         side 0
		0x7021, // 15: out    x, 1            side 1
		0x1020, // 16: jmp    !x, 0           side 1
		0x7021, // 17: out    x, 1            side 1
		0x1037, // 18: jmp    !x, 23          side 1
		0xb022, // 19: mov    x, y            side 1
		0x3083, // 20: wait   1 gpio, 3       side 1
		0x0b55, // 21: jmp    x--, 21         side 0 [11]
		0x101a, // 22: jmp    26              side 1
		0x3083, // 23: wait   1 gpio, 3       side 1
		0xa122, // 24: mov    x, y            side 0 [1]
		0x0059, // 25: jmp    x--, 25         side 0
		0x7021, // 26: out    x, 1            side 1
		0x1020, // 27: jmp    !x, 0           side 1
		0x10c6, // 28: jmp    pin, 6          side 1
		0xb0e6, // 29: mov    osr, isr        side 1
		0x7022, // 30: out    x, 2            side 1
		//     .wrap
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = -1
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned i, offset;
	u32 tc;

	/* Adapt program code according to CSync polarity; configure program */
	pio_sm_set_enabled(dpi->pio, dpi->pio_sm, false);
	if (pcsync) {
		for(i = 0; i < ARRAY_SIZE(instructions); i++)
			instructions[i] ^= 0x1000;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY) {
		pio_close(dpi->pio);
		return -EBUSY;
	}

	i = (dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1;
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, i);
	pio_gpio_init(dpi->pio, i);
	sm_config_set_jmp_pin(&cfg, 2); /* "VSync helper" signal is always GPIO2 */

	pio_sm_init(dpi->pio, dpi->pio_sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, dpi->pio_sm, i, 1, true);

	/* Load magic constants into the ISR and Y registers; start the SM */
	tc = clock_get_hz(clk_sys) / 425532u; /* cycles for 2.35us */
	pio_sm_put(dpi->pio, dpi->pio_sm, ntsc ? 0xAABFFEAA : 0x02ABFFAA);
	pio_sm_put(dpi->pio, dpi->pio_sm, tc - 8);
	pio_sm_exec(dpi->pio, dpi->pio_sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, dpi->pio_sm, pio_encode_out(pio_y, 32));
	pio_sm_exec(dpi->pio, dpi->pio_sm, pio_encode_in(pio_y, 32));
	pio_sm_exec(dpi->pio, dpi->pio_sm, pio_encode_pull(false, false));
	pio_sm_exec(dpi->pio, dpi->pio_sm, pio_encode_out(pio_y, 32));
	pio_sm_set_enabled(dpi->pio, dpi->pio_sm, true);

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

	if (!dpi->pio) {
		dpi->pio = pio_open();
		if (IS_ERR(dpi->pio)) {
			drm_err(&dpi->drm, "Could not open PIO\n");
			dpi->pio = NULL;
			return -ENODEV;
		}

		dpi->pio_sm = pio_claim_unused_sm(dpi->pio, false);
		if ((int)dpi->pio_sm < 0) {
			pio_close(dpi->pio);
			drm_err(&dpi->drm, "No free PIO SM\n");
			dpi->pio = NULL;
			return -EBUSY;
		}
	} else {
		pio_sm_set_enabled(dpi->pio, dpi->pio_sm, false);
	}

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		r = rp1dpi_pio_start_csync_tv(dpi, (mode->vtotal == 525),
					      !!(mode->flags & DRM_MODE_FLAG_PCSYNC));
	} else {
		u32 t = mode->htotal - 2*(mode->hsync_end - mode->hsync_start);
		r = rp1dpi_pio_start_csync_progressive(dpi, t,
						       mode->clock, mode->flags);
	}

	if (!r) {
		drm_info(&dpi->drm, "Start PIO to generate %cCSync on GPIO %d\n",
			 (mode->flags & DRM_MODE_FLAG_PCSYNC) ? '+' : '-',
			(dpi->csync_gpio >= 0) ? dpi->csync_gpio : 1);
	} else {
		drm_err(&dpi->drm, "Failed to initialize PIO\n");
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
