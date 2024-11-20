// SPDX-License-Identifier: GPL-2.0-only
/*
 * PIO code for RP1 DPI. We use PIO to synthesize "Composite Sync".
 * For progressive modes, CSync is optional and can exist alongside
 * HSync and VSync. NOT IMPLEMENTED YET!
 *
 * For Interlaced modes, the use of Composite Sync
 * is mandatory -- normal H/VSync cannot be generated and RP1 GPIOs
 * 2 and 3 should be left disconnected.
 *
 * CSync will be output when any of these conditions is met:
 * - The DPI driver's OF property "csync-gpio" is valid
 * - DPI mode has the DRM_MODE_FLAG_CSYNC flag set
 * - DPI mode has the DRM_MODE_FLAG_INTERLACE flag set
 *
 * If "csync-gpio" is not set, the default pin will be RP1 GPIO1.
 * Polarity will be negative unless DRM_MODE_FLAG_PCSYNC is set.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/pio_rp1.h>
#include <drm/drm_print.h>

#include "rp1_dpi.h"

/*
 * COMPOSITE SYNC FOR TV-STYLE INTERLACED OUTPUT
 *
 * We require HSYNC (GPIO3) to be a square wave, rising at the
 * start of each line and falling in the middle. VSYNC (GPIO2)
 * must be high except during lines 310, 622-623 (for 625/50),
 * i.e. conincident with the last full line of image and with
 * low pulses alternately 1 and 2 scanlines wide.
 *
 * ISR should be loaded with one of the following 32-bit words:
 * For 625/50: 0b00001010101011111111111010101010 (0x0AAFFEAA)
 * For 525/60: 0b10101010111111111111101010101010 (0xAAFFFAAA)
 *
 * Scratch register Y should be loaded with (2.33us - 2 cycles)
 * Narrow pulse width:   Y + 3
 * Normal pulse width: 2*Y + 6
 * Broad pulse width: 12*Y + 15
 *
 * HSYNC->CSYNC latency is about 4 cycles, with a jitter of
 * up to 1 cycle. To minimize jitter, PIO clock rate should
 * be a multiple of 2 * line rate, or as fast as possible.
 */

static int rp1dpi_pio_start_csync_tv(struct rp1_dpi *dpi, u32 tc, bool ntsc, bool pcsync)
{
	static const int wrap_target = 12;
	static const int wrap = 31;
	u16 instructions[] = {
		0x3083, //  0: wait   1 gpio, 3       side 1
		0xa322, //  1: mov    x, y            side 0 [3]
		0x0142, //  2: jmp    x--, 2          side 0 [1]
		0x3003, //  3: wait   0 gpio, 3       side 1
		0x13c0, //  4: jmp    pin, 0          side 1 [3]
		0xb0e6, //  5: mov    osr, isr        side 1
		0xb022, //  6: mov    x, y            side 1
		0x3083, //  7: wait   1 gpio, 3       side 1
		0x0048, //  8: jmp    x--, 8          side 0
		0xa022, //  9: mov    x, y            side 0
		0x00df, // 10: jmp    pin, 31         side 0
		0x004b, // 11: jmp    x--, 11         side 0
		//     .wrap_target
		0x7021, // 12: out    x, 1            side 1
		0x3003, // 13: wait   0 gpio, 3       side 1
		0x0031, // 14: jmp    !x, 17          side 0
		0xa022, // 15: mov    x, y            side 0
		0x0a50, // 16: jmp    x--, 16         side 0 [10]
		0xa022, // 17: mov    x, y            side 0
		0x0052, // 18: jmp    x--, 18         side 0
		0x7021, // 19: out    x, 1            side 1
		0x1020, // 20: jmp    !x, 0           side 1
		0x7021, // 21: out    x, 1            side 1
		0x3083, // 22: wait   1 gpio, 3       side 1
		0x003a, // 23: jmp    !x, 26          side 0
		0xa022, // 24: mov    x, y            side 0
		0x0a59, // 25: jmp    x--, 25         side 0 [10]
		0xa022, // 26: mov    x, y            side 0
		0x005b, // 27: jmp    x--, 27         side 0
		0x7021, // 28: out    x, 1            side 1
		0x1023, // 29: jmp    !x, 3           side 1
		0x100c, // 30: jmp    12              side 1
		0x7022, // 31: out    x, 2            side 1
		//     .wrap
	};
	struct pio_program prog = {
		.instructions = instructions,
		.length = ARRAY_SIZE(instructions),
		.origin = 0
	};
	pio_sm_config cfg = pio_get_default_sm_config();
	unsigned i, offset;

	/* Adapt program code according to CSync polarity; configure program */
	if (pcsync) {
		for(i = 0; i < ARRAY_SIZE(instructions); i++)
			instructions[i] ^= 0x1000;
	}
	offset = pio_add_program(dpi->pio, &prog);
	if (offset == PIO_ORIGIN_ANY) {
		pio_close(dpi->pio);
		return -EBUSY;
	}
	sm_config_set_wrap(&cfg, offset + wrap_target, offset + wrap);
	sm_config_set_sideset(&cfg, 1, false, false);
	sm_config_set_sideset_pins(&cfg, dpi->csync_gpio);
	pio_gpio_init(dpi->pio, dpi->csync_gpio);
	sm_config_set_jmp_pin(&cfg, 2); /* "VSync helper" signal is always GPIO2 */

	pio_sm_init(dpi->pio, dpi->pio_sm, offset, &cfg);
	pio_sm_set_consecutive_pindirs(dpi->pio, dpi->pio_sm, dpi->csync_gpio, 1, true);

	/* Load magic constants into the ISR and Y registers; start the SM */
	pio_sm_put(dpi->pio, dpi->pio_sm, ntsc ? 0xAAFFFAAA : 0x0AAFFEAA);
	pio_sm_put(dpi->pio, dpi->pio_sm, tc - 2);
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

	/* Find which GPIO was requested -- otherwise default to GPIO1 */
	dpi->csync_gpio_specified = false;
	dpi->csync_gpio = 1;

	if (np && !of_parse_phandle_with_args(np, "csync-gpio", "#gpio-cells", 0, &of_args)) {
		bool is_rp1 = of_device_is_compatible(of_args.np, "raspberrypi,rp1-gpio");
		if (is_rp1 && of_args.args_count == 2) {
			dpi->csync_gpio_specified = true;
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

	if (!dpi->csync_gpio_specified && !(mode->flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_CSYNC)))
		return 0;

	dpi->pio = pio_open();
	if (IS_ERR(dpi->pio)) {
		drm_err(&dpi->drm, "Could not open PIO\n");
		return -ENODEV;
	}

	dpi->pio_sm = pio_claim_unused_sm(dpi->pio, false);
	if ((int)dpi->pio_sm < 0) {
		pio_close(dpi->pio);
		drm_err(&dpi->drm, "No free PIO SM\n");
		return -EBUSY;
	}

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		/*
		 * Currently, we can only do interlaced CSync at SDTV rates.
		 * We compute a "time constant" Tc which is always 2.33us.
		 * Narrow sync pulses will be this wide. Normal sync pulses
		 * will be 2 * Tc (4.66us) and broad pulses 12 * Tc (28.0us).
		 * That is an acceptable compromise between 625/25i specified
		 * durations (2.35, 4.7, 27.3) and 525/30i (2.3, 4.7, 27.1).
		 */
		u32 tc = clock_get_hz(clk_sys) / 429184u;

		r = rp1dpi_pio_start_csync_tv(dpi, tc, (mode->vtotal == 525),
					      !!(mode->flags & DRM_MODE_FLAG_PCSYNC));
	} else {
		/* In the progressive case, when VSync is asserted we need to
		 * extend CSync beyond HSync by the computed duration. Note that
		 * HSync, VSync and CSync can have any combination of polarities.
		 */
		//u32 tc = mode->htotal - 2 * (mode->hsync_end - mode->hsync_start);
		//tc = (tc * (u64)clock_get_hz(clk_sys)) / (1000ull * mode->clk);
		//r = rp1dpi_pio_start_csync_progressive(dpi, tc, mode->flags);
		r = -EINVAL; // sorry not implemented!
	}

	if (!r) {
		dpi->pio_enabled = true;
		drm_info(&dpi->drm, "Start PIO to generate %cCSync on GPIO %d\n",
			 (mode->flags & DRM_MODE_FLAG_PCSYNC) ? '+' : '-',
			 dpi->csync_gpio);
	} else {
		drm_err(&dpi->drm, "Failed to initialize PIO\n");
	}

	return r;
}

void rp1dpi_pio_stop(struct rp1_dpi *dpi)
{
	if (dpi->pio_enabled) {
		pio_close(dpi->pio);
		dpi->pio_enabled = false;
	}
}
