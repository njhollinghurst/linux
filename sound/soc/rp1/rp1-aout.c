// SPDX-License-Identifier: GPL-2.0-only
/*
 * ALSA PCM Driver for Raspberry Pi RP1 Audio Out block
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "audio_out_regs.h"

struct rp1_aout {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct device *dev;
	void __iomem *regs;
	phys_addr_t physaddr;
	struct clk *clk;
	struct dma_chan *chan;
	int opened;
	spinlock_t pos_lock; /* protects the following in ISR context */
	dma_cookie_t cookie;
	snd_pcm_uframes_t pos_period;
	snd_pcm_uframes_t pos_last;
};

static inline void aout_reg_wr(struct rp1_aout *ao, unsigned int offset, u32 val)
{
	void __iomem *addr = ao->regs + offset;

	writel(val, addr);
}

static inline void aout_reg_set(struct rp1_aout *ao, unsigned int offset, u32 val)
{
	void __iomem *addr = ao->regs + (2 << AUDIO_OUT_REGS_RWTYPE_LSB) + offset;

	writel(val, addr);
}

static inline void aout_reg_clr(struct rp1_aout *ao, unsigned int offset, u32 val)
{
	void __iomem *addr = ao->regs + (3 << AUDIO_OUT_REGS_RWTYPE_LSB) + offset;

	writel(val, addr);
}

static inline u32 aout_reg_rd(struct rp1_aout *ao, unsigned int offset)
{
	void __iomem *addr = ao->regs + offset;

	return readl(addr);
}

static void audio_powerup(struct rp1_aout *aout) {
    uint32_t val;

    /*
     * Big complicated calculations:
     * FPGA clk_pwm is 76.8MHz. CIC rate is 10, for an overall OSR of 40.
     * For 48kHz input sample rate, we need:
     * 76.8e6/40/48000 = 40 therefore pwm_range=19 (div2-1 because it's dual-edge PWM)
     * Qclamp is not required as  19*(2^15)/2^4 > 32768. Bitwidth 4.
     * Real world? N=64, range=40 ideal. 39*(2^15)/2^6
     */
    /* Bitwidth = 4, range = 14. Clamp is 14*(2^15)/2^4 = +-28672 */
    //        val = ((((int16_t)19456) << AUDIO_OUT_QCLAMP_LEFT_MAX_LSB) & AUDIO_OUT_QCLAMP_LEFT_MAX_BITS) |
//            ((((int16_t)-19456) << AUDIO_OUT_QCLAMP_LEFT_MIN_LSB) & AUDIO_OUT_QCLAMP_LEFT_MIN_BITS);

    val = ((((int16_t)20479) << AUDIO_OUT_QCLAMP_LEFT_MAX_LSB) & AUDIO_OUT_QCLAMP_LEFT_MAX_BITS) |
	   ((((int16_t)-20479) << AUDIO_OUT_QCLAMP_LEFT_MIN_LSB) & AUDIO_OUT_QCLAMP_LEFT_MIN_BITS);

    aout_reg_wr(aout, AUDIO_OUT_QCLAMP_LEFT_OFFSET, val);
    aout_reg_wr(aout, AUDIO_OUT_QCLAMP_RIGHT_OFFSET, val);
    aout_reg_wr(aout, AUDIO_OUT_PWMCTL_LEFT_OFFSET, 0x00000000);
    aout_reg_wr(aout, AUDIO_OUT_PWMCTL_RIGHT_OFFSET, 0x00000000);

    /* Range = 39 */
    aout_reg_wr(aout, AUDIO_OUT_PWMRANGE_LEFT_OFFSET, 0x27);
    aout_reg_wr(aout, AUDIO_OUT_PWMRANGE_RIGHT_OFFSET, 0x27);
    /* bias = 20 (half bitwidth) */
    val = (0x14 << AUDIO_OUT_SDMCTL_LEFT_BIAS_LSB)|
            (1 << AUDIO_OUT_SDMCTL_LEFT_CLAMP_EN_LSB)|
            (1 << AUDIO_OUT_SDMCTL_LEFT_DITHER_EN_LSB)|
            (0x5 << AUDIO_OUT_SDMCTL_LEFT_OUTPUT_BITWIDTH_LSB);
    aout_reg_wr(aout, AUDIO_OUT_SDMCTL_LEFT_OFFSET, val);
    aout_reg_wr(aout, AUDIO_OUT_SDMCTL_RIGHT_OFFSET, val);

    /* ~300ms ramp = 12k*40 samples to go from 0 to -32768 => step size of 1, interval 13 */
    val = (1 << AUDIO_OUT_MUTE_CTRL_LEFT_STEP_SIZE_LSB) | (13 << AUDIO_OUT_MUTE_CTRL_LEFT_MUTE_PERIOD_LSB);
    aout_reg_wr(aout, AUDIO_OUT_MUTE_CTRL_LEFT_OFFSET, val);
    aout_reg_wr(aout, AUDIO_OUT_MUTE_CTRL_RIGHT_OFFSET, val);

    val = (0x2 << AUDIO_OUT_FIFO_CONTROL_DWELL_TIME_LSB)|
            (0x10 << AUDIO_OUT_FIFO_CONTROL_FIFO_THRESHOLD_LSB) |
	    (1 << AUDIO_OUT_FIFO_CONTROL_DMA_DREQ_EN_LSB);
    aout_reg_wr(aout, AUDIO_OUT_FIFO_CONTROL_OFFSET, val);
}

static void audio_startup(struct rp1_aout *aout)
{
    uint32_t val;
    /* press the go button */
    val =  AUDIO_OUT_CTRL_PERIPH_EN_BITS |
            AUDIO_OUT_CTRL_LEFT_CH_ENABLE_BITS |
            AUDIO_OUT_CTRL_RIGHT_CH_ENABLE_BITS |
            (0xa << AUDIO_OUT_CTRL_CIC_RATE_LSB);
    aout_reg_wr(aout, AUDIO_OUT_CTRL_OFFSET, val);
    /* Poke zeroes in to avoid underrun */
    aout_reg_wr(aout, AUDIO_OUT_SAMPLE_FIFO_OFFSET, 0);
}

static void audio_start_unmute(struct rp1_aout *aout)
{
    /* Set the mute engine to start unmuting */
	aout_reg_clr(aout, AUDIO_OUT_MUTE_CTRL_LEFT_OFFSET, AUDIO_OUT_MUTE_CTRL_LEFT_INIT_UNMUTE_BITS | AUDIO_OUT_MUTE_CTRL_LEFT_INIT_MUTE_BITS);
	aout_reg_clr(aout, AUDIO_OUT_MUTE_CTRL_RIGHT_OFFSET, AUDIO_OUT_MUTE_CTRL_LEFT_INIT_UNMUTE_BITS | AUDIO_OUT_MUTE_CTRL_LEFT_INIT_MUTE_BITS);
	aout_reg_set(aout, AUDIO_OUT_MUTE_CTRL_LEFT_OFFSET, AUDIO_OUT_MUTE_CTRL_LEFT_INIT_UNMUTE_BITS);
	aout_reg_set(aout, AUDIO_OUT_MUTE_CTRL_RIGHT_OFFSET, AUDIO_OUT_MUTE_CTRL_RIGHT_INIT_UNMUTE_BITS);
}

static void audio_mute_sync(struct rp1_aout *aout) {
    uint32_t mask = 0x1 | 0x4;
    while ((aout_reg_rd(aout, AUDIO_OUT_MUTE_CTRL_LEFT_OFFSET) & mask) != 0)
	    usleep_range(500, 5000);
    while ((aout_reg_rd(aout, AUDIO_OUT_MUTE_CTRL_RIGHT_OFFSET) & mask) != 0)
	    usleep_range(500, 5000);
}

static const struct snd_pcm_hardware rp1_aout_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min =   1 * 1024,
	.period_bytes_max = 128 * 1024,
	.periods_min = 1,
	.periods_max = 16,
};

static void dummy_free(struct snd_pcm_runtime *runtime)
{
}

static int rp1_aout_open(struct snd_pcm_substream *substream)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (aout->opened)
		return -EBUSY;

	aout->opened = 1;
	runtime->dma_bytes = 0;
	runtime->private_data = NULL;
	runtime->private_free = dummy_free;
	runtime->hw = rp1_aout_hw;

	snd_pcm_hw_constraint_integer(substream->runtime,
				      SNDRV_PCM_HW_PARAM_PERIODS);

	audio_mute_sync(aout);
	return 0;
}

static int rp1_aout_close(struct snd_pcm_substream *substream)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);

	aout->opened = 0;
	dmaengine_terminate_sync(aout->chan);

	/* Poke zeroes in to avoid underrun */
	aout_reg_wr(aout, AUDIO_OUT_SAMPLE_FIFO_OFFSET, 0);

	return 0;
}

static int rp1_aout_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

#if 0
	ret = snd_pcm_lib_malloc_pages(substream,
				       params_buffer_bytes(hw_params));
#else
	dmaengine_terminate_sync(aout->chan);
	aout_reg_wr(aout, AUDIO_OUT_SAMPLE_FIFO_OFFSET, 0);
	runtime->dma_bytes = params_buffer_bytes(hw_params),
	runtime->dma_area = dma_alloc_coherent(aout->chan->device->dev,
					       params_buffer_bytes(hw_params),
					       &runtime->dma_addr, GFP_KERNEL);
#endif
	return ret;
}

static int rp1_aout_hw_free(struct snd_pcm_substream *substream)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	if (runtime->dma_bytes) {
		dmaengine_terminate_sync(aout->chan);
		dma_free_coherent(aout->chan->device->dev,
				  runtime->dma_bytes,
				  runtime->dma_area,
				  runtime->dma_addr);
		runtime->dma_bytes = 0;
	}
	//ret = snd_pcm_lib_free_pages(substream);
	return ret;
}

static void my_callback(void * arg)
{
	if (arg) {
		struct snd_pcm_substream *substream = arg;
		struct rp1_aout *aout = snd_pcm_substream_chip(substream);
		unsigned long flags;

		spin_lock_irqsave(&aout->pos_lock, flags);
		aout->pos_period += substream->runtime->period_size;
		spin_unlock_irqrestore(&aout->pos_lock, flags);

		snd_pcm_period_elapsed(substream);
	}
}

static int rp1_aout_prepare(struct snd_pcm_substream *substream)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct dma_slave_config config = { };
	struct dma_async_tx_descriptor *desc;
	unsigned long flags;
	int ret;

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.dst_addr = aout->physaddr + AUDIO_OUT_SAMPLE_FIFO_OFFSET;
	config.dst_maxburst = 4;
	config.device_fc = false;
	ret = dmaengine_slave_config(aout->chan, &config);
	desc = dmaengine_prep_dma_cyclic(aout->chan, runtime->dma_addr,
					 frames_to_bytes(runtime, runtime->buffer_size),
					 frames_to_bytes(runtime, runtime->period_size),
					 DMA_MEM_TO_DEV,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK | DMA_PREP_FENCE);
	if (!desc) {
		dev_err(aout->dev, "DMA preparation failed\n");
		return -EIO;
	}
	desc->callback = my_callback;
	desc->callback_param = substream;
	ret = dmaengine_submit(desc);
	spin_lock_irqsave(&aout->pos_lock, flags);
	aout->cookie = ret;
	aout->pos_period = 0;
	aout->pos_last = 0;
	spin_unlock_irqrestore(&aout->pos_lock, flags);

	return (ret < 0) ? ret : 0;
}

static int rp1_aout_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* Make DMA start */
		dma_async_issue_pending(aout->chan);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* Make DMA stop */
		dmaengine_terminate_async(aout->chan);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t rp1_aout_pointer(struct snd_pcm_substream *substream)
{
	struct rp1_aout *aout = snd_pcm_substream_chip(substream);
	unsigned long flags;
	struct dma_tx_state state;
	snd_pcm_uframes_t pos;
	enum dma_status st;

	spin_lock_irqsave(&aout->pos_lock, flags);
	st = dmaengine_tx_status(aout->chan, aout->cookie, &state);
	pos = aout->pos_period;
	if (st == DMA_IN_PROGRESS) {
		snd_pcm_uframes_t u;

		u = bytes_to_frames(substream->runtime, state.residue);
		u %= substream->runtime->period_size;
		if (u)
			pos += substream->runtime->period_size - u;
	}
	if (pos < aout->pos_last)
		pos += substream->runtime->period_size;
	if (aout->pos_period >= substream->runtime->buffer_size) {
		aout->pos_period -= substream->runtime->buffer_size;
		pos -= substream->runtime->buffer_size;
	}
	aout->pos_last = pos;
	spin_unlock_irqrestore(&aout->pos_lock, flags);

	return pos % substream->runtime->buffer_size;
}

static struct snd_pcm_ops rp1_aout_ops = {
	.open = rp1_aout_open,
	.close = rp1_aout_close,
	.hw_params = rp1_aout_hw_params,
	.hw_free = rp1_aout_hw_free,
	.prepare = rp1_aout_prepare,
	.trigger = rp1_aout_trigger,
	.pointer = rp1_aout_pointer,
};

static int rp1_aout_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct rp1_aout *aout;
	struct resource *ioresource;

	aout = devm_kzalloc(&pdev->dev, sizeof(*aout), GFP_KERNEL);
	if (!aout)
		return -ENOMEM;

	aout->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aout->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(aout->clk),
				     "could not get clk\n");

	aout->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &ioresource);
	if (IS_ERR(aout->regs))
		return dev_err_probe(&pdev->dev, PTR_ERR(aout->regs),
				     "could not map registers\n");
	aout->physaddr = ioresource->start;

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	aout->chan = dma_request_chan(&pdev->dev, "tx");
	if (IS_ERR(aout->chan))
		return dev_err_probe(&pdev->dev, PTR_ERR(aout->chan),
				     "could not request DMA channel\n");


	spin_lock_init(&aout->pos_lock);
	aout->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, aout);

	clk_prepare_enable(aout->clk);
	audio_powerup(aout);
	audio_startup(aout);
	pinctrl_pm_select_default_state(&pdev->dev);
	audio_start_unmute(aout);

	ret = snd_devm_card_new(&pdev->dev, -1, "rp1aout", THIS_MODULE, 0, &aout->card);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "could not create card\n");

	ret = snd_pcm_new(aout->card, "rp1-aout-0", 0, 1, 0, &aout->pcm);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "could not create PCM\n");

	aout->pcm->private_data = aout;
	strncpy(aout->pcm->name, "rp1-aout-0", sizeof(aout->pcm->name));
	snd_pcm_set_ops(aout->pcm, SNDRV_PCM_STREAM_PLAYBACK, &rp1_aout_ops);

	ret = 0;
	//snd_pcm_lib_preallocate_pages_for_all(aout->pcm, SNDRV_DMA_TYPE_DEV,
	//				      aout->chan->device->dev, 64 * 1024, 128 * 1024);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "could not create buffers\n");

	return snd_card_register(aout->card);
}

static const struct of_device_id rp1_aout_of_match[] = {
	{ .compatible = "raspberrypi,rp1-audio-out", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, rp1_aout_of_match);

static struct platform_driver rp1_audio_out_driver = {
	.probe  = rp1_aout_platform_probe,
	.driver = {
		.name = "rp1-audio-out",
		.of_match_table = rp1_aout_of_match,
	},
};

module_platform_driver(rp1_audio_out_driver);

MODULE_DESCRIPTION("RP1 Audio out");
MODULE_AUTHOR("Nick Hollinghurst <nick.hollinghurst@raspberrypi.com>");
MODULE_LICENSE("GPL v2");

