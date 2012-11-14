/* revert
 * omap4_panda_aic31xx.c  --  SoC audio for TI OMAP4 Panda Board
 *
 *
 * Based on:
 * sdp3430.c by Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Revision 0.1           Developed the Machine driver for AIC3110 Codec Chipset
 *
 * Revision 0.2           Updated the Headset jack insertion code-base.
 *
 * Revision 0.3           Updated the driver for ABE Support. Modified the
 *                        dai_link array to include both Front-End and back-end
 *                        DAI Declarations.
 *
 * Revision 0.4           Updated the McBSP threshold from 1 to 2 within the
 *			  mcbsp_be_hw_params_fixup() function. Also updated the
 *			  ABE Sample format to STEREO_RSHIFTED_16 from
 *                        STEREO_16_16
 *
 * Revision 0.5           Updated the omap4_hw_params() function to check for
 *			  the GPIO and the Jack Status.
 *
 * Revision 0.6           Updated the snd_soc_dai_link array for ABE to use the
 *			  Low-Power "MultiMedia1 LP" Mode
 *
 * Revision 0.7		  DAPM support implemented and switching between speaker
 *			  and headset is taken card DAPM itself
 *
 * Revision 0.8		  Ported to Linux 3.0 Kernel
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/soc-dsp.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>
#include <plat/board.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "omap-abe.h"
#include "omap-dmic.h"
#include "abe/abe_main.h"
#include "../codecs/tlv320aic31xx.h"
//#include "../../fm/fm34.h"

/* Global I2c Client Structure used for registering with I2C Bus */
static struct i2c_client *tlv320aic31xx_client;

/* Forward Declaration */
static int Qoo_headset_jack_status_check(void);

/* Headset jack information structure */
static struct snd_soc_jack hs_jack;

static int first_hs_spk_trans_ignored = 0;
/*
 * omap4_hw_params
 * This function is to configure the Machine Driver
 */
static int omap4_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	void __iomem *phymux_base = NULL;
	int ret, gpio_status;
	dev_dbg(cpu_dai->dev, "%s: Entered\n", __func__);

	/* Recording will not happen if headset is not inserted */
	gpio_status = gpio_get_value(Qoo_HEADSET_DETECT_GPIO_PIN);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
	//	fm34_mode_switch(gpio_status);
		dev_info(cpu_dai->dev, "FM34 mode and interface switch..\n");
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "can't set codec DAI configuration\n");
		return ret;
	}
	dev_dbg(cpu_dai->dev, "snd_soc_dai_set_fmt passed..\n");

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "can't set cpu DAI configuration\n");
		return ret;
	}
	dev_dbg(cpu_dai->dev, "snd_soc_dai_set_fmt passed...\n");

	/* Enabling the 19.2 Mhz Master Clock Output from OMAP4 for KC1 Board */
	phymux_base = ioremap(0x4a30a000, 0x1000);
	__raw_writel(0x00010100, phymux_base + 0x318);

	/* Added the test code to configure the McBSP4 CONTROL_MCBSP_LP
	 * register. This register ensures that the FSX and FSR on McBSP4 are
	 * internally short and both of them see the same signal from the
	 * External Audio Codec.
	 */
	phymux_base = ioremap(0x4a100000, 0x1000);
	__raw_writel(0xC0000000, phymux_base + 0x61c);

	/* Set the codec system clock for DAC and ADC. The
	 * third argument is specific to the board being used.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, AIC31XX_FREQ_19200000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "can't set codec system clock\n");
		return ret;
	}

	dev_dbg(cpu_dai->dev, "%s() - sample rate: %d channels: %d\n",
		 __func__ , params_rate(params), params_channels(params));

	dev_dbg(cpu_dai->dev, "omap4_hw_params passed...\n");
	return 0;
}

/*
 * @struct omap4_ops
 *
 * Structure for the Machine Driver Operations
 */
static struct snd_soc_ops omap4_ops = {
	.hw_params = omap4_hw_params,

};


/* @struct hs_jack_pins
 *
 * Headset jack detection DAPM pins
 *
 * @pin:    name of the pin to update
 * @mask:   bits to check for in reported jack status
 * @invert: if non-zero then pin is enabled when status is not reported
 */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "HSMIC",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

/*
 * @struct hs_jack_gpios
 *
 * Headset jack detection gpios
 *
 * @gpio:         Pin 49 on the Qoo Rev 1 Board
 * @name:         String Name "hsdet-gpio"
 * @report:       value to report when jack detected
 * @invert:       report presence in low state
 * @debouce_time: debouce time in ms
 */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	{
		.gpio = Qoo_HEADSET_DETECT_GPIO_PIN,
		.name = "hsdet-gpio",
		.report = SND_JACK_HEADSET,
#if defined(CONFIG_MINIDSP)
		.debounce_time = 240,
#else
		.debounce_time = 200,
#endif
		.jack_status_check = Qoo_headset_jack_status_check,
	},
};

static int mic_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u8 ret = 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {

		/* Power up control of MICBIAS */
		snd_soc_update_bits(codec, MICBIAS_CTRL, 0x03, 0x03);
	}
	if (SND_SOC_DAPM_EVENT_OFF(event)) {

		/* Power down control of MICBIAS */
		snd_soc_update_bits(codec, MICBIAS_CTRL, 0x03, 0x03);
	}
	return ret;
}

/* OMAP4 machine DAPM */
static const struct snd_soc_dapm_widget omap4_aic31xx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker Jack", NULL),
	SND_SOC_DAPM_MIC("HSMIC", mic_power_up_event),
	SND_SOC_DAPM_MIC("Onboard Mic", mic_power_up_event),
};

static const struct snd_kcontrol_new omap4_aic31xx_controls[]={
	SOC_DAPM_PIN_SWITCH("HSMIC"),
	SOC_DAPM_PIN_SWITCH("Speaker Jack"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
};
static const struct snd_soc_dapm_route audio_map[] = {
	/* External Speakers: HFL, HFR */
	{"Speaker Jack", NULL, "SPL"},

#ifndef AIC3100_CODEC_SUPPORT
	{"Speaker Jack", NULL, "SPR"},
#endif
	/* Headset Mic: HSMIC with bias */
	{"MIC1LP", NULL, "HSMIC"},
	{"MIC1RP", NULL, "HSMIC"},
	{"MIC1LM", NULL, "HSMIC"},
	{"INTMIC", NULL, "Onboard Mic"},

	/* Headset Stereophone(Headphone): HSOL, HSOR */
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},
};


/*
 * omap4_aic31xx_init
 * This function is to initialize the machine Driver.
 */
static int omap4_aic31xx_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct device *dev = cpu_dai->dev;
	int ret = 0;

	dev_info(dev, "entered the omap4_aic31xx_init function....\n");

	/* Add OMAP4 specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, omap4_aic31xx_dapm_widgets,
					ARRAY_SIZE(omap4_aic31xx_dapm_widgets));
	if (ret) {
		dev_err(dev, "snd_soc_dapm_new_controls failed.\n");
		return ret;
	}
	dev_dbg(dev, "snd_soc_dapm_new_controls passed..\n");


	ret = snd_soc_add_controls(codec,omap4_aic31xx_controls,ARRAY_SIZE(omap4_aic31xx_controls));
	if(ret)
	{
		dev_err(dev,"snd_soc_add_controls failed..\n");
		return ret;
	}
	dev_dbg(dev,"snd_soc_add_controls passed\n");
	/* Set up OMAP4 specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	if (ret != 0)
		dev_err(dev, "snd_soc_dapm_add_routes failed..%d\n", ret);
	/* Headset jack detection */
	ret = snd_soc_jack_new(codec, "Headset Jack",
			       SND_JACK_HEADSET, &hs_jack);
	if (ret != 0) {
		dev_err(dev, "snd_soc_jack_new failed...\n");
		return ret;
	}

	ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
				    hs_jack_pins);
	if (ret != 0) {
		dev_err(dev, "snd_soc_jack_add_pins failed... %d\n", ret);
		return ret;
	}

	ret = snd_soc_jack_add_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios),
				     hs_jack_gpios);

	dev_dbg(dev, "%s: Exiting\n", __func__);
	return ret;
}


/*
 * Qoo_headset_jack_status_check
 * This function is to check the Headset Jack Status
 */
static int Qoo_headset_jack_status_check(void)
{
	int gpio_status, ret = 0, hs_status = 0;
	struct snd_soc_codec *codec = hs_jack.codec;
	struct aic31xx_priv *priv = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = &codec->dapm;
#if defined(ENABLE_HS_BUTTON_PRESS)
	int button_press = 0;
#endif

	gpio_status = gpio_get_value(Qoo_HEADSET_DETECT_GPIO_PIN);
	dev_info(codec->dev, "#Entered %s\n", __func__);

	if (hs_jack.codec != NULL) {
		dev_dbg(codec->dev, " codec is  not null\n");
		if (!gpio_status) {
			dev_info(codec->dev,"headset connected\n");

			if (aic31xx_mic_check(codec)) {

				dev_info(codec->dev, "Headset with MIC \
						Detected Recording possible.\n");
				hs_status = 1;
			} else {
				dev_info(codec->dev, "Headset without MIC \
						Inserted Recording not possible...\n");
				hs_status = (1<<1);
			}

#if defined(ENABLE_HS_BUTTON_PRESS)
			button_press = codec->read(codec, DAC_INTR_STATUS);
			dev_info( codec->dev, "r46=0x%x\n", button_press);
			button_press = codec->read(codec, SHORT_CKT_FLAG);
			dev_info( codec->dev, "r44=0x%x\n", button_press);
			if (0x20 & button_press) {
				dev_info( codec->dev, "hook button press down\n");
				hs_status = hs_status | 0x0200;
				input_report_key(priv->idev, KEY_MEDIA, 1);
				mdelay(50);
				input_report_key(priv->idev, KEY_MEDIA, 0);
				input_sync(priv->idev);
				dev_info(codec->dev, "input_report_key is ok\n");
			}
#endif

		} else {
			dev_info(codec->dev, "headset not connected\n");
			hs_status = 0;
		}
		switch(hs_status){
			case 0:
				snd_soc_dapm_enable_pin(dapm, 	"Speaker Jack");
				snd_soc_dapm_disable_pin(dapm, 	"Headphone Jack");
				snd_soc_dapm_disable_pin(dapm, 	"HSMIC");
				break;
			case 1:
				snd_soc_dapm_disable_pin(dapm, 	"Speaker Jack");
				snd_soc_dapm_enable_pin(dapm, 	"Headphone Jack");
				snd_soc_dapm_enable_pin(dapm, 	"HSMIC");
				break;
			case 2:
				snd_soc_dapm_disable_pin(dapm, 	"Speaker Jack");
				snd_soc_dapm_enable_pin(dapm, 	"Headphone Jack");
				snd_soc_dapm_disable_pin(dapm, 	"HSMIC");
				break;
		}
		ret = snd_soc_dapm_sync(dapm);
		switch_set_state(&priv->hs_jack.sdev, hs_status);
		priv->headset_connected = !gpio_status;
		dev_info(codec->dev, "##%s : switch state = %d\n",
				__func__, !gpio_status);

		dev_dbg(codec->dev, "%s: Exiting\n", __func__);
		return ret;

	}
	return 0 ;
}

/*
 * mcbsp_be_hw_params_fixup
 * This function will be invoked by the SOC-Core layer
 * during the back-end DAI initialization.
 */

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
		struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int be_id;
	unsigned int threshold;
	unsigned int val, min_mask;
	/*struct snd_interval *rate = hw_param_interval(params,
	  SNDRV_PCM_HW_PARAM_RATE);*/
	dev_dbg(cpu_dai->dev, "%s: CPU DAI %s BE_ID %d\n",
			__func__, cpu_dai->name, rtd->dai_link->be_id);


	be_id = rtd->dai_link->be_id;

	switch (be_id) {
		case OMAP_ABE_DAI_MM_FM:
			channels->min = 2;
			threshold = 2;
			val = SNDRV_PCM_FORMAT_S16_LE;
			break;
		case OMAP_ABE_DAI_BT_VX:
			channels->min = 1;
			threshold = 1;
			val = SNDRV_PCM_FORMAT_S16_LE;
			break;
		default:
			threshold = 1;
			val = SNDRV_PCM_FORMAT_S16_LE;
			break;
	}

	min_mask = snd_mask_min(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK]);

	dev_dbg(cpu_dai->dev, "%s: Returned min_mask 0x%x Format %x\n",
			__func__, min_mask, val);

	snd_mask_reset(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK],
			min_mask);

	dev_dbg(cpu_dai->dev, "%s: Returned min_mask 0x%x Format %x\n",
			__func__, min_mask, val);

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
			SNDRV_PCM_HW_PARAM_FIRST_MASK], val);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);

	dev_dbg(cpu_dai->dev, "%s: Exiting\n", __func__);
	return 0;
}

struct snd_soc_dsp_link fe_media = {
	.playback	= true,
	.capture	= true,
	.trigger = {SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};

struct snd_soc_dsp_link fe_lp_media = {
	.playback	= true,
	.trigger = {SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};
struct snd_soc_dsp_link fe_media_capture = {
	.capture	= true,
	.trigger = {SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};

static const char *mm1_be[] = {
		OMAP_ABE_BE_MM_EXT0_DL,
		OMAP_ABE_BE_MM_EXT0_UL,
};

static const char *mm_lp_be[] = {
		OMAP_ABE_BE_MM_EXT0_DL,
};

/* ABE Port configuration structure introduced within the
* DAI_LINK Structure as private_data void pointer member
*/
t_port_config mm_ext0_config = {
	/* uplink port configuration */
	.abe_port_id_ul = MM_EXT_IN_PORT,
	.serial_id_ul = MCBSP3_RX,
	.sample_format_ul = STEREO_RSHIFTED_16,
#ifdef CONFIG_ABE_44100
	.sample_rate_ul = 44100,
#else
	.sample_rate_ul = 48000,
#endif
	.bit_reorder_ul = 0,

	/* down link port configuration */
	.abe_port_id_dl = MM_EXT_OUT_PORT,
	.serial_id_dl = MCBSP3_TX,
	.sample_format_dl = STEREO_RSHIFTED_16,
#ifdef CONFIG_ABE_44100
	.sample_rate_dl = 44100,
#else
	.sample_rate_dl = 48000,
#endif

	.bit_reorder_dl = 0,
};

/* DAI_LINK Structure definition with both Front-End and
 * Back-end DAI Declarations.
 */

static struct snd_soc_dai_link omap4_dai_abe[] = {

	{
		.name = "tlv320aic3110 Media1 LP",
		.stream_name = "Multimedia",

		/* ABE components - MM-DL (mmap) */
		.cpu_dai_name = "MultiMedia1 LP",
		.platform_name = "aess",

		/* BE is dynamic */
		.dynamic = 1,
		.dsp_link = &fe_lp_media,
		.supported_be = mm_lp_be,
		.num_be = ARRAY_SIZE(mm_lp_be),
	},
	{
		.name = "tlv320aic3110 Media",
		.stream_name = "Multimedia",

		/* ABE components - MM-UL & MM_DL */
		.cpu_dai_name = "MultiMedia1",
		.platform_name = "omap-pcm-audio",

		/* BE is dynamic */
		.dynamic = 1,
		.dsp_link = &fe_media,
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
	},
	{
		.name = "tlv320aic3110 Media Capture",
		.stream_name = "Multimedia Capture",

		/* ABE components - MM-UL2 */
		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.dsp_link = &fe_media_capture,
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
	},

	{
		.name = "Legacy McBSP",
		.stream_name = "Multimedia",

		/* ABE components - MCBSP3 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.platform_name = "omap-pcm-audio",

		/* FM */

		.codec_dai_name = "tlv320aic3110-MM_EXT",
		.codec_name = "tlv320aic3110-codec",

		.ops = &omap4_ops,
		.ignore_suspend = 1,
	},

/*
 * Backend DAIs - i.e. dynamically matched interfaces, invisible to userspace.
 * Matched to above interfaces at runtime, based upon use case.
 */

	{
		.name = OMAP_ABE_BE_MM_EXT0_DL,
		.stream_name = "FM Playback",

		/* ABE components - MCBSP3 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.platform_name = "aess",

		/* FM */
		.codec_dai_name = "tlv320aic3110-MM_EXT",
		.codec_name = "tlv320aic3110-codec",

		/* don't create ALSA pcm for this */
		.no_pcm = 1,
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &omap4_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
		.private_data = &mm_ext0_config,
		.init = omap4_aic31xx_init,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT0_UL,
		.stream_name = "FM Capture",

		/* ABE components - MCBSP3 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.platform_name = "aess",

		/* FM */
		.codec_dai_name = "tlv320aic3110-MM_EXT",
		.codec_name = "tlv320aic3110-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &omap4_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
		.private_data = &mm_ext0_config,
		.ignore_suspend = 1,
	},
};

/* The below mentioend DAI LINK Structure is to be used in McBSP3 Legacy Mode
 * only when the AIC3110 Audio Codec Chipset is interfaced via McBSP3 Port
 */

/* Audio machine driver with ABE Support */
static struct snd_soc_card snd_soc_omap4_panda_abe = {
	.name = "OMAP4_Qoo-ABE",
	.long_name = "OMAP4_Qoo_AIC3110-ABE",
	.dai_link = omap4_dai_abe,
	.num_links = ARRAY_SIZE(omap4_dai_abe),
};


static struct platform_device *omap4_snd_device;

/*
 * omap4_panda_soc_init
 * This function is used to initialize the machine Driver.
 */
static int __init omap4_panda_soc_init(void)
{
	int ret = 0;

	printk(KERN_INFO "OMAP4 EVT SoC init\n");

	omap4_snd_device = platform_device_alloc("soc-audio", -1);
	if (!omap4_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "Use ABE support\n");
	platform_set_drvdata(omap4_snd_device, &snd_soc_omap4_panda_abe);

	ret = platform_device_add(omap4_snd_device);
	if (ret) {
		printk(KERN_INFO "platform device add failed...\n");
		goto err1;
	}

	printk(KERN_INFO "OMAP4 EVT Soc Init success..\n");

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap4_snd_device);

	return ret;
}
module_init(omap4_panda_soc_init);

/*
 * omap4_soc_exit
 * This function is used to exit the machine Driver.
 */
static void __exit omap4_soc_exit(void)
{
	snd_soc_jack_free_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios),
				hs_jack_gpios);

	platform_device_unregister(omap4_snd_device);
	i2c_unregister_device(tlv320aic31xx_client);
}
module_exit(omap4_soc_exit);

MODULE_AUTHOR("Santosh Sivaraj <santosh.s@mistralsolutions.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP4 Panda");
MODULE_LICENSE("GPL");
