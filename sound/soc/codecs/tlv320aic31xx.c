/* revert
 * linux/sound/soc/codecs/tlv320aic31xx.c
 *
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED AS IS AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support			14-04-2010
 *
 * Rev 0.2   Updated based Review Comments		29-06-2010
 *
 * Rev 0.3   Updated for Codec Family Compatibility     12-07-2010
 *
 * Rev 0.4   Ported to 2.6.35 kernel
 *
 * Rev 0.5   Updated the aic31xx_power_up(), aic31xx_power_down() and
 *           aic31xx_mute_codec() functions to enable and disable the ADC
 *           related registers.
 *
 * Rev 0.6   Updated the PLL Settings and also updated the Common Mode Gain
 *           Settings for better recording volume.
 *
 * Rev 0.7   updated the aic31xx_headset_speaker_path() function to check for
 *	     both playback and record. During record, if the headset jack is
 *	     removed, then the Audio Codec will be powered down.
 *
 * Rev 0.8   updated the aic31xx_hw_params() and aic31xx_set_bias_level()
 *	     functions to check the jack status before starting recording.
 *           Added the aic31xx_mic_check() function to check for the Jack Type
 *	     before allowing Audio Recording.
 *
 * Rev 0.9   Updated the HEADPHONE_DRIVER Register to have CM Voltage Settings
 *	     of 1.5V
 *
 * Rev 1.0   Implemented the DAPM support for power management and simultaneous
 *	     playback and capture support is provided
 *
 * Rev 1.1   Ported the driver to Linux 3.0 Kernel
 */

/******************************************************************************
 * INCLUDE HEADER FILES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <sound/jack.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/regulator/consumer.h>
#include "tlv320aic31xx.h"
#include <linux/clk.h>
#include <sound/tlv.h>
#include <asm/div64.h>

/******************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************/
#ifdef CONFIG_MINIDSP
extern int aic3111_minidsp_program(struct snd_soc_codec *codec);
extern void aic3111_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

#include <plat/clock.h>

static struct i2c_client *tlv320aic31xx_client;
struct regulator *audio_regulator;
static struct i2c_board_info tlv320aic31xx_hwmon_info = {
	I2C_BOARD_INFO("tlv320aic3110", 0x18),
};

/* Used to maintain the Register Access control*/
static u8 aic31xx_reg_ctl;

/* Global Variables introduced to reduce Headphone Analog Volume Control
 *Registers at run-time */
struct i2c_msg i2c_right_transaction[120];
struct i2c_msg i2c_left_transaction[120];

void debug_print_registers(struct snd_soc_codec *codec);
void aic31xx_config_hp_volume(struct snd_soc_codec *codec, int mute);
int aic31xx_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);
int aic31xx_write_locked(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);
int aic31xx_mute_codec(struct snd_soc_codec *codec, int mute);
static unsigned int aic31xx_read(struct snd_soc_codec *codec, unsigned int reg);
static int __new_control_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo);
static int __new_control_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
static int __new_control_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);

#define SOC_SINGLE_AIC31XX_N(xname, reg, shift, max, invert)\
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = n_control_info, .get = n_control_get,\
	.put = n_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

#ifdef CONFIG_MINIDSP
extern int aic3111_minidsp_program(struct snd_soc_codec *codec);
extern void aic3111_add_minidsp_controls(struct snd_soc_codec *codec);
#endif

#define NUM_INIT_REGS (sizeof(aic31xx_reg_init) /       \
				sizeof(struct aic31xx_configs))


#define SOC_SINGLE_AIC31XX(xname) {					\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,			\
	.name = xname,					\
	.info = __new_control_info,			\
	.get = __new_control_get,			\
	.put = __new_control_put,			\
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,	\
}


/* The updated aic31xx_divs Array for the KCI board having 19.2 Mhz
 * Master Clock Input coming from the FSREF2_CLK pin of OMAP4
 */
static const struct aic31xx_rate_divs aic31xx_divs[] = {
	/*
	 * mclk, rate, p_val, pll_j, pll_d, dosr, ndac, mdac, aosr, nadc, madc,
	 * blck_N, codec_speficic_initializations
	 */
	/* 8k rate */
	{19200000, 8000, 1, 5, 1200, 768, 16, 1, 128, 48, 2, 24},
	{19200000, 8000, 1, 5, 1200, 256, 24, 2, 0, 24, 2, 8},
	/* 11.025k rate */
	{19200000, 11025, 1, 4, 4100, 256, 15, 2, 128, 30, 2, 8},
	{19200000, 11025, 1, 4, 4100, 256, 15, 2, 0, 15, 2, 8},
	/* 12K rate */
	{19200000, 12000, 1, 4, 8000, 256, 15, 2, 128, 30, 2, 8},
	{19200000, 12000, 1, 4, 8000, 256, 15, 2, 0, 15, 2, 8},
	/* 16k rate */
	{19200000, 16000, 1, 5, 1200, 256, 12, 2, 128, 24, 2, 8},
	{19200000, 16000, 1, 5, 1200, 256, 12, 2, 0, 12, 2, 8},
	/* 22.05k rate */
	{19200000, 22050, 1, 4, 7040, 256, 8, 2, 128, 16, 2, 8},
	{19200000, 22050, 1, 4, 7040, 256, 8, 2, 0, 8, 2, 8},
	/* 24k rate */
	{19200000, 24000, 1, 5, 1200, 256, 8, 2, 128, 16, 2, 8},
	{19200000, 24000, 1, 5, 1200, 256, 8, 2, 0, 8, 2, 8},
	/* 32k rate */
	{19200000, 32000, 1, 5, 1200, 256, 6, 2, 128, 12, 2, 8},
	{19200000, 32000, 1, 5, 1200, 256, 6, 2, 0, 6, 2, 8},
#ifndef CONFIG_MINIDSP
	/* 44.1k rate */
	{19200000, 44100, 1, 4, 7040, 128, 4, 4, 128, 8, 2, 4},
	{19200000, 44100, 1, 4, 7040, 256, 4, 2, 0, 4, 2, 8},
#else
	/* 44.1k rate */
	 {19200000, 44100, 1, 4, 7040, 128, 2, 8, 128, 2, 8, 4},
	/*{19200000, 44100, 1, 4, 4100, 128, 3, 5, 128, 5, 3, 4}, */

#endif
	/* 48k rate */
#ifndef CONFIG_MINIDSP
	{19200000, 48000, 1, 5, 1200, 128, 4, 4, 128, 8, 2, 4},
#else
	{19200000, 48000, 1, 5, 1200, 128, 2, 8, 128, 2, 8, 4},
#endif

//	{19200000, 48000, 1, 5, 1200, 128, 4, 4, 128, 8, 2, 4},
//	{19200000, 48000, 1, 5, 1200, 128, 4, 4, 0, 4, 2, 8},
	/*96k rate */
	{19200000, 96000, 1, 5, 1200, 256, 2, 2, 128, 4, 2, 8},
	{19200000, 96000, 1, 5, 1200, 256, 2, 2, 0, 2, 2, 8},
	/*192k */
	{19200000, 192000, 1, 5, 1200, 256, 2, 1, 128, 4, 1, 16},
	{19200000, 192000, 1, 5, 1200, 256, 2, 1, 0, 2, 1, 16},
};


/*
 * Global Var aic31xx_reg
 *
 * Used to maintain a cache of Page 0 and 1 Register values.
 */
#if defined(AIC3110_CODEC_SUPPORT)
static const u8 aic31xx_reg[AIC31xx_CACHEREGNUM] = {
	/* Page 0 Registers */
	0x00,
	0x00, 0x12, 0x00, 0x00, 0x11, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x80, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x55,
	0x55, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x14, 0x0c, 0x00, 0x00,
	0x00, 0x6f, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0x10, 0xd8,
	0x7e, 0xe3, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x10, 0x32, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x12, 0x02,
	/* Page 1 Registers */
	0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#elif defined(AIC3100_CODEC_SUPPORT)
static const u8 aic31xx_reg[AIC31XX_CACHEREGNUM] = { /* Page 0
							HPL_DRIVER  Registers */
	0x00, 0x00, 0x12, 0x00, 0x00, 0x11, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x01, 0x80, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x55,
	0x55, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x14, 0x0c, 0x00, 0x00,
	0x00, 0x6f, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0x10, 0xd8,
	0x7e, 0xe3, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x10, 0x32, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x12, 0x02,
	/* Page 1 Registers */
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#endif
/*
 * The global Register Initialization sequence Array. During the Audio
 * Driver initialization, this array will be utilized to perform the
 * default initialization of the audio Driver.
 */
static const struct
aic31xx_configs aic31xx_reg_init[] = {

	/* Clock settings */
	{CLK_REG_1, CODEC_MUX_VALUE},

	{INTERFACE_SET_REG_1, BCLK_DIR_CTRL},

	{INTERFACE_SET_REG_2, DAC_MOD_CLK_2_BDIV_CLKIN},

	/* POP_REMOVAL: Step_1: Setting HP in weakly driver common mode */
	{HPL_DRIVER, 0x00},
	{HPR_DRIVER, 0x00},

	/* Step_5: Reconfiguring the CM to Band Gap mode */

	{HP_POP_CTRL, BIT7 | HP_POWER_UP_3_04_SEC | HP_DRIVER_3_9_MS \
					|CM_VOLTAGE_FROM_BAND_GAP},

	{HEADPHONE_DRIVER, HP_DRIVER_ON},

	/* Speaker Ramp up time scaled to 30.5ms */
	{PGA_RAMP_CTRL, 0x70},

	/* Headset Detect setting */
	{INTL_CRTL_REG_1, 0xC0},

	/* previous value was 0x01 */
	{CM_SET, 0x20},

	/* short circuit protection of HP and Speaker power bits */
	{HP_SPK_ERR_CTL, 3},

	/* Headset detection enabled by default and Debounce programmed to 64 ms
	 * for Headset Detection and 32ms for Headset button-press Detection
	 */
	{HEADSET_DETECT, (HP_DEBOUNCE_64_MS | HS_DETECT_EN | \
					HS_BUTTON_PRESS_32_MS)},

#ifdef USE_DRC
#ifndef CONFIG_MINIDSP
	/* ADC PRB configured to PRB_5 */
	/*{DAC_PRB_SEL_REG, 0x02},*/
	{ADC_PRB_SEL_REG, 0x05},
#endif
	/*DRC settings */
	{DRC_CTL_REG_3, 0xB6},
	{DRC_CTL_REG_2, 0x00},
	{DRC_CTL_REG_1, 0x1A}, /* Increased Threshold to -21db */
#endif

};


static const char *dac_mute[] = {"Unmute", "Mute"};
static const char *adc_mute[] = {"Unmute", "Mute"};
static const char *hpl_pwr[] = {"Off", "On"};
static const char *hpr_pwr[] = {"Off", "On"};
static const char *ldac_pwr[] = {"Off", "On"};
static const char *rdac_pwr[] = {"Off", "On"};

static const char *dacvolume_extra[] = {"L & R Ind Vol", "LVol = RVol",
	"RVol = LVol"};
static const char *dacvolume_control[] = {"control register", "pin"};
static const char *dacsoftstep_control[] = {"1 step / sample",
	"1 step / 2 sample", "disabled"};

static const char *beep_generator[] = {"Disabled", "Enabled"};

static const char *micbias_voltage[] = {"off", "2 V", "2.5 V", "AVDD"};
static const char *dacleftip_control[] = {"off", "left data",
	"right data", "(left + right) / 2"};
static const char *dacrightip_control[] = { "off", "right data", "left data",
	"(left+right)/2" };

static const char *dacvoltage_control[] = {"1.35 V", "5 V ", "1.65 V", "1.8 V"};
static const char *headset_detection[] = {"Disabled", "Enabled"};
static const char *drc_enable[] = {"Disabled", "Enabled"};
static const char *mic1lp_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *mic1rp_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *mic1lm_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *cm_enable[] = {"off", "10 k", "20 k", "40 k"};
static const char *mic_enable[] = {"Gain controlled by D0 - D6", "0 db Gain"};
static const char *mic1_enable[] = {"floating", "connected to CM internally"};


/* Creates an array of the Single Ended Widgets */
static const struct soc_enum aic31xx_enum[] = {
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 3, 2, dac_mute),
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 2, 2, dac_mute),
	SOC_ENUM_SINGLE(DAC_MUTE_CTRL_REG, 0, 3, dacvolume_extra),
	SOC_ENUM_SINGLE(PIN_VOL_CTRL, 7, 2, dacvolume_control),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 0, 3, dacsoftstep_control),
	SOC_ENUM_SINGLE(LEFT_BEEP_GEN, 7, 2, beep_generator),
	SOC_ENUM_SINGLE(MICBIAS_CTRL, 0, 4, micbias_voltage),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 4, 4, dacleftip_control),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 2, 4, dacrightip_control),
	SOC_ENUM_SINGLE(HEADPHONE_DRIVER, 3, 4, dacvoltage_control),
	SOC_ENUM_SINGLE(HEADSET_DETECT, 7, 2, headset_detection),
	SOC_ENUM_DOUBLE(DRC_CTL_REG_1, 6, 5, 2, drc_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 6, 4, mic1lp_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 4, 4, mic1rp_enable),
	SOC_ENUM_SINGLE(MIC_GAIN, 2, 4, mic1lm_enable),
	SOC_ENUM_SINGLE(MIC_PGA, 7, 2, mic_enable),
	SOC_ENUM_SINGLE(ADC_IP_SEL, 6, 4, cm_enable),
	SOC_ENUM_SINGLE(ADC_IP_SEL, 4, 4, mic1lm_enable),
	SOC_ENUM_SINGLE(CM_SET, 7, 2, mic1_enable),
	SOC_ENUM_SINGLE(CM_SET, 6, 2, mic1_enable),
	SOC_ENUM_SINGLE(CM_SET, 5, 2, mic1_enable),
	SOC_ENUM_SINGLE(ADC_FGA, 7, 2, adc_mute),
	SOC_ENUM_SINGLE(HEADPHONE_DRIVER, 7, 2, hpl_pwr),
	SOC_ENUM_SINGLE(HEADPHONE_DRIVER, 6, 2, hpr_pwr),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 7, 2, ldac_pwr),
	SOC_ENUM_SINGLE(DAC_CHN_REG, 6, 2, rdac_pwr),
};


static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_fgain_tlv, 00, 10, 0);
static const DECLARE_TLV_DB_SCALE(adc_cgain_tlv, -2000, 50, 0);
static const DECLARE_TLV_DB_SCALE(mic_pga_tlv, 0, 50, 0);
static const DECLARE_TLV_DB_SCALE(hp_drv_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(class_D_drv_tlv, 600, 600, 0);
static const DECLARE_TLV_DB_SCALE(hp_vol_tlv, -7830, 60, 0);
static const DECLARE_TLV_DB_SCALE(sp_vol_tlv, -7830, 60, 0);

/*
 * controls that need to be exported to the user space
 */
static const struct snd_kcontrol_new aic31xx_snd_controls[] = {
	/* DAC Volume Control */
	 SOC_DOUBLE_R_SX_TLV("DAC Playback Volume", LDAC_VOL, RDAC_VOL, 8, \
						0xffffff81, 0x30, dac_vol_tlv),
	/* DAC mute control */
	SOC_ENUM("Left DAC Mute", aic31xx_enum[LMUTE_ENUM]),
	SOC_ENUM("Right DAC Mute", aic31xx_enum[RMUTE_ENUM]),
	/* DAC volume Extra control */
	SOC_ENUM("DAC volume Extra control", aic31xx_enum[DACEXTRA_ENUM]),
	/* DAC volume Control register/pin control */
	SOC_ENUM("DAC volume Control register/pin",
			aic31xx_enum[DACCONTROL_ENUM]),
	/* DAC Volume soft stepping control */
	SOC_ENUM("DAC Volume soft stepping", aic31xx_enum[SOFTSTEP_ENUM]),
	/* HP driver mute control */
	SOC_DOUBLE_R("HP driver mute", HPL_DRIVER, HPR_DRIVER, 2, 2, 0),

	/* SP driver mute control */
	SOC_DOUBLE_R("SP driver mute", SPL_DRIVER, SPR_DRIVER, 2, 2, 0),

	/* ADC FINE GAIN */
	SOC_SINGLE_TLV("ADC FINE GAIN", ADC_FGA, 4, 4, 1, adc_fgain_tlv),
	/* ADC COARSE GAIN */
	SOC_DOUBLE_S8_TLV("ADC COARSE GAIN", ADC_CGA, 0xffffff68, 0x28, \
					adc_cgain_tlv),
	/* ADC MIC PGA GAIN */
	SOC_SINGLE_TLV("ADC MIC_PGA GAIN", MIC_PGA, 0, 119, 0, mic_pga_tlv),

	/* HP driver Volume Control */
	SOC_DOUBLE_R_TLV("HP driver Volume", HPL_DRIVER, HPR_DRIVER, 3, 0x09, \
					0, hp_drv_tlv),
	/* Left DAC input selection control */
	SOC_ENUM("Left DAC input selection", aic31xx_enum[DACLEFTIP_ENUM]),
	/* Right DAC input selection control */
	SOC_ENUM("Right DAC input selection", aic31xx_enum[DACRIGHTIP_ENUM]),

	/* Beep generator Enable/Disable control */
	SOC_ENUM("Beep generator Enable / Disable", aic31xx_enum[BEEP_ENUM]),
	/* Beep generator Volume Control */
	SOC_DOUBLE_R("Beep Volume Control(0 = -61 db, 63 = 2 dB)",
			LEFT_BEEP_GEN, RIGHT_BEEP_GEN, 0, 0x3F, 1),
	/* Beep Length MSB control */
	SOC_SINGLE("Beep Length MSB", BEEP_LENGTH_MSB, 0, 255, 0),
	/* Beep Length MID control */
	SOC_SINGLE("Beep Length MID", BEEP_LENGTH_MID, 0, 255, 0),
	/* Beep Length LSB control */
	SOC_SINGLE("Beep Length LSB", BEEP_LENGTH_LSB, 0, 255, 0),
	/* Beep Sin(x) MSB control */
	SOC_SINGLE("Beep Sin(x) MSB", BEEP_SINX_MSB, 0, 255, 0),
	/* Beep Sin(x) LSB control */
	SOC_SINGLE("Beep Sin(x) LSB", BEEP_SINX_LSB, 0, 255, 0),
	/* Beep Cos(x) MSB control */
	SOC_SINGLE("Beep Cos(x) MSB", BEEP_COSX_MSB, 0, 255, 0),
	/* Beep Cos(x) LSB control */
	SOC_SINGLE("Beep Cos(x) LSB", BEEP_COSX_LSB, 0, 255, 0),

	/* Mic Bias voltage */
	SOC_ENUM("Mic Bias Voltage", aic31xx_enum[MICBIAS_ENUM]),

	/* DAC Processing Block Selection */
	SOC_SINGLE("DAC Processing Block Selection(0 <->25)",
			DAC_PRB_SEL_REG, 0, 0x19, 0),
	/* ADC Processing Block Selection */
	SOC_SINGLE("ADC Processing Block Selection(0 <->25)",
			ADC_PRB_SEL_REG, 0, 0x12, 0),

	/* Throughput of 7-bit vol ADC for pin control */
	SOC_SINGLE("Throughput of 7 - bit vol ADC for pin",
			PIN_VOL_CTRL, 0, 0x07, 0),

	/* Audio gain control (AGC) */
	SOC_SINGLE("Audio Gain Control(AGC)", AGC_CTRL_1, 7, 0x01, 0),
	/* AGC Target level control */
	SOC_SINGLE("AGC Target Level Control", AGC_CTRL_1, 4, 0x07, 1),
	/* AGC Maximum PGA applicable */
	SOC_SINGLE("AGC Maximum PGA Control", AGC_CTRL_3, 0, 0x77, 0),
	/* AGC Attack Time control */
	SOC_SINGLE("AGC Attack Time control", AGC_CTRL_4, 3, 0x1F, 0),
	/* AGC Attac Time Multiply factor */
	SOC_SINGLE("AGC_ATC_TIME_MULTIPLIER", AGC_CTRL_4, 0, 8, 0),
	/* AGC Decay Time control */
	SOC_SINGLE("AGC Decay Time control", AGC_CTRL_5, 3, 0x1F, 0),
	/* AGC Decay Time Multiplier */
	SOC_SINGLE("AGC_DECAY_TIME_MULTIPLIER", AGC_CTRL_5, 0, 8, 0),
	/* AGC HYSTERISIS */
	SOC_SINGLE("AGC_HYSTERISIS", AGC_CTRL_2, 6, 3, 0),
	/* AGC Noise Threshold */
	SOC_SINGLE("AGC_NOISE_THRESHOLD", AGC_CTRL_2, 1, 32, 1),
	/* AGC Noise Bounce control */
	SOC_SINGLE("AGC Noice bounce control", AGC_CTRL_6, 0, 0x1F, 0),
	/* AGC Signal Bounce control */
	SOC_SINGLE("AGC Signal bounce control", AGC_CTRL_7, 0, 0x0F, 0),

	/* HP Output common-mode voltage control */
	SOC_ENUM("HP Output common - mode voltage control",
			aic31xx_enum[VOLTAGE_ENUM]),

	/* Headset detection Enable/Disable control */
	SOC_ENUM("Headset detection Enable / Disable", aic31xx_enum[HSET_ENUM]),

	/* DRC Enable/Disable control */
	SOC_ENUM("DRC Enable / Disable", aic31xx_enum[DRC_ENUM]),
	/* DRC Threshold value control */
	SOC_SINGLE("DRC Threshold value(0 = -3 db, 7 = -24 db)",
			DRC_CTL_REG_1, 2, 0x07, 0),
	/* DRC Hysteresis value control */
	SOC_SINGLE("DRC Hysteresis value(0 = 0 db, 3 = 3 db)",
			DRC_CTL_REG_1, 0, 0x03, 0),
	/* DRC Hold time control */
	SOC_SINGLE("DRC hold time", DRC_CTL_REG_2, 3, 0x0F, 0),
	/* DRC attack rate control */ SOC_SINGLE("DRC attack rate",
			DRC_CTL_REG_3, 4, 0x0F, 0),
	/* DRC decay rate control */
	SOC_SINGLE("DRC decay rate", DRC_CTL_REG_3, 0, 0x0F, 0),
	/* MIC1LP selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1LP selection for ADC I/P P - terminal",
			aic31xx_enum[MIC1LP_ENUM]),
	/* MIC1RP selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1RP selection for ADC I/P P - terminal",
			aic31xx_enum[MIC1RP_ENUM]),
	/* MIC1LM selection for ADC I/P P-terminal */
	SOC_ENUM("MIC1LM selection for ADC I/P P - terminal",
			aic31xx_enum[MIC1LM_ENUM]),
	/* CM selection for ADC I/P M-terminal */
	SOC_ENUM("CM selection for ADC IP M - terminal",
			aic31xx_enum[CM_ENUM]),
	/* MIC1LM selection for ADC I/P M-terminal */
	SOC_ENUM("MIC1LM selection for ADC I/P M - terminal",
			aic31xx_enum[MIC1LMM_ENUM]),
	/* MIC PGA Setting */
	SOC_ENUM("MIC PGA Setting", aic31xx_enum[MIC_ENUM]),
	/* MIC1LP CM Setting */
	SOC_ENUM("MIC1LP CM Setting", aic31xx_enum[MIC1_ENUM]),
	/* MIC1RP CM Setting */
	SOC_ENUM("MIC1RP CM Setting", aic31xx_enum[MIC2_ENUM]),
	/* MIC1LP CM Setting */
	SOC_ENUM("MIC1LM CM Setting", aic31xx_enum[MIC3_ENUM]),
	/* ADC mute control */
	SOC_ENUM("ADC Mute", aic31xx_enum[ADCMUTE_ENUM]),
	/* DAC Left & Right Power Control */
	SOC_ENUM("LDAC_PWR_CTL", aic31xx_enum[LDAC_ENUM]),
	SOC_ENUM("RDAC_PWR_CTL", aic31xx_enum[RDAC_ENUM]),

	/* HP Driver Power up/down control */
	SOC_ENUM("HPL_PWR_CTL", aic31xx_enum[HPL_ENUM]),
	SOC_ENUM("HPR_PWR_CTL", aic31xx_enum[HPR_ENUM]),

	/* MIC PGA Enable/Disable */
	SOC_SINGLE("MIC_PGA_EN_CTL", MIC_PGA, 7, 2, 0),

	/* HP Detect Debounce Time */
	SOC_SINGLE("HP_DETECT_DEBOUNCE_TIME", HEADSET_DETECT, 2, 0x05, 0),
	/* HP Button Press Debounce Time */
	SOC_SINGLE("HP_BUTTON_DEBOUNCE_TIME", HEADSET_DETECT, 0, 0x03, 0),

	/* Added for Debugging */
	SOC_SINGLE("LoopBack_Control", INTERFACE_SET_REG_2, 4, 4, 0),


#ifdef AIC3110_CODEC_SUPPORT
	/* For AIC3110 output is stereo so we are using	SOC_DOUBLE_R macro */

	/* SP Class-D driver output stage gain Control */
	SOC_DOUBLE_R_TLV("Class - D driver Volume", SPL_DRIVER, SPR_DRIVER, 3, 0x04,\
					 0, class_D_drv_tlv),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	/* SP Class-D driver output stage gain Control */
	SOC_SINGLE("Class - D driver Volume(0 = 6 dB, 4 = 24 dB)",
			SPL_DRIVER, 3, 0x04, 0),
#endif

	/* HP Analog Gain Volume Control */
	SOC_DOUBLE_R_TLV("HP Analog Gain", L_ANLOG_VOL_2_HPL, \
			R_ANLOG_VOL_2_HPR, 0, 0x7F, 1, hp_vol_tlv),

#ifdef AIC3110_CODEC_SUPPORT
	/* SP Analog Gain Volume Control */
	SOC_DOUBLE_R_TLV("SP Analog Gain", L_ANLOG_VOL_2_SPL, \
			R_ANLOG_VOL_2_SPR, 0, 0x7F, 1, sp_vol_tlv),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	/* SP Analog Gain Volume Control */
	SOC_SINGLE("SP Analog Gain(0 = 0 dB, 127 = -78.3 dB)",
			L_ANLOG_VOL_2_SPL, 0, 0x7F, 1),
#endif
	/* Program Registers */
	SOC_SINGLE_AIC31XX("Program Registers"),
};

/* Left Output Mixer */
static const struct snd_kcontrol_new
left_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("From DAC_L", DAC_MIX_CTRL, 6, 1, 0),
	SOC_DAPM_SINGLE("From MIC1LP", DAC_MIX_CTRL, 5, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", DAC_MIX_CTRL, 4, 1, 0),
};

/* Right Output Mixer - Valid only for AIC31xx,3110,3100 */
static const struct
snd_kcontrol_new right_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("From DAC_R", DAC_MIX_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", DAC_MIX_CTRL, 1, 1, 0),
};

static const struct
snd_kcontrol_new pos_mic_pga_controls[] = {
	SOC_DAPM_SINGLE("MIC1LP_PGA_CNTL", MIC_GAIN, 6, 0x3, 0),
	SOC_DAPM_SINGLE("MIC1RP_PGA_CNTL", MIC_GAIN, 4, 0x3, 0),
	SOC_DAPM_SINGLE("MIC1LM_PGA_CNTL", MIC_GAIN, 2, 0x3, 0),
};

static const struct
snd_kcontrol_new neg_mic_pga_controls[] = {
	SOC_DAPM_SINGLE("CM_PGA_CNTL", ADC_IP_SEL, 6, 0x3, 0),
	SOC_DAPM_SINGLE("MIC1LM_PGA_CNTL", ADC_IP_SEL, 4, 0x3, 0),
};

static int pll_power_on_event(struct snd_soc_dapm_widget *w, \
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		dev_dbg(codec->dev, "pll->on pre_pmu");
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		dev_dbg(codec->dev, "pll->off\n" );
	}
	mdelay(10);
	return 0;
}

#if defined(PLL_PWR_ON_EVENT)
static int ndac_power_on_event(struct snd_soc_dapm_widget *w, \
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 11);
			counter++;
		} while ((counter < 20) && ((value & 0x80) != 0x80));
		dev_dbg(codec->dev, "ndac->on  counter= %d\n", counter);
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		dev_(codec->dev, "ndac->off\n");
	}
	return 0;
}

static int mdac_power_on_event(struct snd_soc_dapm_widget *w, \
		struct snd_kcontrol *kcontrol, int event)
{

	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 12);
			counter++;
		} while ((counter < 20) && ((value & 0x80) != 0x80));
		dev_dbg(codec->dev, "mdac->on  counter= %d\n", counter);
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 37);
			counter++;
		} while ((counter < 20) && ((value & 0x88) != 0x00));
		dev_dbg(codec->dev, "mdac->off  L&R DAC powered off,counter= %d\n", counter);
	}
	return 0;
}

static int nadc_power_on_event(struct snd_soc_dapm_widget *w, \
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 18);
			counter++;
		} while ((counter < 20) && ((value & 0x80) != 0x80));
		dev_dbg(codec->dev, "nadc->on  ADC powered off,counter= %d\n", counter);
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 36);
			counter++;
		} while ((counter < 20) && ((value & 0x40) != 0x00));
		dev_dbg(codec->dev, "nadc->off  ADC powered off,counter= %d\n", counter);
	}
	return 0;
}

static int madc_power_on_event(struct snd_soc_dapm_widget *w, \
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 19);
			counter++;
		} while ((counter < 20) && ((value & 0x80) != 0x80));
		dev_dbg(codec->dev, "madc->on  counter= %d\n", counter);
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		dev_dbg(codec->dev, "madc->off\n");
	}
	return 0;
}

static int bclkn_power_on_event(struct snd_soc_dapm_widget *w, \
		struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, 30);
			counter++;
		} while ((counter < 20) && ((value & 0x80) != 0x80));
		dev_dbg(codec->dev, "bclkn->on  counter= %d\n", counter);
	}else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		dev_dbg(codec->dev, "bclkn->off\n");
	}
	return 0;
}
#endif

static int aic31xx_dac_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;
	if (SND_SOC_DAPM_EVENT_ON(event)) {

		/* Check for the DAC FLAG register to know if the DAC is really
		 * powered up
		 */
		if (w->shift == 7) {

			counter = 0;
			do {
				mdelay(1);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x80) == 0));
			dev_dbg(codec->dev, "Left DAC powered up,counter = %d\n", counter);
		} else if (w->shift == 6) {
			counter = 0;
			do {
				mdelay(1);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x08) == 0));
			dev_dbg(codec->dev, "Right DAC powered up,counter= %d\n", counter);
		}

	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {

		/* Check for the DAC FLAG register to know if the DAC is
		 * powered down
		 */
		if (w->shift == 7) {
			counter = 0;
			do {
				mdelay(1);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x80) != 0));
			dev_dbg(codec->dev, "Left DAC powered down, \
						counter = %d\n", counter);
		} else if (w->shift == 6) {
			counter = 0;
			do {
				mdelay(1);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x08) != 0));
			dev_dbg(codec->dev, "RDAC powered down, counter = %d\n", counter);
		}
	}
	return 0;
}

static int aic31xx_adc_power_up_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter, value;
	struct snd_soc_codec *codec = w->codec;

	if (SND_SOC_DAPM_EVENT_ON(event)) {

		/* Check for the ADC FLAG register to know if the ADC is
		 * really powered up
		 */
		counter = 0;
		do {
			mdelay(10);
			value = aic31xx_read(codec, ADC_FLAG);
			counter++;
		} while ((counter < 40) && ((value & 0x40) == 0));
		if (counter  == 40) {
			dev_dbg(codec->dev,"ADC_FLAG not updated while switch on of ADC\n");
#ifdef AIC31x_CODEC_DEBUG
			debug_print_registers(codec);
#endif
			value = aic31xx_read(codec, ADC_FLAG);
			dev_dbg(codec->dev,"ADC_FLAG value = %x\n", value);
		} else {
			dev_dbg(codec->dev,"ADC_FLAG updated while switch on of ADC\n");
			value = aic31xx_read(codec, ADC_FLAG);
			dev_dbg(codec->dev,"ADC_FLAG value=%x, counter=%d\n", value, counter);
		}
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {

		/* Check for the ADC FLAG register to know if the ADC is
		 * powered down
		 */
		counter = 0;
		do {
			mdelay(1);
			value = aic31xx_read(codec, ADC_FLAG);
			counter++;
		} while ((counter < 20) && ((value & 0x40) != 0));
		if (counter == 20) {
			dev_dbg(codec->dev,"ADC_FLAG not updated while switch off of ADC\n");
			value = aic31xx_read(codec, ADC_FLAG);
			dev_dbg(codec->dev,"ADC_FLAG value = %x\n", value);
		} else {
			dev_dbg(codec->dev,"ADC_FLAG updated while switch off of ADC\n");
			value = aic31xx_read(codec, ADC_FLAG);
			dev_dbg(codec->dev,"ADC_FLAG value=%x,counter= %d\n", value, counter);
		}
	}
	return 0;
}

/*Hp_power_up_event without powering on/off headphone driver, instead muting hpl & hpr */
static int aic31xx_hp_power_up_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	u8 lv, rv, val;
	struct snd_soc_codec *codec = w->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	if (event & SND_SOC_DAPM_POST_PMU) {

		if (!(strcmp(w->name,"HPL Driver"))) {
			lv = aic31xx_read(codec, L_ANLOG_VOL_2_HPL);
			lv |= 0x7f;
			rv = aic31xx_read(codec, R_ANLOG_VOL_2_HPR);
			rv |= 0x7f;
			val = lv ^ ((lv ^ rv) & -(lv < rv));
			mutex_lock(&aic31xx->mutex_page);
			while(val > 9) {
				aic31xx_write_locked(codec, L_ANLOG_VOL_2_HPL,(0x80 |(val & 0x7f)));
				aic31xx_write_locked(codec, R_ANLOG_VOL_2_HPR, (0x80 |(val & 0x7f)));
				val-=5;
			}
			aic31xx_write_locked(codec, L_ANLOG_VOL_2_HPL,(0x80 | 0x09));
			aic31xx_write_locked(codec, R_ANLOG_VOL_2_HPR,(0x80 | 0x09));
			mutex_unlock(&aic31xx->mutex_page);
		}


		if (aic31xx->from_resume) {
			aic31xx_mute_codec(codec, 0);
			aic31xx->from_resume = 0;
		}
	}

	if (event & SND_SOC_DAPM_PRE_PMD) {

		if (!(strcmp(w->name,"HPL Driver"))) {
			lv = aic31xx_read(codec, L_ANLOG_VOL_2_HPL);
			lv &= 0x7f;
			rv = aic31xx_read(codec, R_ANLOG_VOL_2_HPR);
			rv &= 0x7f;
			val = lv ^ ((lv ^ rv) & -(lv < rv));
			mutex_lock(&aic31xx->mutex_page);
			while(val < 127) {
				aic31xx_write_locked(codec, L_ANLOG_VOL_2_HPL, (0x80 |(val & 0x7f)));
				aic31xx_write_locked(codec, R_ANLOG_VOL_2_HPR, (0x80 | (val & 0x7f)));
				val+=10;
			}
			/* The D7 bit is set to zero to mute the gain */
			aic31xx_write_locked(codec, L_ANLOG_VOL_2_HPL, 0x7f);
			aic31xx_write_locked(codec, R_ANLOG_VOL_2_HPR, 0x7f);
			mutex_unlock(&aic31xx->mutex_page);
		}

	}

	return 0;
}

static int aic31xx_sp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 counter;
	int value,lv,rv,val;
	struct snd_soc_codec *codec = w->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	if (event & SND_SOC_DAPM_POST_PMU) {
		/* Check for the DAC FLAG register to know if the SPL & SPR are
		 * really powered up
		 */
		if (w->shift == 7) {
			counter = 0;
			do {
				mdelay(5);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((value & 0x10) == 0);
			dev_dbg(codec->dev, "##SPL Power up Iterations %d\r\n", counter);
		}
		if (w->shift == 6) {
			counter = 0;
			do {
				mdelay(5);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((value & 0x01) == 0);
			dev_dbg(codec->dev, "##SPR Power up Iterations %d\r\n", counter);
		}

		if (!(strcmp(w->name,"SPL Class - D"))) {
			lv = aic31xx_read(codec,L_ANLOG_VOL_2_SPL);
			lv|= 0x7f;
			rv = aic31xx_read(codec,R_ANLOG_VOL_2_SPR);
			rv|= 0x7f;
			val= lv ^ ((lv ^ rv) & -(lv < rv));
			mutex_lock(&aic31xx->mutex_page);
			while(val > 10){
				aic31xx_write_locked(codec,L_ANLOG_VOL_2_SPL,(0x80 |(val & 0x7f)));
				aic31xx_write_locked(codec,R_ANLOG_VOL_2_SPR,(0x80 |(val & 0x7f)));
				val--;
			}
			mutex_unlock(&aic31xx->mutex_page);
		}
	}

	if ( event & SND_SOC_DAPM_POST_PMD ) {
		/* Check for the DAC FLAG register to know if the SPL & SPR are
		 * powered down
		 */
		if (w->shift == 7) {
			counter = 0;
			do {
				mdelay(5);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x10) != 0));
		dev_dbg(codec->dev, "##SPL Power down Iterations %d\r\n", counter);

		}
		if (w->shift == 6) {
			counter = 0;
			do {
				mdelay(5);
				value = aic31xx_read(codec, DAC_FLAG_1);
				counter++;
			} while ((counter < 50) && ((value & 0x01) != 0));
		dev_dbg(codec->dev, "##SPR Power down Iterations %d\r\n", counter);

		}
	}

	if ( event & SND_SOC_DAPM_PRE_PMD) {
		if (!(strcmp(w->name,"SPL Class - D"))) {
			lv = aic31xx_read(codec,L_ANLOG_VOL_2_SPL);
			lv&= 0x7f;
			rv = aic31xx_read(codec,R_ANLOG_VOL_2_SPR);
			rv&= 0x7f;
			val= lv ^ ((lv ^ rv) & -(lv < rv));
                        mutex_lock(&aic31xx->mutex_page);
			while(val < 127){
				aic31xx_write_locked(codec,L_ANLOG_VOL_2_SPL,(0x80 | (val & 0x7f)));
				aic31xx_write_locked(codec,R_ANLOG_VOL_2_SPR,(0x80 | (val & 0x7f)));
				val++;
			}
			/* The D7 bit is set to zero to mute the gain */
			aic31xx_write_locked(codec,L_ANLOG_VOL_2_SPL,(val & 0x7f));
			aic31xx_write_locked(codec,R_ANLOG_VOL_2_SPR,(val & 0x7f));
			mutex_unlock(&aic31xx->mutex_page);
		}

	}

	return 0;
}



static const struct snd_soc_dapm_widget aic31xx_dapm_widgets[] = {
	/* DACs */
	SND_SOC_DAPM_DAC_E("Left DAC", "Left Playback", DAC_CHN_REG, 7, 0, \
			aic31xx_dac_power_up_event, SND_SOC_DAPM_POST_PMU | \
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("Right DAC", "Right Playback", DAC_CHN_REG, 6, 0, \
			aic31xx_dac_power_up_event, SND_SOC_DAPM_POST_PMU | \
			SND_SOC_DAPM_POST_PMD),

	/* Output Mixers */
	SND_SOC_DAPM_MIXER("Left Output Mixer", SND_SOC_NOPM, 0, 0,
			left_output_mixer_controls,
			ARRAY_SIZE(left_output_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Output Mixer", SND_SOC_NOPM, 0, 0,
			right_output_mixer_controls,
			ARRAY_SIZE(right_output_mixer_controls)),

	/* Output drivers */
	SND_SOC_DAPM_PGA_E("HPL Driver", HPL_DRIVER, 2, 0,\
			NULL, 0, aic31xx_hp_power_up_event,\
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("HPR Driver", HPR_DRIVER, 2, 0,\
			NULL, 0, aic31xx_hp_power_up_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),


#ifndef AIC3100_CODEC_SUPPORT
	/* For AIC3111 and AIC3110 as it is stereo both left and right channel
	 * class-D can be powered up/down
	 */
	SND_SOC_DAPM_PGA_E("SPL Class - D", CLASSD_SPEAKER_AMP, 7, 0, NULL, 0, \
				aic31xx_sp_event, SND_SOC_DAPM_POST_PMU | \
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("SPR Class - D", CLASSD_SPEAKER_AMP, 6, 0, NULL, 0, \
				aic31xx_sp_event, SND_SOC_DAPM_POST_PMU | \
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
#endif

#ifdef AIC3100_CODEC_SUPPORT
	/* For AIC3100 as is mono only left
	 * channel class-D can be powered up/down
	 */
	SND_SOC_DAPM_PGA("SPL Class - D", CLASSD_SPEAKER_AMP, 7, 0, NULL, 0, \
			aic31xx_sp_event, SND_SOC_DAPM_POST_PMU | \
			SND_SOC_DAPM_POST_PMD),

#endif

	/* ADC */
	SND_SOC_DAPM_ADC_E("ADC", "Capture", ADC_DIG_MIC, 7, 0, \
			aic31xx_adc_power_up_event, SND_SOC_DAPM_POST_PMU | \
			SND_SOC_DAPM_POST_PMD),

	/*Input Selection to MIC_PGA*/
	SND_SOC_DAPM_MIXER("P_Input_Mixer", SND_SOC_NOPM, 0, 0,
		pos_mic_pga_controls, ARRAY_SIZE(pos_mic_pga_controls)),
	SND_SOC_DAPM_MIXER("M_Input_Mixer", SND_SOC_NOPM, 0, 0,
		neg_mic_pga_controls, ARRAY_SIZE(neg_mic_pga_controls)),

	/*Enabling & Disabling MIC Gain Ctl */
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL", MIC_PGA, 7, 1, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLLCLK", CLK_REG_2, 7, 0, pll_power_on_event, \
				 SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("CODEC_CLK_IN", SND_SOC_NOPM, 0, 0, NULL, 0),
#if !defined(PLL_PWR_ON_EVENT)
	SND_SOC_DAPM_SUPPLY("NDAC_DIV", NDAC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MDAC_DIV", MDAC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NADC_DIV", NADC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MADC_DIV", MADC_CLK_REG, 7, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", BCLK_N_VAL, 7, 0, NULL, 0),
#else
	SND_SOC_DAPM_SUPPLY("NDAC_DIV", NDAC_CLK_REG, 7, 0, ndac_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("MDAC_DIV", MDAC_CLK_REG, 7, 0, mdac_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("NADC_DIV", NADC_CLK_REG, 7, 0, nadc_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("MADC_DIV", MADC_CLK_REG, 7, 0, madc_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", BCLK_N_VAL, 7, 0, bclkn_power_on_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
#endif
	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("SPL"),

#ifndef AIC3100_CODEC_SUPPORT
	SND_SOC_DAPM_OUTPUT("SPR"),
#endif

	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	SND_SOC_DAPM_INPUT("MIC1RP"),
	SND_SOC_DAPM_INPUT("MIC1LM"),
	SND_SOC_DAPM_INPUT("INTMIC"),

};

static const struct snd_soc_dapm_route
aic31xx_audio_map[] = {

	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"NDAC_DIV", NULL, "CODEC_CLK_IN"},
	{"NADC_DIV", NULL, "CODEC_CLK_IN"},
	{"MDAC_DIV", NULL, "NDAC_DIV"},
	{"MADC_DIV", NULL, "NADC_DIV"},
	{"BCLK_N_DIV", NULL, "MADC_DIV"},
	{"BCLK_N_DIV", NULL, "MDAC_DIV"},

	/* Clocks for ADC */
	{"ADC", NULL, "MADC_DIV"},
	{"ADC", NULL, "BCLK_N_DIV"},

	/* Mic input */
	{"P_Input_Mixer", "MIC1LP_PGA_CNTL", "MIC1LP"},
	{"P_Input_Mixer", "MIC1RP_PGA_CNTL", "MIC1RP"},
	{"P_Input_Mixer", "MIC1LM_PGA_CNTL", "MIC1LM"},

	{"M_Input_Mixer", "CM_PGA_CNTL", "MIC1LM"},
	{"M_Input_Mixer", "MIC1LM_PGA_CNTL", "MIC1LM"},

	{"MIC_GAIN_CTL", NULL, "P_Input_Mixer"},
	{"MIC_GAIN_CTL", NULL, "M_Input_Mixer"},

	{"ADC", NULL, "MIC_GAIN_CTL"},
	{"ADC", NULL, "INTMIC"},

	/* Clocks for DAC */
	{"Left DAC", NULL, "MDAC_DIV" },
	{"Right DAC", NULL, "MDAC_DIV"},
	{"Left DAC", NULL, "BCLK_N_DIV" },
	{"Right DAC", NULL, "BCLK_N_DIV"},

	/* Left Output */
	{"Left Output Mixer", "From DAC_L", "Left DAC"},
	{"Left Output Mixer", "From MIC1LP", "MIC1LP"},
	{"Left Output Mixer", "From MIC1RP", "MIC1RP"},

	/* Right Output */
	{"Right Output Mixer", "From DAC_R", "Right DAC"},
	{"Right Output Mixer", "From MIC1RP", "MIC1RP"},

	/* HPL path */
	{"HPL Driver", NULL, "Left Output Mixer"},
	{"HPL", NULL, "HPL Driver"},

	/* HPR path */
	{"HPR Driver", NULL, "Right Output Mixer"},
	{"HPR", NULL, "HPR Driver"},

	/* SPK L path */
	{"SPL Class - D", NULL, "Left Output Mixer"},
	{"SPL", NULL, "SPL Class - D"},

#ifndef AIC3100_CODEC_SUPPORT
	/* SPK R path */
	{"SPR Class - D", NULL, "Right Output Mixer"},
	{"SPR", NULL, "SPR Class - D"},
#endif
};


#if 0
/*
 *----------------------------------------------------------------------------
 * Function : n_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC3110 registers.
 *
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : n_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3110 registers.
 *
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val, reg;
	unsigned short mask, shift;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	reg = mc->reg;
	if (!strcmp(kcontrol->id.name, "ADC COARSE GAIN")) {
		mask = 0xFF;
		shift = 0;
		val = aic31xx_read(codec, reg);
		ucontrol->value.integer.value[0] =
			(val >= 40) ? (val - 40) : (val);
	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static unsigned int n_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u8 val, val_mask;
	int reg, err;
	unsigned int invert = mc->invert;
	int max = mc->max;
	dev_dbg(codec->dev,"n_control_put\n");
	reg = mc->reg;
	val = ucontrol->value.integer.value[0];
	if (invert)
		val = max - val;
	if (!strcmp(kcontrol->id.name, "ADC COARSE GAIN")) {
		val = (val >= 0) ? (val + 40) : (40);
		val_mask = 0xFF;
	}

	err = snd_soc_update_bits_locked(codec, reg, val_mask, val);
	if (err < 0) {
		dev_err(codec->dev,"Error while updating bits\n");
		return err;
	}


	return 0;
}
#endif



/*
 * aic31xx_change_page
 *
 * This function is to switch between page 0 and page 1.
 */
/* aic31xx_priv::mutex_page must be locked! */
int aic31xx_change_page(struct snd_soc_codec *codec, u8 new_page)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];

	if (aic31xx == NULL)
		dev_err(codec->dev,"##Codec Private member NULL..\n");

	data[0] = 0;
	data[1] = new_page;

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		dev_err(codec->dev,"##Error in changing page to 1\n");
		return -1;
	}
	aic31xx->page_no = new_page;
	return 0;
}

/*
 * aic31xx_write_reg_cache
 * This function is to write aic31xx register cache
 */
static inline void aic31xx_write_reg_cache(struct snd_soc_codec
		*codec, u16 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= AIC31xx_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * aic31xx_write_locked
 *
 * This function is to write to the aic31xx register space.
 *
 * IMPORTANT: caller assures that aic31xx_priv::mutex_page is locked
 */
int aic31xx_write_locked(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 data[2];
	u8 page;
	int status;

	page = reg / 128;
	data[AIC31XX_REG_OFFSET_INDEX] = reg % 128;

	if (aic31xx->page_no != page) {
		status = aic31xx_change_page(codec, page);

		if (status != 0) {
			dev_err(codec->dev, "#%s: Error in switching page %d\n",
				__func__, page);
			return -1;
		}
	}
	/* data is * D15..D8 aic31xx register offset * D7...D0
	 * register data
	 */
	data[AIC31XX_REG_DATA_INDEX] = value & AIC31XX_8BITS_MASK;

#if defined(EN_REG_CACHE)
	if ((page == 0) || (page == 1) || (page == 2))
		aic31xx_write_reg_cache(codec, reg, value);
#endif

#if 0
	/* Code Added to detect PLL Register Overwrite Problem */
	if((data[0] == 5) && (page == 0)) {
		do {
			dev_dbg(codec->dev,
				"/n/n#%s: Writing into Page 0 Reg 5 with \
				 value of 0x%x\n", __func__, data[1]);
			dump_stack();
		}while (0);
	}
#endif

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		dev_err(codec->dev, "Error in i2c write\n");
		return -EIO;
	}
	return 0;
}

/*
 * aic31xx_write
 *
 * This function is to write to the aic31xx register space.
 *
 * It is a general-use version that will lock aic31xx_priv::mutex_page
 */
int aic31xx_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&aic31xx->mutex_page);
	ret = aic31xx_write_locked(codec, reg, value);
	mutex_unlock(&aic31xx->mutex_page);
	return ret;
}

/*
 * aic31xx_read
 *
 * This function is to read the aic31xx register space.
 */
static unsigned int aic31xx_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value;
	u8 page = reg / 128;
	int status;

	mutex_lock(&aic31xx->mutex_page);

	reg = reg % 128;
	if (aic31xx->page_no != page) {
		status = aic31xx_change_page(codec, page);

		if (status != 0) {
			dev_err(codec->dev, "#%s: Error in Changing Page %d\n",
					__func__, page);
			value = 0;
			goto end;
		}
	}

	i2c_master_send(codec->control_data, (char *)&reg, 1);
	i2c_master_recv(codec->control_data, &value, 1);

end:
	mutex_unlock(&aic31xx->mutex_page);

	return value;
}


/*
 *----------------------------------------------------------------------------
 * Function : debug_print_registers
 * Purpose  : Debug routine to dump all the Registers of Page 0
 *
 *----------------------------------------------------------------------------
 */
void debug_print_registers(struct snd_soc_codec *codec)
{
	int i;
	u8 data;

	dev_info(codec->dev,"### Page 0 Regs from 0 to 95\n");

	for (i = 0; i < 95; i++) {
		data = (u8) aic31xx_read(codec, i);
		dev_info(codec->dev, "reg = %d val = %x\n", i, data);
	}
	dev_info(codec->dev,"### Page 1 Regs from 30 to 52\n");

	for (i = 158; i < 180; i++) {
		data = (u8) aic31xx_read(codec, i);
		dev_info(codec->dev, "reg = %d val = %x\n", (i%128), data);
	}


	dev_info(codec->dev,"####SPL_DRIVER_GAIN %d SPR_DRIVER_GAIN %d\n\n",
			aic31xx_read(codec, SPL_DRIVER), \
			aic31xx_read(codec, SPR_DRIVER));

	dev_info(codec->dev,"##### L_ANALOG_VOL_2_SPL %d R_ANLOG_VOL_2_SPR %d\n\n",
			aic31xx_read(codec, L_ANLOG_VOL_2_SPL), \
			aic31xx_read(codec, R_ANLOG_VOL_2_SPR));

	dev_info(codec->dev,"#### LDAC_VOL %d RDAC_VOL %d\n\n",
			aic31xx_read(codec, LDAC_VOL),
			aic31xx_read(codec, RDAC_VOL));
	dev_info(codec->dev,"###OVER Temperature STATUS (Page 0 Reg 3) %x\n\n",
			aic31xx_read(codec, OT_FLAG));
	dev_info(codec->dev,"###SHORT CIRCUIT STATUS (Page 0 Reg 44) %x\n\n",
			aic31xx_read(codec, SHORT_CKT_FLAG));

	dev_info(codec->dev,"###INTR_FLAG: SHORT_CKT(Page 0 Reg 46) %x\n\n",
			aic31xx_read(codec, DAC_INTR_STATUS));
	dev_info(codec->dev,"###Speaker_Driver_Short_Circuit (Page 1 Reg 32)%x\n\n",
			aic31xx_read(codec, CLASSD_SPEAKER_AMP));

	dev_info(codec->dev,"@@@  MIC_PGA (P1 R47) = 0x%x\n\n",
			aic31xx_read(codec, MIC_PGA));

	dev_info(codec->dev,"@@@  ADC_FGA (P0 R82) = 0x%x\n\n",
			aic31xx_read(codec, ADC_FGA));

	dev_info(codec->dev,"@@@  ADC_CGA (P0 R83) = 0x%x\n\n",
			aic31xx_read(codec, ADC_CGA));
}

/*
 * __new_control_info
 *
 * This function is to initialize data for new control required to * program the
 * AIC31xx registers.
 */
static int __new_control_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 65535;

	return 0;
}

/*
 * __new_control_get
 *
 * This function is to read data of new control for program the AIC31xx
 * registers.
 */
static int __new_control_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val;

	val = aic31xx_read(codec, aic31xx_reg_ctl);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

/*
 * __new_control_put
 *
 * __new_control_put is called to pass data from user/application to the
 * driver.
 */
static int __new_control_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	u32 data_from_user = ucontrol->value.integer.value[0];
	u8 data[2];
	int ret = 0;

	mutex_lock(&aic31xx->mutex_page);

	aic31xx_reg_ctl = data[0] = (u8) ((data_from_user & 0xFF00) >> 8);
	data[1] = (u8) ((data_from_user & 0x00FF));

	if (!data[0])
		aic31xx->page_no = data[1];

	dev_dbg(codec->dev,"reg = %d val = %x\n", data[0], data[1]);

	if (codec->hw_write(codec->control_data, data, 2) != 2) {
		dev_alert(codec->dev, "Error in i2c write\n");
		ret = -EIO;
		goto end;
	}

end:
	mutex_unlock(&aic31xx->mutex_page);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_mic_check
 * Purpose  : This function checks for the status of the Page0 Register 67
 *            [Headset Detect] Register and checks if Bit6 is set. This denotes
 *            that a Jack with Microphone is plugged in or not.
 *
 * Returns  : 1 is the Bit 6 of Pg 0 Reg 67 is set
 *            0 is the Bit 6 of Pg 0 Reg 67 is not set.
 *----------------------------------------------------------------------------
 */
int aic31xx_mic_check(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	u8 mic_status = 0, value;
	u8 regval;

	/* Read the Register contents */
	regval = aic31xx_read(codec, HEADSET_DETECT);
	/*
	 * Disabling and enabling the headset detection functionality
	 * to avoid false detection of microphone
	 */
	aic31xx_write(codec, HEADSET_DETECT, regval & ~BIT7);
	mdelay(10);
	regval = aic31xx_read(codec, HEADSET_DETECT);
	value = aic31xx_read(codec, MICBIAS_CTRL);
	aic31xx_write(codec, MICBIAS_CTRL, (value | (BIT1 | BIT0)));

	aic31xx_write(codec, HEADSET_DETECT, regval | BIT7);
	/*
	 * Delay configured with respect to the Debounce time. If headset
	 * debounce time is changing, it should reflect in the delay in the
	 * below line.
	 */
	mdelay(HP_DEBOUNCE_TIME_IN_MS);
	regval = aic31xx_read(codec, HEADSET_DETECT);
	aic31xx_write(codec, HEADSET_DETECT, regval & ~BIT7);

	value = aic31xx_read(codec, MICBIAS_CTRL);
	aic31xx_write(codec, MICBIAS_CTRL, (value & ~(BIT1 | BIT0)));

	aic31xx->headset_current_status = regval;

	mic_status = (regval & BIT6);
	return mic_status;
}

/*
 * aic31xx_set_bias_level
 *
 * This function is to get triggered when dapm events occurs.
 */
static int aic31xx_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	//struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s: Entered: level %d\n", __func__, level);

	if (level == codec->dapm.bias_level) {
		dev_dbg(codec->dev,"##%s :Current & previous levels same\n", __func__);
		return 0;
	}

	dev_dbg(codec->dev,"###aic31xx_set_bias_level New Level %d\n", level);

	switch (level) {
	/* full On */
	case SND_SOC_BIAS_ON:
		dev_dbg(codec->dev,"###aic31xx_set_bias_level BIAS_ON\n");
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev,"###aic31xx_set_bias_level BIAS_PREPARE\n");
		break;

	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev,"###aic31xx_set_bias_level STANDBY\n");
		break;

	/* Off, without power */
	case SND_SOC_BIAS_OFF:
		dev_dbg(codec->dev,"###aic31xx_set_bias_level OFF\n");
		break;
	}
	codec->dapm.bias_level = level;

	dev_dbg(codec->dev,"%s: Exiting\n", __func__);
	return 0;
}


/*
 * aic31xx_get_divs
 *
 * This function is to get required divisor from the "aic31xx_divs" table.
 */
static inline int aic31xx_get_divs(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aic31xx_divs); i++) {
		if ((aic31xx_divs[i].rate == rate) &&
				(aic31xx_divs[i].mclk == mclk)) {
			return i;
		}
	}
	return -EINVAL;
}

/**
 * aic31xx_hw_params
 *
 * This function is to set the hardware parameters for AIC31xx.
 * The functions set the sample rate and audio serial data word length.
 */
static int aic31xx_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;
	u8 data;
	dev_dbg(codec->dev," %s:Stream= %d\n", __func__ , substream->stream);
	dev_dbg(codec->dev,"%s===>", __func__);
	mutex_lock(&aic31xx->mutex_codec);
	dev_dbg(codec->dev,"###aic31xx_hw_params SyCLK %x MUTE %d PowerStatus %d\n", \
			aic31xx->sysclk, aic31xx->mute, aic31xx->power_status);

	/* Setting the playback status.
	 * Update the capture_stream Member of the Codec's Private structure
	 * to denote that we will be performing Audio capture from now on.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic31xx->playback_status = 1;
		aic31xx->playback_stream = 1;
	} else if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) && \
							(codec->active < 2))
		aic31xx->playback_stream = 0;
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		aic31xx->playback_status = 1;
		aic31xx->capture_stream = 1;
	} else if ((substream->stream != SNDRV_PCM_STREAM_CAPTURE) &&
							(codec->active < 2))
		aic31xx->capture_stream = 0;

	dev_dbg(codec->dev, "%s: playback_stream= %d capture_stream=%d \
		priv_playback_stream= %d priv_record_stream=%d\n" , __func__, \
		SNDRV_PCM_STREAM_PLAYBACK, SNDRV_PCM_STREAM_CAPTURE, \
		aic31xx->playback_stream, aic31xx->capture_stream);

	codec->dapm.bias_level = 2;

	i = aic31xx_get_divs(aic31xx->sysclk, params_rate(params));

	if (i < 0) {
		dev_alert(codec->dev, "sampling rate not supported\n");
		mutex_unlock(&aic31xx->mutex_codec);
		return i;
	}

	dev_dbg(codec->dev,"###aic31xx_hw_params Sampling Rate %d\n", params_rate(params));

	/* We will fix R value to 1 and will make P & J=K.D as
	 * varialble  Setting P & R values
	 */

	if (codec->active < 2) {

		snd_soc_update_bits(codec, CLK_REG_2, 0x7F, \
					((aic31xx_divs[i].p_val << 4) | 0x01));

		/* J value */
		snd_soc_update_bits(codec, CLK_REG_3, 0x3F,
						aic31xx_divs[i].pll_j);

		/* MSB & LSB for D value */
		aic31xx_write(codec, CLK_REG_4, (aic31xx_divs[i].pll_d >> 8));
		aic31xx_write(codec, CLK_REG_5, (aic31xx_divs[i].pll_d & \
					AIC31XX_8BITS_MASK));

		/* NDAC divider value */
		snd_soc_update_bits(codec, NDAC_CLK_REG, 0x7F, \
						aic31xx_divs[i].ndac);

		/* MDAC divider value */
		snd_soc_update_bits(codec, MDAC_CLK_REG , 0x7F, \
						aic31xx_divs[i].mdac);

		/* DOSR MSB & LSB values */
		aic31xx_write(codec, DAC_OSR_MSB, aic31xx_divs[i].dosr >> 8);
		aic31xx_write(codec, DAC_OSR_LSB, \
				aic31xx_divs[i].dosr & AIC31XX_8BITS_MASK);

		/* NADC divider value */
		snd_soc_update_bits(codec, NADC_CLK_REG, 0x7F, \
						aic31xx_divs[i].nadc);

		/* MADC divider value */
		snd_soc_update_bits(codec, MADC_CLK_REG, 0x7F, \
						aic31xx_divs[i].madc);

		/* AOSR value */
		aic31xx_write(codec, ADC_OSR_REG, aic31xx_divs[i].aosr);
		}
		/* BCLK N divider */
		snd_soc_update_bits(codec, BCLK_N_VAL, 0x7F, \
						aic31xx_divs[i].blck_N);
		dev_dbg(codec->dev,"## Writing NDAC %d MDAC %d NADC %d MADC %d DOSR %d \
		AOSR %d\n", aic31xx_divs[i].ndac, aic31xx_divs[i].mdac, \
			aic31xx_divs[i].nadc, aic31xx_divs[i].madc, \
			aic31xx_divs[i].dosr, aic31xx_divs[i].aosr);


	data = aic31xx_read(codec, INTERFACE_SET_REG_1);

	data = data & ~(3 << 4);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (AIC31XX_WORD_LEN_20BITS << DATA_LEN_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (AIC31XX_WORD_LEN_24BITS << DATA_LEN_SHIFT);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (AIC31XX_WORD_LEN_32BITS << DATA_LEN_SHIFT);
		break;
	}

	aic31xx_write(codec, INTERFACE_SET_REG_1, data);

	/*setting pmdown_time of pcm rtd structure to 0*/
	/* rtd->pmdown_time = 0; */

	dev_dbg(codec->dev,"%s: Exiting\n", __func__);
	mutex_unlock(&aic31xx->mutex_codec);
	dev_dbg(codec->dev, "%s<====",__func__);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_config_hp_volume
 * Purpose  : This function is used to configure the I2C Transaction Global
 *            Variables. One of them is for ramping down the HP Analog Volume
 *            and the other one is for ramping up the HP Analog Volume
 *
 *----------------------------------------------------------------------------
 */
void aic31xx_config_hp_volume(struct snd_soc_codec *codec, int mute)
{
	struct i2c_client *client = codec->control_data;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	unsigned int count;
	struct aic31xx_configs  *pReg;
	signed char regval;
	unsigned char low_value;
	unsigned int  reg_update_count;

	/* User has requested to mute or bring down the Headphone Analog Volume
	 * Move from 0 db to -35.2 db
	 */
	if (mute > 0) {
		pReg = &aic31xx->hp_analog_right_vol[0];

		for (count = 0, regval = 0; regval <= 30; \
							count++, regval += 1) {

			(pReg + count)->reg_offset = R_ANLOG_VOL_2_HPR;
			(pReg + count)->reg_val = (0x80 | regval);
		}
		(pReg + (count - 1))->reg_val = (0x80 | \
				HEADPHONE_ANALOG_VOL_MIN);

		pReg = &aic31xx->hp_analog_left_vol[0];

		for (count = 0, regval = 0; regval <= 30; \
							count++, regval += 1) {

			(pReg + count)->reg_offset = L_ANLOG_VOL_2_HPL;
			(pReg + count)->reg_val = (0x80 | regval);
		}
		(pReg + (count - 1))->reg_val = (0x80 | \
				HEADPHONE_ANALOG_VOL_MIN);
		reg_update_count = count - 1;
		dev_dbg(codec->dev,"##CFG_HP_VOL count %d reg_update %d regval %d\n", count,
				reg_update_count, regval);
	} else {
		/* User has requested to unmute or bring up the Headphone Analog
		 * Volume Move from -35.2 db to 0 db
		 */
		pReg = &aic31xx->hp_analog_right_vol[0];

		low_value = HEADPHONE_ANALOG_VOL_MIN;

		for (count = 0, regval = low_value; regval >= 0;
				count++, regval -= 1) {
			(pReg + count)->reg_offset = R_ANLOG_VOL_2_HPR;
			(pReg + count)->reg_val = (0x80 | regval);
		}
		(pReg + (count - 1))->reg_val = (0x80);

		pReg = &aic31xx->hp_analog_left_vol[0];

		for (count = 0, regval = low_value; regval >= 0;
				count++, regval -= 1) {
			(pReg + count)->reg_offset = L_ANLOG_VOL_2_HPL;
			(pReg + count)->reg_val = (0x80 | regval);
		}
		(pReg + (count - 1))->reg_val = (0x80);
		reg_update_count = count;
		dev_dbg(codec->dev,"##CFG_HP_VOL LowVal 0x%x count %d reg_update %d \
				regval %d\n", low_value, count, \
				reg_update_count, regval);
	}

	/* Change to Page 1 */
	aic31xx_change_page(codec, 1);

	if (aic31xx->i2c_regs_status == 0) {
		for (count = 0; count < reg_update_count; count++) {
			i2c_right_transaction[count].addr = client->addr;
			i2c_right_transaction[count].flags =
				client->flags & I2C_M_TEN;
			i2c_right_transaction[count].len = 2;
			i2c_right_transaction[count].buf = (char *)
				&aic31xx->hp_analog_right_vol[count];
		}

		for (count = 0; count < reg_update_count; count++) {
			i2c_left_transaction[count].addr = client->addr;
			i2c_left_transaction[count].flags =
				client->flags & I2C_M_TEN;
			i2c_left_transaction[count].len = 2;
			i2c_left_transaction[count].buf = (char *)
				&aic31xx->hp_analog_left_vol[count];
		}
		aic31xx->i2c_regs_status = 1;
	}
	/* Perform bulk I2C transactions */
	if (i2c_transfer(client->adapter, i2c_right_transaction,
				reg_update_count) != reg_update_count) {
		dev_err(codec->dev, "Error while Write brust i2c data error on "
				"RIGHT_ANALOG_HPR!\n");
	}


	if (i2c_transfer(client->adapter, i2c_left_transaction,
				reg_update_count) != reg_update_count) {
		dev_err(codec->dev, "Error while Write brust i2c data error on "
				"LEFT_ANALOG_HPL!\n");
	}

	return;
}


#if 0
/*
 *----------------------------------------------------------------------------
 * Function : aic31xx_mute_codec
 * Purpose  : This function is to mute or unmute the left and right DAC
 *
 *----------------------------------------------------------------------------
 */
int aic31xx_mute_codec(struct snd_soc_codec *codec, int mute)
{

	u8 dac_reg;
	u8 value;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&aic31xx->mutex_codec);

	dev_dbg(codec->dev,"##+ new aic31xx_mute_codec %d (current state is %d,\
		headset_connected=%d)\n", mute, aic31xx->mute,\
					 aic31xx->headset_connected);

	if ((mute) && (aic31xx->mute != 1)) {

		if (codec->active != 0) {
			if ((aic31xx->playback_stream == 1)  && \
						(aic31xx->capture_stream /*record_stream*/ == 1)) {

				dev_warn(codec->dev, "session still going on..\n");
				mutex_unlock(&aic31xx->mutex_codec);
				return 0;
			}
		}

		dev_dbg(codec->dev, "muting codec\n");

		/* Also update the global Playback Status Flag. This is required
		 * for biquad update
		 */

		aic31xx->playback_status = 0;

		if (aic31xx->playback_stream) {

			/* Mute the DAC channel */
			dac_reg = aic31xx_read(codec, DAC_MUTE_CTRL_REG);
			aic31xx_write(codec, DAC_MUTE_CTRL_REG, \
						(dac_reg | MUTE_ON));
			dev_dbg(codec->dev,"##DAC MUTE Completed..\r\n");
		}

		if (aic31xx->capture_stream /*record_stream*/) {
			/* Mute the ADC channel */
			value = aic31xx_read(codec, ADC_FGA);
			aic31xx_write(codec, ADC_FGA, (value | BIT7));
			dev_dbg(codec->dev,"##ADC MUTE Completed..\r\n");
		}
		aic31xx->mute = 1;

	} else if ((!mute) || (aic31xx->playback_status)) {

		dev_dbg(codec->dev, "unmuting codec\n");
		aic31xx->playback_status = 1;

		/* Check whether Playback or Record Session is about to Start */
		if (aic31xx->playback_stream) {
			/* Unmuting the DAC channel */
			dac_reg = aic31xx_read(codec, DAC_MUTE_CTRL_REG);
			aic31xx_write(codec, DAC_MUTE_CTRL_REG, (dac_reg & \
								~MUTE_ON));
			dev_dbg(codec->dev,"##DAC UNMUTED ...\n");
		}

		if (aic31xx->capture_stream /*record_stream*/) {
			/* Unmuting the ADC channel */
			value = aic31xx_read(codec, ADC_FGA);
			aic31xx_write(codec, ADC_FGA, (value & ~BIT7));
			dev_dbg(codec->dev,"##ADC UNMUTED ...\n");

		}

		aic31xx->power_status = 1;
		aic31xx->mute = 0;
	}

	dev_dbg(codec->dev,"##-aic31xx_mute_codec %d\n", mute);

	dev_dbg(codec->dev,"%s: Exiting\n", __func__);
	mutex_unlock(&aic31xx->mutex_codec);
	return 0;
}

/*
 * aic31xx_mute
 * This function is to mute or unmute the left and right DAC
 */
static int aic31xx_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev,"###aic31xx_mute Flag %x\n", mute);
	aic31xx_mute_codec(codec, mute);

#ifdef AIC31x_CODEC_DEBUG
	debug_print_registers(codec);
#endif

	return 0;
}
#else
/*
 * aic31xx_adc_mute
 *
 * Mutes or Unmutes the ADC part of the Codec depending on the
 * value of the Mute flag.
 */
static int aic31xx_adc_mute(struct snd_soc_codec *codec, int mute)
{
	int retval = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	if ((mute) && (aic31xx->mute != 1)) {
		retval = snd_soc_update_bits(codec, ADC_FGA, 0x80,
							(CLEAR | BIT7));
		aic31xx->mute = 1;
	} else if (!mute ) {
		retval = snd_soc_update_bits(codec, ADC_FGA, 0x80,
							(CLEAR & ~BIT7));
		aic31xx->mute = 0;
	}
#ifdef AIC31XX_DEBUG
	debug_print_registers(codec);
#endif
	return retval;
}

/*
 * aic31xx_dac_mute - mute or unmute the left and right DAC
 */
static int aic31xx_dac_mute(struct snd_soc_codec *codec, int mute)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s: mute = %d\t priv->mute = %d\t headset_detect = %d\n",
		__func__, mute, aic31xx->mute, aic31xx->headset_connected);

	/* Also update the global Playback Status Flag. This is required for
	 * biquad update.
	*/
	if ((mute) && (aic31xx->mute != 1)) {
		aic31xx->playback_status = 0;

#ifdef USE_DRC
		if (!aic31xx->headset_connected) {
			/*Switch off the DRC*/
			snd_soc_update_bits(codec, DRC_CTL_REG_1, 0x60,
						(CLEAR & ~ (BIT6 | BIT5)));
		}
#endif
		snd_soc_update_bits(codec, DAC_MUTE_CTRL_REG, 0x0C,
						(CLEAR | MUTE_ON));
		aic31xx->mute = 1;
	} else if (!mute) {
		aic31xx->playback_status = 1;
		/* Check whether Playback or Record Session is about to Start */
		if (aic31xx->playback_stream) {
			if (!aic31xx->headset_connected) {
#ifdef USE_DRC
				/*DRC enable for speaker path*/
				snd_soc_update_bits(codec, DRC_CTL_REG_1, 0x60,
						(CLEAR | (BIT6 | BIT5)));
#endif

			} else {
#ifdef USE_DRC
				/*Switch off the DRC*/
				snd_soc_update_bits(codec, DRC_CTL_REG_1, 0x60,
						(CLEAR & ~ (BIT6 | BIT5)));
#endif
			}
			snd_soc_update_bits(codec, DAC_MUTE_CTRL_REG,
						0x0C, (CLEAR & ~MUTE_ON));
		}
		aic31xx->power_status = 1;
		aic31xx->mute = 0;
	}
#ifdef AIC31XX_DEBUG
	debug_print_registers(codec);
#endif
	dev_dbg(codec->dev,"##-aic31xx_mute_codec %d\n", mute);

	return 0;
}

/*
 * aic31xx_mute- mute or unmute the left and right DAC
 */
static int aic31xx_mute(struct snd_soc_dai *dai, int mute)
{
	int result = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	dev_dbg(codec->dev,"%s: mute = %d\t priv_mute = %d\n", __func__, mute, aic31xx->mute);

	mutex_lock(&aic31xx->mutex_codec);
	dev_dbg(codec->dev,"%s:lock  mute = %d\t priv_mute = %d\n", __func__, mute, aic31xx->mute);

	/* Check for playback and record status and accordingly
	 * mute or unmute the ADC or the DAC
	 */
	if ((mute == 1) && (codec->active != 0)) {
		if ((aic31xx->playback_stream == 1) &&
					(aic31xx->capture_stream == 1)) {
			dev_warn(codec->dev, "Session still active\n");
			mutex_unlock(&aic31xx->mutex_codec);
		return 0;
		}
	}
	if (aic31xx->capture_stream)
		result = aic31xx_adc_mute(dai->codec, mute);

	if (aic31xx->playback_stream)
		result = aic31xx_dac_mute(dai->codec, mute);

	dev_dbg(codec->dev,"%s: mute = %d\t priv_mute = %d\n", __func__, mute, aic31xx->mute);
	mutex_unlock(&aic31xx->mutex_codec);
	return result;
}

int aic31xx_mute_codec(struct snd_soc_codec *codec, int mute)
{
	int result = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s: mute = %d\t priv_mute = %d=====>\n", __func__, mute, aic31xx->mute);

	mutex_lock(&aic31xx->mutex_codec);

	/* Check for playback and record status and accordingly
	 * mute or unmute the ADC or the DAC
	 */
	if ((mute == 1) && (codec->active != 0)) {
		if ((aic31xx->playback_stream == 1) &&
					(aic31xx->capture_stream == 1)) {
			dev_warn(codec->dev, "Session still active\n");
			mutex_unlock(&aic31xx->mutex_codec);
		return 0;
		}
	}
	if (aic31xx->capture_stream)
		result = aic31xx_adc_mute(codec, mute);

	if (aic31xx->playback_stream)
		result = aic31xx_dac_mute(codec, mute);

	mutex_unlock(&aic31xx->mutex_codec);
	return result;
}
#endif

/*
 * aic31xx_set_dai_sysclk
 *
 * This function is to the DAI system clock
 */
static int aic31xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev,"%s: Entered\n", __func__);
	dev_dbg(codec->dev,"###aic31xx_set_dai_sysclk SysClk %x\n", freq);
	switch (freq) {
	case AIC31XX_FREQ_12000000:
	case AIC31XX_FREQ_24000000:
	case AIC31XX_FREQ_19200000:
		aic31xx->sysclk = freq;
		dev_dbg(codec->dev,"%s: Exiting\n", __func__);
		return 0;
	}
	dev_alert(codec->dev, "Invalid frequency to set DAI system clock\n");
	dev_err(codec->dev,"%s: Exiting with error\n", __func__);
	return -EINVAL;
}

/*
 * aic31xx_set_dai_fmt
 *
 * This function is to set the DAI format
 */
static int aic31xx_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 iface_reg = 0;

	dev_dbg(codec->dev,"%s: Entered\n", __func__);
	dev_dbg(codec->dev,"###aic31xx_set_dai_fmt %x\n", fmt);

	//iface_reg = aic31xx_read(codec, INTERFACE_SET_REG_1);
	//iface_reg = iface_reg & ~(3 << 6 | 3 << 2);

	dev_dbg(codec->dev,"##+ aic31xx_set_dai_fmt (%x)\n", fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic31xx->master = 1;
		iface_reg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic31xx->master = 0;
		iface_reg &= ~(BIT_CLK_MASTER | WORD_CLK_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aic31xx->master = 0;
		iface_reg |= BIT_CLK_MASTER;
		iface_reg &= ~(WORD_CLK_MASTER);
		break;
	default:
		dev_alert(codec->dev, "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface_reg |= (AIC31XX_DSP_MODE << AUDIO_MODE_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg |= (AIC31XX_RIGHT_JUSTIFIED_MODE << AUDIO_MODE_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg |= (AIC31XX_LEFT_JUSTIFIED_MODE << AUDIO_MODE_SHIFT);
		break;
	default:
		dev_alert(codec->dev, "Invalid DAI interface format\n");
		return -EINVAL;
	}

	aic31xx_write(codec, INTERFACE_SET_REG_1, iface_reg);
	dev_dbg(codec->dev,"##-aic31xx_set_dai_fmt Master %d\n", aic31xx->master);
	dev_dbg(codec->dev,"%s: Exiting\n", __func__);

	return 0;
}


/*
 * aic31xx_add_widgets
 *
 * adds all the ASoC Widgets identified by aic31xx_snd_controls array. This
 * routine will be invoked * during the Audio Driver Initialization.
 */
static int aic31xx_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	dev_dbg(codec->dev,"###aic31xx_add_widgets\n");

	ret = snd_soc_dapm_new_controls(dapm, aic31xx_dapm_widgets,
					ARRAY_SIZE(aic31xx_dapm_widgets));
	if (!ret)
		dev_dbg(codec->dev,"#Completed adding dapm widgets size = %d\n", \
					ARRAY_SIZE(aic31xx_dapm_widgets));

	ret = snd_soc_dapm_add_routes(dapm, aic31xx_audio_map,\
					ARRAY_SIZE(aic31xx_audio_map));
	if (!ret)
		dev_dbg(codec->dev,"#Completed adding DAPM routes = %d\n", \
				ARRAY_SIZE(aic31xx_audio_map));

	ret = snd_soc_dapm_new_widgets(dapm);
	if (!ret)
		dev_dbg(codec->dev,"widgets updated\n");

	return 0;
}



/*
 * aic31xx_probe
 *
 * This is first driver function called by the SoC core driver.
 */
static int aic31xx_probe(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx;
	struct i2c_adapter *adapter;
	struct aic31xx_jack_data *jack;
#if defined(ENABLE_HS_BUTTON_PRESS)
	struct input_dev *idev;
#endif
	int ret = 0, i, size = 0;

	dev_dbg(codec->dev,"##aic31xx_probe: AIC31xx Audio Codec %s\n", AIC31XX_VERSION);

	adapter = i2c_get_adapter(3);
	if (!adapter) {
		dev_err(codec->dev,"##Can't get i2c adapter\n");
		ret = -ENODEV;
		return ret;
	}
	dev_dbg(codec->dev,"##i2c_get_adapter success. Creating a new i2c client device..\n");

	tlv320aic31xx_client = i2c_new_device(adapter, \
				&tlv320aic31xx_hwmon_info);
	if (!tlv320aic31xx_client) {
		dev_err(codec->dev,"##can't add i2c device\n");
		ret = -ENODEV;
		return ret;
	}
	dev_dbg(codec->dev,"##i2c_device Pntr %x\n", (unsigned int) tlv320aic31xx_client);

	codec->control_data = (void *)tlv320aic31xx_client;

	dev_dbg(codec->dev,"##Codec CntrlData %x\n", (unsigned int) codec->control_data);

	aic31xx = kzalloc(sizeof(struct aic31xx_priv), GFP_KERNEL);
	mutex_init(&aic31xx->mutex_page);

	if (aic31xx == NULL) {
		dev_err(codec->dev,"aic31xx_probe kzalloc for Codec Private failed..\n");
		return -ENOMEM;
	}
	dev_dbg(codec->dev,"aic31xx_probe: Codec Private allocation fine...\n");

	snd_soc_codec_set_drvdata(codec, aic31xx);

	codec->hw_write = (hw_write_t) i2c_master_send;
	//codec->hw_read = (hw_read_t) i2c_master_recv;
	codec->read = aic31xx_read;
	codec->write = aic31xx_write;

	/* use switch-class based headset reporting if platform requires it */
	jack = &aic31xx->hs_jack;
	jack->sdev.name = "h2w";
	ret = switch_dev_register(&jack->sdev);
	if (ret) {
		dev_err(codec->dev, "error registering switch device %d\n", ret);
		goto reg_err;
	}
#if defined(ENABLE_HS_BUTTON_PRESS)
	idev = input_allocate_device();
	aic31xx->idev = idev;
	if(idev > 0) {
		input_set_capability(idev, EV_KEY, KEY_MEDIA);
		ret = input_register_device(idev);
		if (ret < 0){
			dev_err(codec->dev,"register input dev fail\n");
			goto reg_err;

		}
	}
#endif

	aic31xx->page_no = 0;
	aic31xx->power_status = 0;
	aic31xx->mute = 1;
	aic31xx->headset_connected = 0;
	aic31xx->playback_status = 0;
	aic31xx->headset_current_status = 0;
	aic31xx->i2c_regs_status = 0;
	aic31xx->from_resume = 0;

	dev_dbg(codec->dev,"##Writing default values to Codec Regs..\n");

	aic31xx_change_page(codec, 0x00);
	aic31xx_write(codec, RESET, 0x01);
	mdelay(10);

	for (i = 0; i < sizeof(aic31xx_reg_init)/sizeof(struct aic31xx_configs);
									  i++) {
		aic31xx_write(codec, aic31xx_reg_init[i].reg_offset,\
					aic31xx_reg_init[i].reg_val);
		mdelay(5);
	}
#ifdef AIC31xx_CODEC_DEBUG
	debug_print_registers(codec);
#endif
	/* Power-off the DAC and Headphone Drivers initially */
	aic31xx->power_status = 1;
	aic31xx->headset_connected = 1;
	aic31xx->mute = 0;
	aic31xx_dac_mute(codec, 1);
	aic31xx->headset_connected = 0;

	dev_dbg(codec->dev,"##Switching the Codec to STANDBY State...\n");

	size = ARRAY_SIZE(aic31xx_snd_controls);
	ret = snd_soc_add_controls(codec, aic31xx_snd_controls,
			ARRAY_SIZE(aic31xx_snd_controls));

	dev_dbg(codec->dev,"##snd_soc_add_controls: ARRAY SIZE : %d\n", size);
		aic31xx_add_widgets(codec);

	mutex_init(&aic31xx->mutex_codec);

#ifdef CONFIG_MINIDSP
	aic3111_minidsp_program(codec);
	aic3111_add_minidsp_controls(codec);
#endif

	dev_dbg(codec->dev,"%s: Exiting : %d\n", __func__, ret);

	return ret;

reg_err:
	kfree(aic31xx);
	return ret;
}

/*
 * aic31xx_remove
 *
 * This function is used to unregister the Driver.
 */
static int aic31xx_remove(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
#if defined(ENABLE_HS_BUTTON_PRESS)
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
#endif
	dev_dbg(codec->dev,"aic31xx_remove ...\n");

	/* power down chip */
	if (codec->control_data)
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);
#if defined(ENABLE_HS_BUTTON_PRESS)
	input_unregister_device(aic31xx->idev);
#endif
#ifdef NO_PCMS
	snd_soc_free_pcms(socdev);
#endif
	snd_soc_dapm_free(dapm);
	kfree(snd_soc_codec_get_drvdata(codec));

	return 0;
}

/*
 * aic31xx_suspend
 * This function is to suspend the AIC31xx driver.
 */
static int aic31xx_suspend(struct snd_soc_codec *codec,	pm_message_t state)
{
	int val, lv, rv;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	dev_dbg(codec->dev,"%s: Entered\n", __func__);

	if (aic31xx->playback_status == 0) {

		lv = aic31xx_read(codec,L_ANLOG_VOL_2_HPL);
		lv |= 0x7f;
		rv = aic31xx_read(codec, R_ANLOG_VOL_2_HPR);
		rv |= 0x7f;
		val = lv ^ ((lv ^ rv) & -(lv < rv));
		while(val > 0) {
			aic31xx_write(codec, L_ANLOG_VOL_2_HPL, val);
			mdelay(1);
			aic31xx_write(codec, R_ANLOG_VOL_2_HPR, val);
			val--;
			mdelay(1);
		}
		aic31xx->from_resume = 0;
		aic31xx_mute_codec(codec, 1);
		aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

		/* Bit 7 of Page 1/ Reg 46 gives the soft powerdown control.
		 * Setting this bit will further reduces the amount of power
		 * consumption
		 */
		val = aic31xx_read(codec, MICBIAS_CTRL);
		aic31xx_write(codec, MICBIAS_CTRL, val | BIT7);

		/*Switching off the headphone driver*/
		val=aic31xx_read(codec, HEADPHONE_DRIVER);
		val &= (~(BIT7));
		val &= (~(BIT6));
		aic31xx_write(codec, HEADPHONE_DRIVER,val);

		/* Disable Audio clock from FREF_CLK2_OUT */
		omap_writew(omap_readw(0x4a30a318) & 0xFEFF, 0x4a30a318);

		regulator_disable(audio_regulator);
	}
	dev_dbg(codec->dev,"%s: Exiting\n", __func__);
	return 0;
}

/*
 * aic31xx_resume
 * This function is to resume the AIC31xx driver.
 */
static int aic31xx_resume(struct snd_soc_codec *codec)
{
	/*struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);*/
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct aic31xx_priv *priv = snd_soc_codec_get_drvdata(codec);
	u8 val;

	dev_dbg(codec->dev,"###aic31xx_resume\n");
	dev_dbg(codec->dev,"%s: Entered\n", __func__);

	if (regulator_set_voltage(audio_regulator, REGU_MIN_VOL, REGU_MAX_VOL))
		dev_warn(codec->dev, "%s: regulator_set 3V error\n", __func__);

	regulator_enable(audio_regulator);

	/* Enable Audio clock from FREF_CLK2_OUT */
	omap_writew(omap_readw(0x4a30a318) | ~0xFEFF, 0x4a30a318);


	/* Switching on the headphone driver*/
	val=aic31xx_read(codec,HEADPHONE_DRIVER);
	val |=  (BIT7|BIT6);
	aic31xx_write(codec,HEADPHONE_DRIVER, val);

	priv->from_resume = 1;

	val = aic31xx_read(codec, MICBIAS_CTRL);
	aic31xx_write(codec, MICBIAS_CTRL, val & ~BIT7);

	/* Check headset jack and re-configure proper routing */
	val = !!gpio_get_value(Qoo_HEADSET_DETECT_GPIO_PIN);

	switch_set_state(&priv->hs_jack.sdev, !val);
	dev_dbg(codec->dev,"## %s : switch_state = %d n", __func__, !val);

	dev_dbg(codec->dev,"%s: Exiting\n", __func__);

	return 0;
}
#if 0
/*
 * aic31xx_get_record_status
 *
 * Helper function which checks if the Record Path is
 * active within the Audio Codec Driver. This function
 * returns the following:
 * 0	- No Recording Session in Progress or Recording stopped
 * 1	- Recording session is active and in progress
 */
int aic31xx_get_record_status(void)
{
	/* Check the global Codec Priv Struct */
	if (unlikely(aic31xx == 0))
		return 0;

	/* Check the Priv Struct power_status and
	 * capture_status variable along with mute
	 * All these three will let us know if recording is
	 * active and in progress.
	 */
	if (aic31xx->power_status && aic31xx->capture_stream &&
			aic31xx->mute == 0)
		return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(aic31xx_get_record_status);
#endif


/**
 * @struct snd_soc_codec_dev_aic31xx
 *
 * This structure is soc audio codec device sturecute which pointer * to basic
 * functions aic31xx_probe(), aic31xx_remove(), aic31xx_suspend() and
 * aic31xx_resume()
 */
struct snd_soc_codec_driver soc_codec_dev_aic31xx = {
	.probe = aic31xx_probe,
	.remove = aic31xx_remove,
	.suspend = aic31xx_suspend,
	.resume = aic31xx_resume,
	.read = aic31xx_read,
	.write = aic31xx_write,
	.set_bias_level = aic31xx_set_bias_level,
	.reg_cache_size = sizeof(aic31xx_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = aic31xx_reg,
};

/**
 * @struct tlv320aic31xx_dai_ops
 *
 * The DAI Operations Structure which contains the call-back function
 * Addresses used by the ALSA Core Layer when operating with this Codec
 * Driver.
 */
static struct snd_soc_dai_ops tlv320aic31xx_dai_ops = {
	.hw_params = aic31xx_hw_params,
	.digital_mute = aic31xx_mute,
	.set_sysclk = aic31xx_set_dai_sysclk,
	.set_fmt = aic31xx_set_dai_fmt,
};

/**
 * @struct tlv320aic31xx_dai
 *
 * SoC Codec DAI driver which has DAI capabilities viz., playback and
 * capture, DAI runtime information viz. state of DAI and pop wait
 * state, and DAI private data.  The AIC31xx rates ranges from 8k to
 * 192k The PCM bit format supported are 16, 20, 24 and 32 bits
 */
struct snd_soc_dai_driver tlv320aic31xx_dai[] = {
	{
		.name = "tlv320aic3110-MM_EXT",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC31XX_RATES,
			.formats = AIC31XX_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC31XX_RATES,
			.formats = AIC31XX_FORMATS,
		},
		.ops =  &tlv320aic31xx_dai_ops,
	},
};


/*
 * tlv320aic31xx_codec_probe
 * This function is invoked by the soc_probe of the ALSA Core Layer during the
 * execution of Core Layer Initialization. This function is used to register the
 * Audio Codec with the ALSA Core Layer using the snd_soc_register_codec API.
 */
static int __devinit tlv320aic31xx_codec_probe(struct platform_device *pdev)
{

	int ret;
	int err;
	dev_dbg(&pdev->dev,"Came to tlv320aic31xx_codec_probe...\n");


	audio_regulator = regulator_get(NULL, "audio-pwr");
	if (IS_ERR(audio_regulator))
		dev_err(&pdev->dev,"%s: regulator_get error\n", __func__);

	err = regulator_set_voltage(audio_regulator, REGU_MIN_VOL,
						REGU_MAX_VOL);
	if (err)
		dev_crit(&pdev->dev,"%s: regulator_set 3V error\n", __func__);

	regulator_enable(audio_regulator);
	ret =  snd_soc_register_codec(&pdev->dev,
		&soc_codec_dev_aic31xx, tlv320aic31xx_dai, \
		ARRAY_SIZE(tlv320aic31xx_dai));

	dev_info(&pdev->dev,"snd_soc_register_codec returned %d\n", ret);
	return ret;
}

/*
 * aic31xx_codec_remove
 * This function is to unregister the Audio Codec from the ALSA Core Layer.
 */
static int __devexit aic31xx_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	regulator_disable(audio_regulator);
	regulator_put(audio_regulator);
	return 0;
}

/*
 * @struct tlv320aic31xx_i2c_driver
 *
 * Platform Driver structure used to describe the Driver structure.
 */
static struct platform_driver tlv320aic31xx_i2c_driver = {
	.driver = {
		.name = "tlv320aic3110-codec",
		.owner = THIS_MODULE,
	},
	.probe = tlv320aic31xx_codec_probe,
	.remove = __devexit_p(aic31xx_codec_remove),
};

/*
 * tlv320aic31xx_init
 * This function is as MODULE_INIT Routine for the Codec Driver.
 */
static int __init tlv320aic31xx_init(void)
{
	int ret;
	dev_info(NULL, "##Came to Codec DRiver Init routine...\n");

	ret = platform_driver_register(&tlv320aic31xx_i2c_driver);

	if (ret != 0) {
		dev_crit(NULL,"Failed to register TLV320AIC31xx I2C driver: %d\n", ret);
		return ret;
	}

	dev_info(NULL, "tlv320aic31xx_init success !!!\n");
	return ret;
}
module_init(tlv320aic31xx_init);

/*
 * tlv320aic31aic31xx_exit
 * This function is as MODULE_EXIT Routine for the Codec Driver.
 */
static void __exit tlv320aic31xx_exit(void)
{
	dev_info(NULL, "tlv320aic3xx_exit....\n");
	platform_driver_unregister(&tlv320aic31xx_i2c_driver);
}

module_exit(tlv320aic31xx_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3100 codec driver");
MODULE_AUTHOR("Ravindra <ravindra@mistralsolutions.com>");
MODULE_LICENSE("GPL");
