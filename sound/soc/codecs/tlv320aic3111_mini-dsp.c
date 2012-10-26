/*
 * linux/sound/soc/codecs/tlv320aic3111_mini-dsp.c
 *
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1 	 mini DSP support    		Mistral         22-06-2010
 *
 *          The mini DSP programming support is added to codec AIC3111.
 *
 *
 * Rev 1.0 	 mini DSP mixer controls 	Mistral         24-06-2010
 *
 *         	 Added mixer controls for aic3111 codec, and code cleanup
 *
 * Rev 1.1 	 TiLoad support         	Mistral         25-06-2010
 *
 *         	 Added char driver for TiLoad support, and mixer controls
 *         	 for adaptive filtering and coefficient buffer controls
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/control.h>

#include "tlv320aic31xx.h"
#include "tlv320aic3111_mini-dsp.h"
/* enable debug prints in the driver */

#undef REG_DUMP_MINIDSP

/* Function prototypes */
#ifdef REG_DUMP_MINIDSP
static void aic3111_dump_page(struct i2c_client *i2c, u8 page);
#endif /* REG_DUMP_MINIDSP */

/* externs */
extern int aic31xx_change_page(struct snd_soc_codec *codec, u8 new_page);
extern int aic31xx_write_locked(struct snd_soc_codec *codec, u16 reg, u8 value);
extern int aic31xx_mute_codec(struct snd_soc_codec *codec, int mute);

/*local static declarations*/
static int minidsp_driver_init(struct snd_soc_codec *codec);
static int aic3111_minidsp_load_process_flow(struct snd_soc_codec *codec, int mode);
static int mix_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo);
static int mix_control_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);
static int mix_control_put(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol);



 static unsigned int magic_num;

 #define AIC3111_MINIDSP_CREATE_SECTION(section_name, section_program,\
	 val_areg, val_dreg, muxctl, muxctl_name, volctl, volctl_name)\
 {\
	.reg_section_name =  section_name,\
	.reg_section_program =  section_program,\
	.reg_section_values_a_reg = val_areg,\
	.reg_section_values_d_reg = val_dreg,\
	.mux_ctl = muxctl,\
	.mux_ctl_name = muxctl_name,\
	.vol_ctl = volctl,\
	.vol_ctl_name = volctl_name,\
	.size_of_reg_section_name = (unsigned int) ARRAY_SIZE(section_name),\
	.size_of_reg_section_program = (unsigned int)ARRAY_SIZE(section_program),\
	.size_of_values_a_reg	= (unsigned int) ARRAY_SIZE(val_areg),\
	.size_of_values_d_reg	= (unsigned int) ARRAY_SIZE(val_dreg),\
	.size_of_mux_ctl = (unsigned int) ARRAY_SIZE(muxctl),\
	.size_of_mux_ctl_name = (unsigned int) ARRAY_SIZE(muxctl_name),\
	.size_of_vol_ctl = (unsigned int) ARRAY_SIZE(volctl),\
	.size_of_vol_ctl_name = (unsigned int)  ARRAY_SIZE(volctl_name),\
 }


 static	minidsp_config_sections dsp_config[] = {

	 AIC3111_MINIDSP_CREATE_SECTION(Second_Rate_REG_Section_names,
			 Second_Rate_REG_Section_program,
			 Second_Rate_miniDSP_A_reg_values,
			 Second_Rate_miniDSP_D_reg_values,
			 Second_Rate_MUX_controls,
			 Second_Rate_MUX_control_names,
			 Second_Rate_VOLUME_controls,
			 Second_Rate_VOLUME_control_names),

	 AIC3111_MINIDSP_CREATE_SECTION(Srs_REG_Section_names,
			 Srs_REG_Section_program,
			 Srs_miniDSP_A_reg_values,
			 Srs_miniDSP_D_reg_values,
			 Srs_MUX_controls,
			 Srs_MUX_control_names,
			 Srs_VOLUME_controls,
			 Srs_VOLUME_control_names ),

	 AIC3111_MINIDSP_CREATE_SECTION(HS_REG_Section_names,
			 HS_REG_Section_program,
			 HS_miniDSP_A_reg_values,
			 HS_miniDSP_D_reg_values,
			 HS_MUX_controls,
			 HS_MUX_control_names,
			 HS_VOLUME_controls,
			 HS_VOLUME_control_names ),

#if 0

	 AIC3111_MINIDSP_CREATE_SECTION(REG_Section_names,
			 REG_Section_program,
			 miniDSP_A_reg_values,
			 miniDSP_D_reg_values,
			 MUX_controls,
			 MUX_control_names,
			 VOLUME_controls,
			 VOLUME_control_names ),


	 AIC3111_MINIDSP_CREATE_SECTION(Loud_REG_Section_names,
			 Loud_REG_Section_program,
			 Loud_miniDSP_A_reg_values,
			 Loud_miniDSP_D_reg_values,
			 Loud_MUX_controls,
			 Loud_MUX_control_names,
			 Loud_VOLUME_controls,
			 Loud_VOLUME_control_names ),
#endif
	 /*
	* If you add new section then define the corresponding mux/volume control.
	*/

	};

#define MAX_DSP_CONFIG	(ARRAY_SIZE(dsp_config))
/******************************** Debug section *****************************/

#ifdef REG_DUMP_MINIDSP
/*
 *----------------------------------------------------------------------------
 * Function : aic3111_dump_page
 * Purpose  : Read and display one codec register page, for debugging purpose
 *----------------------------------------------------------------------------
 */
static void aic3111_dump_page(struct i2c_client *i2c, u8 page)
{
	int i;
	u8 data;
	u8 test_page_array[256];
	struct snd_soc_codec *codec;

	aic31xx_change_page(codec, page);

	data = 0x0;

	i2c_master_send(i2c, data, 1);
	i2c_master_recv(i2c, test_page_array, 128);

	dev_dbg(codec->dev,"\n------- MINI_DSP PAGE %d DUMP --------\n", page);
	for (i = 0; i < 128; i++) {
		dev_dbg(codec->dev," [ %d ] = 0x%x\n", i, test_page_array[i]);
	}
}
#endif /* REG_DUMP_MINIDSP */

/******************** MINI DSP Static Programming section *******************/

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_minidsp_get_burst
 * Purpose  : Format one I2C burst for transfer from mini dsp program array.
 * 			  This function will parse the program array and get next burst
 * 			  data for doing an I2C bulk transfer.
 *----------------------------------------------------------------------------
 */
static void aic3111_minidsp_get_burst(reg_value * program_ptr, int program_size,
				      minidsp_parser_data * parse_data)
{
	int index = parse_data->current_loc;
	int burst_write_count = 0;

	/* check if first location is page register, and populate page addr */
	if (program_ptr[index].reg_off == 0) {
		parse_data->page_num = program_ptr[index].reg_val;
		index++;
	}

	parse_data->burst_array[burst_write_count++] =
	    program_ptr[index].reg_off;
	parse_data->burst_array[burst_write_count++] =
	    program_ptr[index].reg_val;
	index++;

	for (; index < program_size; index++) {
		if (program_ptr[index].reg_off !=
		    (program_ptr[index - 1].reg_off + 1))
			break;
		else {
			parse_data->burst_array[burst_write_count++] =
			    program_ptr[index].reg_val;
		}
	}

	parse_data->burst_size = burst_write_count;

	if (index == program_size) {
		/* parsing completed */
		parse_data->current_loc = MINIDSP_PARSING_END;
	} else
		parse_data->current_loc = index;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_minidsp_write_burst
 *
 * Purpose  : Write one I2C burst to the codec registers. The buffer for burst
 * 			  transfer is given by aic3111_minidsp_get_burst() function.
 *----------------------------------------------------------------------------
 */
static int aic3111_minidsp_write_burst(struct snd_soc_codec *codec,
				       minidsp_parser_data * parse_data)
{
#ifdef VERIFY_MINIDSP
	int i;
	char read_addr;
	char test_page_array[256];
#endif

	struct i2c_client *i2c = codec->control_data;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&aic31xx->mutex_page);

	aic31xx_change_page(codec, parse_data->page_num);

	/* write burst data */
	if ((i2c_master_send(i2c, parse_data->burst_array,
	    parse_data->burst_size)) != parse_data->burst_size) {
		dev_dbg(codec->dev,"Mini DSP: i2c_master_send failed\n");
		ret = -1;
		goto end;
	}

#ifdef VERIFY_MINIDSP
	read_addr = parse_data->burst_array[0];
	i2c_master_send(i2c, &read_addr, 1);

	if ((i2c_master_recv(i2c, test_page_array, parse_data->burst_size))
	    != parse_data->burst_size) {
		dev_err(codec->dev,"Mini DSP: i2c_master_recv failed\n");
		ret = -1;
		goto end;
	}

	for (i = 0; i < parse_data->burst_size - 1; i++) {
		if (test_page_array[i] != parse_data->burst_array[i + 1]) {
			dev_err(codec->dev,
			    "MINI DSP program verification failure on page 0x%x\n",
			     parse_data->page_num);
			ret = -1;
			goto end;
		}
	}
	dev_info(codec->dev,"MINI DSP program verification success on page 0x%x\n",
		parse_data->page_num);
#endif /* VERIFY_MINIDSP */

end:
	mutex_unlock(&aic31xx->mutex_page);
	return ret;
}


/*
 *----------------------------------------------------------------------------
 * Function : minidsp_program
 * Purpose  : Program mini dsp instructions and/or coeffients
 *----------------------------------------------------------------------------
 */
static int minidsp_program(struct snd_soc_codec *codec,
			   reg_value * program_ptr, int program_size)
{
	minidsp_parser_data parse_data;

	/* point the current location to start of program array */
	parse_data.current_loc = 0;

	do {
		/* Get first burst data */
		aic3111_minidsp_get_burst(program_ptr, program_size,
					  &parse_data);

		dev_dbg(codec->dev,"Burst,PAGE=0x%x Size=%d\n", parse_data.page_num,
			parse_data.burst_size);

		/* Write one burst to the MINI DSP register space */
		aic3111_minidsp_write_burst(codec, &parse_data);

		/* Proceed to the next burst reg_addr_incruence */
	} while (parse_data.current_loc != MINIDSP_PARSING_END);

	return 0;
}

/********************* CODEC REGISTER PROGRAMMING ***********************/

/*
 * There may be instance where the same register may be programmed by the
 * default ASoC driver and when programming the registers in the PPS
 * configuration file. The register number in the below array indicates which
 * registers in the PPS config file will be written. Others will be ignored
 */

/* Allowed registers for page '0' */
static u8 page_0_allow_regs[] = { 21, 22, 15, 16, 60, 61, 17, 255, 23, 35 };

/* Allowed registers for page '1' */
static u8 page_1_allow_regs[] = { };

/*
 *----------------------------------------------------------------------------
 * Function : check_allow_list
 *
 * Purpose  : Check and return if the register is allowed to write.
 * 			  Allowed registers are store in an arry per page. The register
 * 			  and page address passed are checked against this list and
 * 			  return whether to ignore or not
 *----------------------------------------------------------------------------
 */
static int check_allow_list(u8 page, u8 reg_addr)
{
	int i;

	if (page == 0) {
		for (i = 0; i < sizeof(page_0_allow_regs); i++) {
			if (page_0_allow_regs[i] == reg_addr)
				/* allow register write */
				return (CODEC_REG_DONT_IGNORE);
		}
		/* not in allow list, ignore the register */
		return (CODEC_REG_IGNORE);
	} else if (page == 1) {
		for (i = 0; i < sizeof(page_1_allow_regs); i++) {
			if (page_1_allow_regs[i] == reg_addr)
				/* allow register write */
				return (CODEC_REG_DONT_IGNORE);

		}
		/* not in allow list, ignore the register */
		return (CODEC_REG_IGNORE);
	} else {
		/* Write to pages other than 0,1 are allowed */
		return (CODEC_REG_DONT_IGNORE);
	}
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_codec_reg_program
 *
 * Purpose  : Program codec registers, from the codec register section
 * 			  of mini dsp program header file.
 *----------------------------------------------------------------------------
 */
static void minidsp_codec_reg_program(struct snd_soc_codec *codec,
				      reg_value * reg_array, int program_size,
				      int pre_post)
{
	int i, ret;
	u8 current_page = 0;
	u16 reg_addr;
	static int stored_index = 0x0;
	static u8 stored_page = 0x0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&aic31xx->mutex_page);

	/* If post init is defined, start writing
	 * codec registers from previously stored
	 * array index and page, else from beginning
	 */
	if (pre_post == CODEC_REG_POST_INIT) {
		i = stored_index;
		aic31xx_change_page(codec, stored_page);
		dev_info(codec->dev, "%s: Changing page\n", __func__);
	} else {
		i = 0;
	}

	/* parse the register array */
	for (; i < program_size; i++) {
		/* check if page number is changed */
		if (reg_array[i].reg_off == 0x0) {
			current_page = reg_array[i].reg_val;
			aic31xx_change_page(codec, current_page);
			dev_info(codec->dev, "#%s: Switching to Page %d\n",
					 __func__, current_page);
		} else {
			/*  Initialization is prior to MINI DSP programming
			 *      stop register writes, at the delimiter register, and
			 *      continue after DSP initialization is complete
			 */
			if ((pre_post == CODEC_REG_PRE_INIT) &&
			    (reg_array[i].reg_off == INIT_SEQ_DELIMITER)) {
				/*
				 * Stop writing codec registers and
				 * move index to next valid register (2 entries ahead)
				 * and store index and page number statically
				 * and return back
				 */
				stored_index = i + DELIMITER_COUNT;
				stored_page = current_page;
				goto end;
			}

			/* Check the register to be ignored or written */
			ret = check_allow_list(current_page,
					     reg_array[i].reg_off);

			if (ret == CODEC_REG_DONT_IGNORE) {
				dev_dbg(codec->dev,"Writing page 0x%x reg %d = 0x%x\n",
						current_page,
						reg_array[i].reg_off,
						reg_array[i].reg_val);
				/* convert addr to 16 bit address, by adding page offset */
				reg_addr =
					(current_page * 128) + reg_array[i].reg_off;

				ret =
					aic31xx_write_locked(codec, reg_addr,
							reg_array[i].reg_val);
				if (ret) {
					dev_err(codec->dev,
							"Write failed for register %d\n",
							reg_array[i].reg_off);
				}
			}
		}
	}
end:
	mutex_unlock(&aic31xx->mutex_page);
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3111_minidsp_program
 *
 * Purpose  : Program mini dsp for AIC3111 codec chip. This routine is
 * 		  called from the aic3111 codec driver, if mini dsp programming
 * 		  is enabled.
 *----------------------------------------------------------------------------
 */
extern int aic3111_minidsp_program(struct snd_soc_codec *codec)
{
	int ret;
	dev_dbg(codec->dev,"AIC3111: programming mini dsp\n");
	/*codec clk initialization done at pfw 0*/
	aic3111_minidsp_load_process_flow(codec, 0);
	ret = minidsp_driver_init(codec);
	return ret;
}


/*
 *----------------------------------------------------------------------------
 * Function : aic3111_minidsp_program
 *
 * Purpose  : Program mini dsp for AIC3111 codec chip. This routine is
 * 		  called from the aic3111 codec driver, if mini dsp programming
 * 		  is enabled.
 *----------------------------------------------------------------------------
 */
static int aic3111_minidsp_load_process_flow(struct snd_soc_codec *codec, int mode)
{
	int i;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	if(  aic31xx->current_process_flow < 0 ||
			aic31xx->current_process_flow > (MAX_DSP_CONFIG-1))
	{
		/* Default set to Speaker Pfw */
		aic31xx->current_process_flow = 1;
		mode = 1;
	}

	dev_info(codec->dev,"AIC3111: programming mini dsp\n");
	dev_dbg(codec->dev,"The register sections found in pps header file:\n");
	for (i = 0; i < dsp_config[mode].size_of_reg_section_name; i++) {
		dev_info(codec->dev,"%s\n", dsp_config[mode].reg_section_name[i]);
	}

#ifdef PROGRAM_CODEC_REG_SECTIONS
	/* Array size should be greater than 1 to start programming,
	 * since first write command will be the page register
	 */

	dev_dbg(codec->dev, "#%s: Reg Section Size %d\n", __func__,
			dsp_config[mode].size_of_reg_section_program);

	if (dsp_config[mode].size_of_reg_section_program > 1) {
		minidsp_codec_reg_program(codec, dsp_config[mode].reg_section_program,
				dsp_config[mode].size_of_reg_section_program,
				CODEC_REG_PRE_INIT);
	} else {
		dev_warn(codec->dev, "CODEC_REGS: Insufficient data for programming\n");
	}
#endif /* PROGRAM_CODEC_REG_SECTIONS */

#ifdef PROGRAM_MINI_DSP_A
	if (dsp_config[mode].size_of_values_a_reg > 1) {
		minidsp_program(codec, dsp_config[mode].reg_section_values_a_reg,
				dsp_config[mode].size_of_values_a_reg);
	} else {
		dev_warn(codec->dev, "MINI_DSP_A: Insufficient data for programming\n");
	}
#endif /* PROGRAM_MINI_DSP_A */

#ifdef PROGRAM_MINI_DSP_D
	if (dsp_config[mode].size_of_values_d_reg > 1) {
		minidsp_program(codec, dsp_config[mode].reg_section_values_d_reg,
				dsp_config[mode].size_of_values_d_reg);
		dev_info(codec->dev, "mini_dsp_d reg values are programmed successfully\n");
	} else {
		dev_warn(codec->dev,"MINI_DSP_D: Insufficient data for programming\n");
	}
#endif /* PROGRAM_MINI_DSP_D */

#ifdef PROGRAM_CODEC_REG_POST_SECTIONS
	if (dsp_config[mode].size_of_reg_section_program > 1) {
		minidsp_codec_reg_program(codec, dsp_config[mode].reg_section_program,
				dsp_config[mode].size_of_reg_section_program,
				CODEC_REG_POST_INIT);
	} else {
		dev_dbg(codec->dev,"CODEC_REGS: Insufficient data for programming\n");
	}
#endif /* PROGRAM_CODEC_REG_POST_SECTIONS */
	aic31xx->current_process_flow = mode;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : set_minidsp_mode
 * Purpose  : Switch to the first minidsp mode.
 *----------------------------------------------------------------------------
 */
	int
set_minidsp_mode(struct snd_soc_codec *codec, int new_mode)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int current_mode = aic31xx->current_process_flow;
	dev_info(codec->dev,"%s: switch mode started\n", __func__);
	if(current_mode == new_mode)
	{
		dev_info(codec->dev,"%s: switch mode not required\n", __func__);
		return 0;
	}

	aic3111_minidsp_load_process_flow(codec, new_mode);
	dev_info(codec->dev, "processflow loading finished\n");
	return 0;
}

/********************* AMIXER Controls for mini dsp *************************/

#ifdef ADD_MINI_DSP_CONTROLS

#if defined(USE_VOLUME_CONTROLS)
/* Volume Lite coefficents table */
static int volume_lite_table[] = {
	0x00000D, 0x00000E, 0x00000E, 0x00000F,
	0x000010, 0x000011, 0x000012, 0x000013,
	0x000015, 0x000016, 0x000017, 0x000018,
	0x00001A, 0x00001C, 0x00001D, 0x00001F,
	0x000021, 0x000023, 0x000025, 0x000027,
	0x000029, 0x00002C, 0x00002F, 0x000031,
	0x000034, 0x000037, 0x00003B, 0x00003E,
	0x000042, 0x000046, 0x00004A, 0x00004F,
	0x000053, 0x000058, 0x00005D, 0x000063,
	0x000069, 0x00006F, 0x000076, 0x00007D,
	0x000084, 0x00008C, 0x000094, 0x00009D,
	0x0000A6, 0x0000B0, 0x0000BB, 0x0000C6,
	0x0000D2, 0x0000DE, 0x0000EB, 0x0000F9,
	0x000108, 0x000118, 0x000128, 0x00013A,
	0x00014D, 0x000160, 0x000175, 0x00018B,
	0x0001A3, 0x0001BC, 0x0001D6, 0x0001F2,
	0x000210, 0x00022F, 0x000250, 0x000273,
	0x000298, 0x0002C0, 0x0002E9, 0x000316,
	0x000344, 0x000376, 0x0003AA, 0x0003E2,
	0x00041D, 0x00045B, 0x00049E, 0x0004E4,
	0x00052E, 0x00057C, 0x0005D0, 0x000628,
	0x000685, 0x0006E8, 0x000751, 0x0007C0,
	0x000836, 0x0008B2, 0x000936, 0x0009C2,
	0x000A56, 0x000AF3, 0x000B99, 0x000C49,
	0x000D03, 0x000DC9, 0x000E9A, 0x000F77,
	0x001062, 0x00115A, 0x001262, 0x001378,
	0x0014A0, 0x0015D9, 0x001724, 0x001883,
	0x0019F7, 0x001B81, 0x001D22, 0x001EDC,
	0x0020B0, 0x0022A0, 0x0024AD, 0x0026DA,
	0x002927, 0x002B97, 0x002E2D, 0x0030E9,
	0x0033CF, 0x0036E1, 0x003A21, 0x003D93,
	0x004139, 0x004517, 0x00492F, 0x004D85,
	0x00521D, 0x0056FA, 0x005C22, 0x006197,
	0x006760, 0x006D80, 0x0073FD, 0x007ADC,
	0x008224, 0x0089DA, 0x009205, 0x009AAC,
	0x00A3D7, 0x00B7D4, 0x00AD8C, 0x00C2B9,
	0x00CE43, 0x00DA7B, 0x00E76E, 0x00F524,
	0x0103AB, 0x01130E, 0x01235A, 0x01349D,
	0x0146E7, 0x015A46, 0x016ECA, 0x018486,
	0x019B8C, 0x01B3EE, 0x01CDC3, 0x01E920,
	0x02061B, 0x0224CE, 0x024553, 0x0267C5,
	0x028C42, 0x02B2E8, 0x02DBD8, 0x030736,
	0x033525, 0x0365CD, 0x039957, 0x03CFEE,
	0x0409C2, 0x044703, 0x0487E5, 0x04CCA0,
	0x05156D, 0x05628A, 0x05B439, 0x060ABF,
	0x066666, 0x06C77B, 0x072E50, 0x079B3D,
	0x080E9F, 0x0888D7, 0x090A4D, 0x09936E,
	0x0A24B0, 0x0ABE8D, 0x0B6188, 0x0C0E2B,
	0x0CC509, 0x0D86BD, 0x0E53EB, 0x0F2D42,
	0x101379, 0x110754, 0x1209A3, 0x131B40,
	0x143D13, 0x157012, 0x16B543, 0x180DB8,
	0x197A96, 0x1AFD13, 0x1C9676, 0x1E481C,
	0x201373, 0x21FA02, 0x23FD66, 0x261F54,
	0x28619A, 0x2AC625, 0x2D4EFB, 0x2FFE44,
	0x32D646, 0x35D96B, 0x390A41, 0x3C6B7E,
	0x400000, 0x43CAD0, 0x47CF26, 0x4C106B,
	0x50923B, 0x55586A, 0x5A6703, 0x5FC253,
	0x656EE3, 0x6B7186, 0x71CF54, 0x788DB4,
	0x7FB260,
};

/************************ VolumeLite control section ************************/
static struct snd_kcontrol_new snd_vol_controls[MAX_VOLUME_CONTROLS];

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_volume
 *
 * Purpose  : info routine for volumeLite amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_volume(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	int index, mode;
	int ret_val = -1;

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	mode = aic31xx->current_process_flow;
	dev_dbg(codec->dev,KERN_INFO "%s: mode=%d", "info_mdsp_vol", mode);
	for (index = 0; index < dsp_config[mode].size_of_vol_ctl; index++) {
		if (strstr(kcontrol->id.name,
			dsp_config[mode].vol_ctl_name[index]))
			break;
	}

	if (index < dsp_config[mode].size_of_vol_ctl) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_VOLUME;
		uinfo->value.integer.max = MAX_VOLUME;
		ret_val = 0;
	}

	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_vol
 *
 * Purpose  : get routine for amixer kcontrols, read current register
 * 		  values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_volume(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_volume
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 * 		  values. Used for for mini dsp 'VolumeLite' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_volume(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	u8 data[5];
	int index, mode;
	struct i2c_client *i2c;
	int ret_val = -1;
	int coeff;
	int user_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 value[2], swap_reg_pre, swap_reg_post;

	mode = aic31xx->current_process_flow;
	user_value  = ucontrol->value.integer.value[0];
	i2c = codec->control_data;

	dev_dbg(codec->dev,"user value = 0x%x\n", user_value);

	mutex_lock(&aic31xx->mutex_page);

	for (index = 0; index < dsp_config[mode].size_of_vol_ctl; index++) {
		if (strstr(kcontrol->id.name,
			 dsp_config[mode].vol_ctl_name[index]))
			break;
	}

	if (index < dsp_config[mode].size_of_vol_ctl) {
		aic31xx_change_page(codec,
		 dsp_config[mode].vol_ctl[index].control_page);

		coeff = volume_lite_table[user_value << 1];

		data[1] = (u8) ((coeff >> 16) & AIC31XX_8BITS_MASK);
		data[2] = (u8) ((coeff >> 8) & AIC31XX_8BITS_MASK);
		data[3] = (u8) ((coeff) & AIC31XX_8BITS_MASK);

		/* Start register address */
		data[0] = dsp_config[mode].vol_ctl[index].control_base;

		ret_val = i2c_master_send(i2c, data, VOLUME_REG_SIZE + 1);

		if (ret_val != VOLUME_REG_SIZE + 1) {
			dev_dbg(codec->dev,"i2c_master_send transfer failed\n");
		} else {
			/* store the current level */
			kcontrol->private_value = user_value;
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */
			//data[0] = 0x1;  /* reg 1*/
			//data[1] = 0x05;/* Enable shifting buffer frm A to B */
			//i2c_master_send(i2c, data, 2);
		}
	/* Initiate buffer swap */
		value[0] = 1;

		if (i2c_master_send(i2c, value, 1) != 1) {
			dev_dbg(codec->dev,"Can not write register address\n");
		}
		/* Read the Value of the Page 8 Register 1 which controls the
		   Adaptive Switching Mode */
		if (i2c_master_recv(i2c, value, 1) != 1) {
			dev_dbg(codec->dev,"Can not read codec registers\n");
		}
		dev_dbg(codec->dev,"volume: adaptive=0x%x", value[0]);
		swap_reg_pre = value[0];
		/* Write the Register bit updates */
		value[1] = value[0] | 1 | 0x4;
		value[0] = 1;
		if (i2c_master_send(i2c, value, 2) != 2) {
			dev_dbg(codec->dev,"Can not write register address\n");
		}
		value[0] = 1;
		/* verify buffer swap */
		if (i2c_master_send(i2c, value, 1) != 1) {
			dev_dbg(codec->dev,"Can not write register address\n");
		}
		/* Read the Value of the Page 8 Register 1 which controls the
		   Adaptive Switching Mode */
		if (i2c_master_recv(i2c, &swap_reg_post, 1) != 1) {
			dev_dbg(codec->dev,"Can not read codec registers\n");
		}
		dev_dbg(codec->dev, "Volume...\nswap_reg_pre=%x, \
			swap_reg_post=%x\n", swap_reg_pre, swap_reg_post);
		if ((swap_reg_pre == 4 && swap_reg_post == 6)
				|| (swap_reg_pre == 6 && swap_reg_post == 4))
			dev_dbg(codec->dev, "Buffer swap success\n");
		else
			dev_dbg(codec->dev, "Buffer swap...FAILED\n \
					swap_reg_pre= %x, swap_reg_post=%x\n",
					swap_reg_pre, swap_reg_post);

		/* update the new buffer value in the old, just swapped out buffer */
		aic31xx_change_page(codec, dsp_config[mode].vol_ctl[index].control_page);
		ret_val=i2c_master_send(i2c, data, MUX_CTRL_REG_SIZE + 1);
		ret_val=0;
	}

	aic31xx_change_page(codec, 0);
	mutex_unlock(&aic31xx->mutex_page);
	return (ret_val);
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_volume_lite_mixer_controls
 *
 * Purpose  : Add amixer kcontrols for mini dsp volume Lite controls,
 *----------------------------------------------------------------------------
 */
static int minidsp_volume_mixer_controls(struct snd_soc_codec *codec,
		int size, control * cntl, string * name)
{
	int i, err, no_volume_controls;
	static char volume_control_name[MAX_VOLUME_CONTROLS][40];

	no_volume_controls = size;

	dev_dbg(codec->dev," %d mixer controls for mini dsp 'volumeLite' \n",
		no_volume_controls);

	if (no_volume_controls) {
		for (i = 0; i < no_volume_controls; i++) {
			strcpy(volume_control_name[i], name[i]);
			strcat(volume_control_name[i], VOLUME_KCONTROL_NAME);

			dev_dbg(codec->dev,"Volume controls: %s\n",
				volume_control_name[i]);

			snd_vol_controls[i].name = volume_control_name[i];
			snd_vol_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_vol_controls[i].access =
			    SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_vol_controls[i].info =
			    __new_control_info_minidsp_volume;
			snd_vol_controls[i].get =
			    __new_control_get_minidsp_volume;
			snd_vol_controls[i].put =
			    __new_control_put_minidsp_volume;
			/*
			 *      TBD: read volume reg and update the index number
			 */
			snd_vol_controls[i].private_value = 0;
			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card->snd_card,
					  snd_soc_cnew(&snd_vol_controls[i],
						       codec, NULL, NULL));
			if (err < 0) {
				dev_err(codec->dev, "%s:Invalid control %s\n",
				 __FILE__, snd_vol_controls[i].name);
			}
		}
	}
	return 0;
}
#endif /* USE_VOLUME_CONTROLS */

/************************** MUX CONTROL section *****************************/
#if defined(USE_MUX_CONTROLS)
static struct snd_kcontrol_new snd_mux_controls[MAX_MUX_CONTROLS];

#define SOC_SINGLE_AIC31XX_MIX(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,\
	.name = xname, \
	.info = mix_control_info, \
	.get = mix_control_get, \
	.put = mix_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}


static const struct snd_kcontrol_new aic3111_minidsp_controls[] = {
	SOC_SINGLE_AIC31XX_MIX("Minidsp mode"),
};



#define DEFINE_MUX_CONTROL_INFO( post_name, config_no)\
	static int __new_control_info_minidsp_mux_#post_name \
( struct snd_kcontrol *kcontrol,  struct snd_ctl_elem_info *uinfo)\
{\
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);\
	int index, mode;\
	int ret_val = -1;\
	mode = config_no ;\
	\
	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {\
		if (strstr(kcontrol->id.name,\
			  dsp_config[mode].mux_ctl_name[index] )) {\
			break;\
		}\
	}\
	\
	dev_dbg(codec->dev, "Mux_ctl_name=%s mode=%d index=%d size=%d",\
			dsp_config[mode].mux_ctl_name[index], mode, index,\
			dsp_config[mode].size_of_mux_ctl);\
	if (index < dsp_config[mode].size_of_mux_ctl) {\
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;\
		uinfo->count = 1;\
		uinfo->value.integer.min = MIN_MUX_CTRL;\
		uinfo->value.integer.max = MAX_MUX_CTRL;\
		ret_val = 0;\
	}\
	return ret_val;\
}



typedef  int (*__fp_new_control_info_minidsp) (struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo);

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_mux
 *
 * Purpose  : info routine for mini dsp mux control amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_mux(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index, mode;
	int ret_val = -1;
	mode = 0;

	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {
		if (strstr(kcontrol->id.name,
			 dsp_config[mode].mux_ctl_name[index])) {
			break;
		}
	}

	dev_dbg(codec->dev, "Mux_ctl_name=%s mode=%d index=%d size=%d",
			dsp_config[mode].mux_ctl_name[index], mode, index,
			dsp_config[mode].size_of_mux_ctl);

	if (index < dsp_config[mode].size_of_mux_ctl) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}
	return ret_val;
}
/*
DEFINE_MUX_CONTROL_INFO(second, 2)
DEFINE_MUX_CONTROL_INFO(third, 3)
DEFINE_MUX_CONTROL_INFO(fourth, 4)

__fp_new_control_info_minidsp fp_mux_control_info [MAX_DSP_CONFIG] = {
	__new_control_info_minidsp_mux,
	__new_control_info_minidsp_mux_second,
	__new_control_info_minidsp_mux_third,
	__new_control_info_minidsp_mux_fourth,
};
*/

static int __new_control_info_minidsp_mux_second(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index, mode;
	int ret_val = -1;
	mode = 1;

	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {
		if (strstr(kcontrol->id.name,
			  dsp_config[mode].mux_ctl_name[index] )) {
			break;
		}
	}

	dev_dbg (codec->dev, "Mux_ctl_name=%s mode=%d index=%d size=%d",
			dsp_config[mode].mux_ctl_name[index], mode, index,
			dsp_config[mode].size_of_mux_ctl);

	if (index < dsp_config[mode].size_of_mux_ctl) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}
	return ret_val;
}

static int __new_control_info_minidsp_mux_third(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index, mode;
	int ret_val = -1;
	mode = 2;

	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {
		if (strstr(kcontrol->id.name,
			  dsp_config[mode].mux_ctl_name[index] )) {
			break;
		}
	}

	dev_dbg(codec->dev, "Mux_ctl_name=%s mode=%d index=%d size=%d",
			dsp_config[mode].mux_ctl_name[index], mode, index,
			dsp_config[mode].size_of_mux_ctl);

	if (index < dsp_config[mode].size_of_mux_ctl) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}
	return ret_val;
}

/*
static int __new_control_info_minidsp_mux_fourth(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int index, mode;
	int ret_val = -1;
	mode = 3;

	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {
		if (strstr(kcontrol->id.name,
			  dsp_config[mode].mux_ctl_name[index] )) {
			break;
		}
	}

	dev_dbg(codec->dev, "Mux_ctl_name=%s mode=%d index=%d size=%d",
			dsp_config[mode].mux_ctl_name[index], mode, index,
			dsp_config[mode].size_of_mux_ctl);

	if (index < dsp_config[mode].size_of_mux_ctl) {
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}
	return ret_val;
}
*/

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_mux
 *
 * Purpose  : get routine for  mux control amixer kcontrols,
 * 			  read current register values to user.
 * 			  Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_mux(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_mux
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 *            values. Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_mux(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 data[MUX_CTRL_REG_SIZE + 1];
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int user_value;
	int index, mode;

	struct i2c_client *i2c;
	int ret_val = -1;
	u8 value[2], swap_reg_pre, swap_reg_post;
	i2c = codec->control_data;

	mode = aic31xx->current_process_flow;
	user_value = ucontrol->value.integer.value[0];
	dev_dbg(codec->dev,"user value = 0x%x\n", user_value);

	mutex_lock(&aic31xx->mutex_page);

	for (index = 0; index < dsp_config[mode].size_of_mux_ctl; index++) {
		if (strstr(kcontrol->id.name,
			 dsp_config[mode].mux_ctl_name[index]) )
			break;
	}

	if (index < dsp_config[mode].size_of_mux_ctl) {
		dev_dbg(codec->dev,"Index %d Changing to Page %d\n", index,
				dsp_config[mode].mux_ctl[index].control_page);
		aic31xx_change_page(codec,
			dsp_config[mode].mux_ctl[index].control_page);
		if (user_value == 1) {
			data[1] = 0x00;
			data[2] = 0x00;
			data[3] = 0x00;
		} else {
			data[1] = 0xFF;
			data[2] = 0xFF;
			data[3] = 0xFF;
		}
		data[1] = (u8) ((user_value >> 16) & AIC31XX_8BITS_MASK);
		data[2] = (u8) ((user_value >> 8) & AIC31XX_8BITS_MASK);
		data[3] = (u8) ((user_value) & AIC31XX_8BITS_MASK);

		/* start register address */
		data[0] = dsp_config[mode].mux_ctl[index].control_base;

		dev_dbg(codec->dev,"writing %d %d %d %d\r\n",
				 data[0], data[1], data[2], data[3]);
		ret_val = i2c_master_send(i2c, data, MUX_CTRL_REG_SIZE + 1);

		if (ret_val != MUX_CTRL_REG_SIZE + 1) {
			dev_err(codec->dev, "i2c_master_send transfer failed\n");
		} else {
			/* store the current level */
			kcontrol->private_value = user_value;
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */
			//data[0] = 0x1;  /* reg 1*/
			//data[1] = 0x05; /* Enable shifting buffer from A to B */
			//i2c_master_send(i2c, data, 2);
		}
		/* Perform a BUFFER SWAP Command. Check if we are currently not
		 * in Page 8, if so, swap to Page 8 first
		 */

		value[0] = 1;
		aic31xx_change_page(codec, 8);
		if (i2c_master_send(i2c, value, 1) != 1) {
			dev_err(codec->dev, "Can not write register address\n");
		}
		/* Read the Value of the Page 8 Register 1 which controls the
		   Adaptive Switching Mode */
		if (i2c_master_recv(i2c, value, 1) != 1) {
			dev_err(codec->dev, "Can not read codec registers\n");
		}
		swap_reg_pre = value[0];
		dev_dbg(codec->dev,"mux: adaptive=0x%x", value[0]);
		/* Write the Register bit updates */
		value[1] = value[0] | 1 | 0x4; //Adaptive mode enabled
		value[0] = 1;

		if (i2c_master_send(i2c, value, 2) != 2) {
			dev_err(codec->dev, "Can not write register address\n");
		}
		value[0] = 1;
		/* verify buffer swap */
		if (i2c_master_send(i2c, value, 1) != 1) {
			dev_err(codec->dev, "Can not write register address\n");
		}
		/* Read the Value of the Page 8 Register 1 which controls the
		   Adaptive Switching Mode */
		if (i2c_master_recv(i2c, &swap_reg_post, 1) != 1) {
			dev_err(codec->dev, "Can not read codec registers\n");
		}
		dev_dbg(codec->dev, "swap_reg_pre=%x, swap_reg_post=%x\n",
				swap_reg_pre, swap_reg_post);
		if ((swap_reg_pre == 4 && swap_reg_post == 6)
				|| (swap_reg_pre == 6 && swap_reg_post == 4))
			dev_dbg(codec->dev, "Buffer swap success\n");
		else
			dev_err(codec->dev, "Buffer swap...FAILED\n\
					swap_reg_pre=%x, swap_reg_post=%x\n",
					swap_reg_pre, swap_reg_post);

		/* update the new buffer value in the old, just swapped out buffer */
		aic31xx_change_page(codec,
			dsp_config[mode].mux_ctl[index].control_page);
		ret_val = i2c_master_send(i2c, data, MUX_CTRL_REG_SIZE + 1);
		ret_val = 0;

	}

	aic31xx_change_page(codec, 0);
	mutex_unlock(&aic31xx->mutex_page);
	return (ret_val);
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_mux_ctrl_mixer_controls
 *
 * Purpose  : Add amixer kcontrols for mini dsp mux controls,
 *----------------------------------------------------------------------------
 */
static int minidsp_mux_ctrl_mixer_controls(struct snd_soc_codec *codec,
			int mode, int size, control * cntl, string * name)
{
	int i, err, no_mux_controls;
#if 0
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
#endif
	no_mux_controls = size;

	dev_dbg(codec->dev," %d mixer controls for mini dsp MUX \n",
			 no_mux_controls);

	if (no_mux_controls) {
		for (i = 0; i < no_mux_controls; i++) {

			//snd_mux_controls[i].name = MUX_control_names[i];
			snd_mux_controls[i].name = name[i];
			snd_mux_controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			snd_mux_controls[i].access =
			    SNDRV_CTL_ELEM_ACCESS_READWRITE;
			snd_mux_controls[i].get = __new_control_get_minidsp_mux;
			snd_mux_controls[i].put = __new_control_put_minidsp_mux;
#if 1
			if(0==mode){
				snd_mux_controls[i].info = __new_control_info_minidsp_mux;
			}else if( 1==mode){
				snd_mux_controls[i].info = __new_control_info_minidsp_mux_second;
			}else if( 2==mode) {
				snd_mux_controls[i].info = __new_control_info_minidsp_mux_third;
			}/*  else if( 3==mode) {
				snd_mux_controls[i].info = __new_control_info_minidsp_mux_fourth;
			} */
			dev_dbg(codec->dev, "%s: control %s\n", name[i],
				       snd_mux_controls[i].name);
#else
			dev_dbg(codec->dev, "%s: control %s\n", name[i],
				       snd_mux_controls[i].name);
			snd_mux_controls[i].info = *fp_mux_control_info[mode];
#endif
	//		snd_mux_controls[i].info =	__new_control_info_minidsp_mux;
			/*
			 *  TBD: read volume reg and update the index number
			 */
#if 0
			mutex_lock(&aic31xx->mutex_page);
			aic31xx_change_page(codec, cntl[i].control_page);
			value = i2c_smbus_read_byte_data(codec->control_data, cntl[i].control_base);
			mutex_unlock(&aic31xx->mutex_page);
			dev_info(codec->dev, " Control data %x \n", value);
			if (value >= 0 && value != 255) {
				snd_mux_controls[i].private_value = value;
			} else {
				snd_mux_controls[i].private_value = 0;
			}
#endif
//			snd_vol_controls[i].private_value = 0;
//			snd_vol_controls[i].count = 0;

			err = snd_ctl_add(codec->card->snd_card,
					  snd_soc_cnew(&snd_mux_controls[i],
						       codec, NULL, NULL));
			if (err < 0) {
				dev_err(codec->dev,"%s:Invalid control %s\n", __FILE__,
				       snd_mux_controls[i].name);
			}
		}
	}
	return 0;
}
#endif /* USE_MUX_CONTROLS */

/************************** Adaptive filtering section **********************/

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_filter
 *
 * Purpose  : info routine for adaptive filter control amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_adaptive(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_adaptive
 *
 * Purpose  : get routine for  adaptive filter control amixer kcontrols,
 *            reads to user if adaptive filtering is enabled or disabled.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_adaptive(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC31XX_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC31XX_8BITS_MASK;
	u8 rmask = (kcontrol->private_value >> 16) & AIC31XX_8BITS_MASK;

	i2c = codec->control_data;

	dev_dbg(codec->dev,"page %d, reg %d, mask 0x%x\n", page, reg, rmask);

	mutex_lock(&aic31xx->mutex_page);

	/* Read the register value */
	aic31xx_change_page(codec, page);

	/* write register addr to read */
	data[0] = reg;

	if (i2c_master_send(i2c, data, 1) != 1) {
		dev_err(codec->dev, "Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv(i2c, data, 1) != 1) {
		dev_err(codec->dev, "Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dev_dbg(codec->dev,"read: 0x%x\n", data[0]);

	/* return the read status to the user */
	if (data[0] & rmask) {
		ucontrol->value.integer.value[0] = 1;
	} else {
		ucontrol->value.integer.value[0] = 0;
	}

      revert:
	/* put page back to zero */
	aic31xx_change_page(codec, 0);
	mutex_unlock(&aic31xx->mutex_page);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_minidsp_adaptive
 *
 * Purpose  : put routine for adaptive filter controls amixer kcontrols.
 * 			  This routine will enable/disable adaptive filtering.
 *----------------------------------------------------------------------------
 */
static int __new_control_put_minidsp_adaptive(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int user_value = ucontrol->value.integer.value[0];
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC31XX_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC31XX_8BITS_MASK;
	u8 wmask = (kcontrol->private_value >> 24) & AIC31XX_8BITS_MASK;
	/*u8 rmask = (kcontrol->private_value >> 16) & AIC31XX_8BITS_MASK;*/

	i2c = codec->control_data;

	dev_dbg(codec->dev,"page %d, reg %d, mask 0x%x, user_value %d\n",
		page, reg, wmask, user_value);

	mutex_lock(&aic31xx->mutex_page);

	/* Program the register value */
	aic31xx_change_page(codec, page);

	/* read register addr to read */
	data[0] = reg;

	if (i2c_master_send(i2c, data, 1) != 1) {
		dev_err(codec->dev,"Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv(i2c, data, 1) != 1) {
		dev_err(codec->dev,"Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dev_dbg(codec->dev,"read: 0x%x\n", data[0]);

	/* set the bitmask and update the register */
	if (user_value == 0) {
		data[1] = (data[0]) & (~wmask);
	} else {
		data[1] = (data[0]) | wmask;
	}
	data[0] = reg;

	if (i2c_master_send(i2c, data, 2) != 2) {
		dev_err(codec->dev,"Can not write register address\n");
		ret = -1;
	}

      revert:
	/* put page back to zero */
	aic31xx_change_page(codec, 0);
	mutex_unlock(&aic31xx->mutex_page);
	return ret;
}

#define SOC_ADAPTIVE_CTL_AIC3111(xname, page, reg, read_mask, write_mask) \
{   .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
    .info = __new_control_info_minidsp_adaptive, \
    .get = __new_control_get_minidsp_adaptive, 	\
	.put = __new_control_put_minidsp_adaptive, \
	.count = 0,	\
    .private_value = (page) | (reg << 8) | 	\
		( read_mask << 16) | (write_mask << 24) \
}

/* Adaptive filtering control and buffer swap  mixer kcontrols */
static struct snd_kcontrol_new snd_adaptive_controls[] = {
	SOC_ADAPTIVE_CTL_AIC3111(FILT_CTL_NAME_ADC, BUFFER_PAGE_ADC, 0x1, 0x4,
				 0x4),
	SOC_ADAPTIVE_CTL_AIC3111(FILT_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x4,
				 0x4),
	SOC_ADAPTIVE_CTL_AIC3111(COEFF_CTL_NAME_ADC, BUFFER_PAGE_ADC, 0x1, 0x2,
				 0x1),
	SOC_ADAPTIVE_CTL_AIC3111(COEFF_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x2,
				 0x1),
};

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_adaptive_filter_mixer_controls
 *
 * Purpose  : registers adaptive filter mixer kcontrols
 *----------------------------------------------------------------------------
 */
static int minidsp_adaptive_filter_mixer_controls(struct snd_soc_codec *codec)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(snd_adaptive_controls); i++) {
		err = snd_ctl_add(codec->card->snd_card,
				  snd_soc_cnew(&snd_adaptive_controls[i], codec,
					       NULL, NULL));

		if (err < 0) {
			dev_err(codec->dev,"%s:Invalid control %s\n", __FILE__,
			       snd_adaptive_controls[i].name);
			return err;
		}
	}
	return 0;
}

#endif /* ADD_MINIDSP_CONTROLS */
/*
 *----------------------------------------------------------------------------
 * Function : mix_control_info
 * Purpose  : This function is to initialize data for new control required to
 *            program the AIC3111 registers.
 *
 *----------------------------------------------------------------------------
 */
static int mix_control_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->count=1;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = MAX_DSP_CONFIG-1;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : mix_control_get
 * Purpose  : This function is to read data of new control for
 *            program the AIC3111 registers.
 *
 *----------------------------------------------------------------------------
 */
static int mix_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip (kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u32 val;

	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		val = aic31xx->current_process_flow;
		ucontrol->value.integer.value[0] = val;
		dev_dbg(codec->dev, "control get : mode=%d\n",
				aic31xx->current_process_flow);
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : mix_new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *            the driver.
 *
 *----------------------------------------------------------------------------
 */
static int mix_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	u32 val;
	int mode = aic31xx->current_process_flow;

	dev_dbg(codec->dev, "mix_control_put\n");
	val = ucontrol->value.integer.value[0];
	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		dev_dbg(codec->dev, "\nMini dsp put\n mode = %d, val=%d\n",
				aic31xx->current_process_flow, val);
		if( val >= 0 && val < MAX_DSP_CONFIG ) {
			if (val != mode){
				if (aic31xx->mute == 1 ){
					set_minidsp_mode(codec, val);
					aic31xx->current_process_flow = val;
				} else {
					dev_warn(codec->dev,
							" Mode Switching: Playback in progress");
#if defined(CONFIG_MINIDSP)
					aic31xx_mute_codec(codec, 1);
					set_minidsp_mode(codec, val);
					aic31xx->current_process_flow = val;
					aic31xx_mute_codec(codec, 0);
#endif
				}
			}
		} else {
			dev_warn(codec->dev, "MiniDSP Mode %d invalide", val);
		}
	}
	dev_dbg(codec->dev, "\nmode = %d\n", mode);
	return mode;
}


extern void aic3111_add_minidsp_controls(struct snd_soc_codec *codec)
{
#ifdef ADD_MINI_DSP_CONTROLS

	int i, err, no_controls, no_dsp_config ;
	/* add mode k control */
	for(i = 0; i < ARRAY_SIZE(aic3111_minidsp_controls); i++) {
		err = snd_ctl_add(codec->card->snd_card,
		 snd_soc_cnew(&aic3111_minidsp_controls[i], codec, NULL, NULL));
		if (err < 0) {
			dev_err(codec->dev,"Invalid control\n");
			return ;
		}
	}
	/* add mux controls */

	no_dsp_config = MAX_DSP_CONFIG;
	for(i=0; i<no_dsp_config; i++) {
#if defined(USE_MUX_CONTROLS)
		no_controls = dsp_config[i].size_of_mux_ctl;
		minidsp_mux_ctrl_mixer_controls(codec, i, no_controls,
			 dsp_config[i].mux_ctl, dsp_config[i].mux_ctl_name);
#endif /* USE_MUX_CONTROLS */

#if defined(USE_VOLUME_CONTROLS)
		no_controls = dsp_config[i].size_of_vol_ctl;
		minidsp_volume_mixer_controls(codec, no_controls,
			 dsp_config[i].vol_ctl, dsp_config[i].vol_ctl_name);
#endif /* USE_VOLUME_CONTROLS */
	}

	if (minidsp_adaptive_filter_mixer_controls(codec)) {
		dev_dbg(codec->dev,
		    "Adaptive filter mixer control registration failed\n");
	}
#endif /* ADD_MINIDSP_CONTROLS */
}

/************** Dynamic MINI DSP programmer, TI LOAD support  ***************/

static struct cdev *minidsp_cdev;
static int minidsp_major = 0;	/* Dynamic allocation of Mjr No. */
static int minidsp_opened = 0;	/* Dynamic allocation of Mjr No. */
static struct snd_soc_codec *minidsp_codec;

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_open
 *
 * Purpose  : open method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static int minidsp_open(struct inode *in, struct file *filp)
{
	struct snd_soc_codec *codec = minidsp_codec;
	if (minidsp_opened) {
		dev_dbg(codec->dev,"%s device is already opened\n", "minidsp");
		dev_dbg(codec->dev,
				"%s: only one instance of driver is allowed\n",
				"minidsp");
		return -1;
	}
	minidsp_opened++;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_release
 *
 * Purpose  : close method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static int minidsp_release(struct inode *in, struct file *filp)
{
	minidsp_opened--;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_read
 *
 * Purpose  : read method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t minidsp_read(struct file *file, char __user * buf,
			    size_t count, loff_t * offset)
{
	static char rd_data[256];
	char reg_addr;
	size_t size;
	struct i2c_client *i2c = minidsp_codec->control_data;
	struct snd_soc_codec *codec = minidsp_codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	if (count > 128) {
		dev_warn(codec->dev,"Max 256 bytes can be read\n");
		count = 128;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (size != 0) {
		dev_err(codec->dev,"read: copy_from_user failure\n");
		return -1;
	}

	mutex_lock(&aic31xx->mutex_page);

	if (i2c_master_send(i2c, &reg_addr, 1) != 1) {
		dev_err(codec->dev,"Can not write register address\n");
		ret = -1;
		goto end;
	}
	/* read the codec/minidsp registers */
	size = i2c_master_recv(i2c, rd_data, count);

	if (size != count) {
		dev_warn(codec->dev,"read %d registers from the codec\n", size);
	}

	if (copy_to_user(buf, rd_data, size) != 0) {
		dev_err(codec->dev,"copy_to_user failed\n");
		ret = -1;
		goto end;
	}

end:
	mutex_unlock(&aic31xx->mutex_page);

	return size;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_write
 *
 * Purpose  : write method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t minidsp_write(struct file *file, const char __user * buf,
			     size_t count, loff_t * offset)
{
	static char wr_data[258];
	size_t size;
	struct i2c_client *i2c = minidsp_codec->control_data;
	struct snd_soc_codec *codec = minidsp_codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&aic31xx->mutex_page);

	/* copy buffer from user space  */
	size = copy_from_user(wr_data, buf, count);
	if (size != 0) {
		dev_err(codec->dev,"copy_from_user failure %d\n", size);
		ret = -1;
		goto end;
	}

	if (wr_data[0] == 0) {
		aic31xx_change_page(minidsp_codec, wr_data[1]);
	}

	size = i2c_master_send(i2c, wr_data, count);

	ret = size;
end:
	mutex_unlock(&aic31xx->mutex_page);

	return ret;
}

long minidsp_ioctl( struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	long ret=0;

	if (_IOC_TYPE(cmd) != AIC3111_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
		case AIC3111_IOMAGICNUM_GET:
			ret = copy_to_user((void __user *)arg,
					(const void*) &magic_num,
					(long) sizeof(int));
			break;
		case AIC3111_IOMAGICNUM_SET:
			ret = copy_from_user((void*)&magic_num,
					(const void __user *)arg,
					(long) sizeof(int));
			break;
	}
	return ret;
}

/*********** File operations structure for minidsp programming *************/
static struct file_operations minidsp_fops = {
	.owner = THIS_MODULE,
	.open = minidsp_open,
	.release = minidsp_release,
	.read = minidsp_read,
	.write = minidsp_write,
	.unlocked_ioctl = minidsp_ioctl,
};

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_driver_init
 *
 * Purpose  : Registeer a char driver for dynamic mini dsp programming
 *----------------------------------------------------------------------------
 */
static int minidsp_driver_init(struct snd_soc_codec *codec)
{
	int result;
	dev_t dev = MKDEV(minidsp_major, 0);

	minidsp_codec = codec;

	result = alloc_chrdev_region(&dev, 0, 1, "minidsp");

	if (result < 0) {
		dev_err(codec->dev, "cannot allocate major number %d\n",
				minidsp_major);
		return result;
	}

	minidsp_major = MAJOR(dev);

	minidsp_cdev = cdev_alloc();
	cdev_init(minidsp_cdev, &minidsp_fops);
	minidsp_cdev->owner = THIS_MODULE;
	minidsp_cdev->ops = &minidsp_fops;

	if (cdev_add(minidsp_cdev, dev, 1) < 0) {
		dev_err(codec->dev,"minidsp_driver: cdev_add failed \n");
		unregister_chrdev_region(dev, 1);
		minidsp_cdev = NULL;
		return 1;
	}
	dev_info(codec->dev, "Registered minidsp driver, Major number: %d \n",
			minidsp_major);
	return 0;
}

