/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
//#include <linux/i2c/bq2415x.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
//#include <linux/regulator/tps6130x.h>
//#include <linux/leds.h>
//#include <linux/leds_pwm.h>

//for power button led
#include <linux/leds-omap4430sdp-display.h>

#include <linux/omapfb.h>
#include <linux/reboot.h>
//#include <linux/twl6040-vib.h>
#include <linux/wl12xx.h>
#include <linux/memblock.h>
//#include <linux/mfd/twl6040-codec.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
//#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
//#include <video/omap-panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>

#include <mach/bowser_idme_init.h>

#include <linux/wakelock.h>
#include "board-blaze.h"
#include "omap4_ion.h"
#include "omap_ram_console.h"
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
/* for TI WiLink devices */
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <plat/omap-serial.h>
#include <linux/omap4_duty_cycle_governor.h>

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

//#define ETH_KS8851_IRQ			34
//#define ETH_KS8851_POWER_ON		48
//#define ETH_KS8851_QUART		138
#define OMAP4_TOUCH_IRQ_1		35
#define OMAP4_TOUCH_RESET_GPIO			18
#define OMAP4_CHARGER_IRQ       7
//#define OMAP4_TOUCH_IRQ_2		36
//#define HDMI_GPIO_CT_CP_HPD		60
#define HDMI_GPIO_HPD			63  /* Hot plug pin for HDMI */
//#define HDMI_GPIO_LS_OE 41 /* Level shifter for HDMI */
//#define LCD_BL_GPIO		27	/* LCD Backlight GPIO */
/* PWM2 and TOGGLE3 register offsets */
#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define TWL6030_TOGGLE3		0x92

#define TPS62361_GPIO   7

#define GPIO_WIFI_PMENA		54
#define GPIO_WIFI_IRQ		53

#define GPIO_3V_ENABLE          47
#define GPIO_OMAP_RGB_SHTDN     37
//#define OMAP_HDMI_HPD_ADDR	0x4A100098
//#define OMAP_HDMI_PULLTYPE_MASK	0x00000010

#define MBID0_GPIO 174
#define MBID1_GPIO 173
#define MBID2_GPIO 178
#define MBID3_GPIO 177
#define PANELID0_GPIO 176
#define PANELID1_GPIO 175
#define TOUCHID0_GPIO 50
#define TOUCHID1_GPIO 51

#if 0
static const int sdp4430_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(0, 1, KEY_R),
	KEY(0, 2, KEY_T),
	KEY(0, 3, KEY_HOME),
	KEY(0, 4, KEY_F5),
	KEY(0, 5, KEY_UNKNOWN),
	KEY(0, 6, KEY_I),
	KEY(0, 7, KEY_LEFTSHIFT),

	KEY(1, 0, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(1, 2, KEY_G),
	KEY(1, 3, KEY_SEND),
	KEY(1, 4, KEY_F6),
	KEY(1, 5, KEY_UNKNOWN),
	KEY(1, 6, KEY_K),
	KEY(1, 7, KEY_ENTER),

	KEY(2, 0, KEY_X),
	KEY(2, 1, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(2, 3, KEY_END),
	KEY(2, 4, KEY_F7),
	KEY(2, 5, KEY_UNKNOWN),
	KEY(2, 6, KEY_DOT),
	KEY(2, 7, KEY_CAPSLOCK),

	KEY(3, 0, KEY_Z),
	KEY(3, 1, KEY_KPPLUS),
	KEY(3, 2, KEY_B),
	KEY(3, 3, KEY_F1),
	KEY(3, 4, KEY_F8),
	KEY(3, 5, KEY_UNKNOWN),
	KEY(3, 6, KEY_O),
	KEY(3, 7, KEY_SPACE),

	KEY(4, 0, KEY_W),
	KEY(4, 1, KEY_Y),
	KEY(4, 2, KEY_U),
	KEY(4, 3, KEY_F2),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(4, 5, KEY_UNKNOWN),
	KEY(4, 6, KEY_L),
	KEY(4, 7, KEY_LEFT),

	KEY(5, 0, KEY_S),
	KEY(5, 1, KEY_H),
	KEY(5, 2, KEY_J),
	KEY(5, 3, KEY_F3),
	KEY(5, 4, KEY_F9),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_M),
	KEY(5, 7, KEY_RIGHT),

	KEY(6, 0, KEY_Q),
	KEY(6, 1, KEY_A),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_BACK),
	KEY(6, 4, KEY_BACKSPACE),
	KEY(6, 5, KEY_UNKNOWN),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_UP),

	KEY(7, 0, KEY_PROG1),
	KEY(7, 1, KEY_PROG2),
	KEY(7, 2, KEY_PROG3),
	KEY(7, 3, KEY_PROG4),
	KEY(7, 4, KEY_F4),
	KEY(7, 5, KEY_UNKNOWN),
	KEY(7, 6, KEY_OK),
	KEY(7, 7, KEY_DOWN),
};

static struct matrix_keymap_data sdp4430_keymap_data = {
	.keymap			= sdp4430_keymap,
	.keymap_size		= ARRAY_SIZE(sdp4430_keymap),
};

void keypad_pad_wkup(int enable)
{
	int (*set_wkup_fcn)(const char *muxname);

	/* PAD wakup for keyboard is needed for off mode
	 * due to IO isolation.
	 */
	if (!off_mode_enabled)
		return;

	if (enable)
		set_wkup_fcn = omap_mux_enable_wkup;
	else
		set_wkup_fcn = omap_mux_disable_wkup;

	set_wkup_fcn("kpd_col0.kpd_col0");
	set_wkup_fcn("kpd_col1.kpd_col1");
	set_wkup_fcn("kpd_col2.kpd_col2");
	set_wkup_fcn("kpd_col0.kpd_col0");
	set_wkup_fcn("kpd_col1.kpd_col1");
	set_wkup_fcn("kpd_col2.kpd_col2");
	set_wkup_fcn("kpd_col3.kpd_col3");
	set_wkup_fcn("kpd_col4.kpd_col4");
	set_wkup_fcn("kpd_col5.kpd_col5");
	set_wkup_fcn("gpmc_a23.kpd_col7");
	set_wkup_fcn("gpmc_a22.kpd_col6");
	set_wkup_fcn("kpd_row0.kpd_row0");
	set_wkup_fcn("kpd_row1.kpd_row1");
	set_wkup_fcn("kpd_row2.kpd_row2");
	set_wkup_fcn("kpd_row3.kpd_row3");
	set_wkup_fcn("kpd_row4.kpd_row4");
	set_wkup_fcn("kpd_row5.kpd_row5");
	set_wkup_fcn("gpmc_a18.kpd_row6");
	set_wkup_fcn("gpmc_a19.kpd_row7");

}

#ifdef CONFIG_OMAP4_DUTY_CYCLE

static struct pcb_section omap4_duty_governor_pcb_sections[] = {
	{
		.pcb_temp_level			= 65,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= false,
		.tduty_params = {
			.nitro_rate		= 0,
			.cooling_rate		= 0,
			.nitro_interval		= 0,
			.nitro_percentage	= 0,
		},
	},
	{
		.pcb_temp_level			= 70,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 37,
		},
	},
	{
		.pcb_temp_level			= 75,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 24,
		},
	},
	{
		.pcb_temp_level			= 80,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 19,
		},
	},
	{
		.pcb_temp_level			= 90,
		.max_opp			= 1200000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1200000,
			.cooling_rate		= 1008000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 14,
		},
	},
	{
		.pcb_temp_level			= 110,
		.max_opp			= 1008000,
		.duty_cycle_enabled		= true,
		.tduty_params = {
			.nitro_rate		= 1008000,
			.cooling_rate		= 800000,
			.nitro_interval		= 20000,
			.nitro_percentage	= 1,
		},
	},
};

void init_duty_governor(void)
{
	omap4_duty_pcb_section_reg(omap4_duty_governor_pcb_sections,
		ARRAY_SIZE(omap4_duty_governor_pcb_sections));
}
#else
void init_duty_governor(void){}
#endif /*CONFIG_OMAP4_DUTY_CYCLE*/


static struct omap4_keypad_platform_data sdp4430_keypad_data = {
	.keymap_data		= &sdp4430_keymap_data,
	.rows			= 8,
	.cols			= 8,
	.keypad_pad_wkup        = keypad_pad_wkup,
};
static struct gpio_led sdp4430_gpio_leds[] = {
	{
		.name	= "omap4:green:debug0",
		.gpio	= 61,
	},
	{
		.name	= "omap4:green:debug1",
		.gpio	= 30,
	},
	{
		.name	= "omap4:green:debug2",
		.gpio	= 7,
	},
	{
		.name	= "omap4:green:debug3",
		.gpio	= 8,
	},
	{
		.name	= "omap4:green:debug4",
		.gpio	= 50,
	},
	{
		.name	= "omap4:blue:user",
		.gpio	= 169,
	},
	{
		.name	= "omap4:red:user",
		.gpio	= 170,
	},
	{
		.name	= "omap4:green:user",
		.gpio	= 139,
	},

};

static struct gpio_led_platform_data sdp4430_led_data = {
	.leds	= sdp4430_gpio_leds,
	.num_leds	= ARRAY_SIZE(sdp4430_gpio_leds),
};

static struct led_pwm sdp4430_pwm_leds[] = {
	{
		.name		= "omap4:green:chrg",
		.pwm_id		= 1,
		.max_brightness	= 255,
		.pwm_period_ns	= 7812500,
	},
};

static struct led_pwm_platform_data sdp4430_pwm_data = {
	.num_leds	= ARRAY_SIZE(sdp4430_pwm_leds),
	.leds		= sdp4430_pwm_leds,
};

static struct platform_device sdp4430_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_pwm_data,
	},
};

static struct platform_device sdp4430_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &sdp4430_led_data,
	},
};
#endif

/* Board IDs */
static u8 quanta_mbid;
static u8 quanta_touchid;
static u8 quanta_panelid;
u8 quanta_get_mbid(void)
{
	return quanta_mbid;
}
EXPORT_SYMBOL(quanta_get_mbid);

u8 quanta_get_touchid(void)
{
	return quanta_touchid;
}
EXPORT_SYMBOL(quanta_get_touchid);

u8 quanta_get_panelid(void)
{
	return quanta_panelid;
}
EXPORT_SYMBOL(quanta_get_panelid);

static void __init quanta_boardids(void)
{
    gpio_request(MBID0_GPIO, "MBID0");
    gpio_direction_input(MBID0_GPIO);
    gpio_request(MBID1_GPIO, "MBID1");
    gpio_direction_input(MBID1_GPIO);
    gpio_request(MBID2_GPIO, "MBID2");
    gpio_direction_input(MBID2_GPIO);
    gpio_request(MBID3_GPIO, "MBID3");
    gpio_direction_input(MBID3_GPIO);
    gpio_request(PANELID0_GPIO, "PANELID0");
    gpio_direction_input(PANELID0_GPIO);
    gpio_request(PANELID1_GPIO, "PANELID1");
    gpio_direction_input(PANELID1_GPIO);
    gpio_request(TOUCHID0_GPIO, "TOUCHID0");
    gpio_direction_input(TOUCHID0_GPIO);
    gpio_request(TOUCHID1_GPIO, "TOUCHID1");
    gpio_direction_input(TOUCHID1_GPIO);
    quanta_mbid=gpio_get_value(MBID0_GPIO) | ( gpio_get_value(MBID1_GPIO)<<1)
        | ( gpio_get_value(MBID2_GPIO)<<2) | ( gpio_get_value(MBID3_GPIO)<<3);
    quanta_touchid = gpio_get_value(TOUCHID0_GPIO) | ( gpio_get_value(TOUCHID1_GPIO)<<1);
    quanta_panelid = gpio_get_value(PANELID0_GPIO) | ( gpio_get_value(PANELID1_GPIO)<<1);
}

static struct spi_board_info sdp4430_spi_board_info[] __initdata = {
#if 0
	{
		.modalias               = "ks8851",
		.bus_num                = 1,
		.chip_select            = 0,
		.max_speed_hz           = 24000000,
		.irq                    = ETH_KS8851_IRQ,
	},
#endif
	{
		.modalias		= "otter1_disp_spi",
		.bus_num		= 4,     /* McSPI4 */
		.chip_select		= 0,
		.max_speed_hz		= 375000,
	},
};

#if 0
static struct gpio sdp4430_eth_gpios[] __initdata = {
	{ ETH_KS8851_POWER_ON,	GPIOF_OUT_INIT_HIGH,	"eth_power"	},
	{ ETH_KS8851_QUART,	GPIOF_OUT_INIT_HIGH,	"quart"		},
	{ ETH_KS8851_IRQ,	GPIOF_IN,		"eth_irq"	},
};

static int __init omap_ethernet_init(void)
{
	int status;

	/* Request of GPIO lines */
	status = gpio_request_array(sdp4430_eth_gpios,
				    ARRAY_SIZE(sdp4430_eth_gpios));
	if (status)
		pr_err("Cannot request ETH GPIOs\n");

	return status;
}
#endif
/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is capable enough to wake-up the
 * OMAP host.
 */
static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
		state)
{
	return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);
	return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 55,
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_wlink_kim_suspend,
	.resume = plat_wlink_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake  = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};
static struct platform_device sdp4430_aic3110 = {
    .name           = "tlv320aic3110-codec",
    .id             = -1,
};    
static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

#if 0
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
#endif

static struct twl4030_madc_platform_data twl6030_gpadc = {
	.irq_line = -1,
};
//For power button leds
static void sdp4430_init_display_led(void)
{
	twl_i2c_write_u8(TWL_MODULE_PWM, 0xFF, LED_PWM2ON);
	twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	//twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x38, TWL6030_TOGGLE3);
}

static void sdp4430_set_primary_brightness(u8 brightness)
{
	if (brightness > 1) {
		if (brightness == 255)
			brightness = 0x7f;
		else
			brightness = (~(brightness/2)) & 0x7f;

		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL_MODULE_PWM, brightness, LED_PWM2ON);
	} else if (brightness <= 1) {
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x38, TWL6030_TOGGLE3);
	}
}
static struct omap4430_sdp_disp_led_platform_data sdp4430_disp_led_data = {
	.flags = LEDS_CTRL_AS_ONE_DISPLAY,
	.display_led_init = sdp4430_init_display_led,
	.primary_display_set = sdp4430_set_primary_brightness,
	//.secondary_display_set = sdp4430_set_secondary_brightness,
};
static struct platform_device sdp4430_disp_led = {
	.name	=	"display_led",
	.id	=	-1,
	.dev	= {
		.platform_data = &sdp4430_disp_led_data,
	},
};

static struct platform_device sdp4430_keypad_led = {
	.name	=	"keypad_led",
	.id	=	-1,
	.dev	= {
		.platform_data = NULL,
	},
};

static struct omap_pwm_led_platform_data kc1_led_data = {
	.name = "lcd-backlight",
	.intensity_timer = 10,
	.def_brightness = 0x7F,
};

static struct platform_device kc1_led_device = {
	.name       = "omap_pwm_led",
	.id     = -1,
	.dev        = {
		.platform_data = &kc1_led_data,
	},
};

static struct platform_device *sdp4430_devices[] __initdata = {
	//&sdp4430_leds_gpio,
	//&sdp4430_leds_pwm,
	&wl128x_device,
	//&btwilink_device,
	&sdp4430_aic3110,
	&kc1_led_device,
    &sdp4430_disp_led,//For power button
    &sdp4430_keypad_led,//For power button
};

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
};

static void __init omap_4430sdp_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#else
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 200,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.no_off_init	= true,
	},
#if 0
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_wp	= -EINVAL,
	},
#endif
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp4430_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};
static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
static struct regulator_consumer_supply sdp4430_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};
static struct regulator_consumer_supply omap4_sdp4430_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};
static struct regulator_consumer_supply sdp4430_audio_supply[] = {
    {
        .supply = "audio-pwr",
    },

};
static struct regulator_init_data sdp4430_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_sdp4430_vmmc5_supply,
};
static struct fixed_voltage_config sdp4430_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &sdp4430_vmmc5,
};
static struct platform_device omap_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &sdp4430_vwlan,
               }
};
static struct regulator_consumer_supply sdp4430_gsensor_supply[] = {
	{
		.supply = "g-sensor-pwr",
	},
};
static struct regulator_consumer_supply sdp4430_vaux3_supply[] = {
	{
		.supply = "vaux3",
	},
};
static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	if (pdev->id == 4) {
		pdata->slots[0].mmc_data.built_in = 1;
	}

	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.always_on  		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vaux_supply,
};

#if 0
static struct regulator_consumer_supply sdp4430_vaux2_supply[] = {
	REGULATOR_SUPPLY("av-switch", "soc-audio"),
};
#endif

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.always_on      = true,
        .valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
    .num_consumer_supplies = 1,
    .consumer_supplies = sdp4430_gsensor_supply,
};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_vaux3_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.disabled       = true,
		},
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                    
	},
    .num_consumer_supplies  = 1,
    .consumer_supplies      = sdp4430_audio_supply,
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
        .state_mem = {
            .enabled    = false,
            .disabled   = true,
        },
	},
	.num_consumer_supplies	= ARRAY_SIZE(sdp4430_vcxio_supply),
	.consumer_supplies	= sdp4430_vcxio_supply,
};

static struct regulator_consumer_supply sdp4430_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
			.enabled    = false,
			.disabled   = true,
	        },
	},
	.num_consumer_supplies  = ARRAY_SIZE(sdp4430_vdac_supply),
	.consumer_supplies      = sdp4430_vdac_supply,
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data sdp4430_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

#if 0
static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

static int tps6130x_enable(int on)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
	if (ret < 0) {
		pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
		return ret;
	}

	/* TWL6040 GPO2 connected to TPS6130X NRESET */
	if (on)
		val |= TWL6040_GPO2;
	else
		val &= ~TWL6040_GPO2;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

	return ret;
}

static struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable	= tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
	.constraints = {
		.min_uV			= 4075000,
		.max_uV			= 4950000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies	= twl6040_vddhf_supply,
	.driver_data		= &tps6130x_pdata,
};

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
				&rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
	 */
	if (rev == TWL6040_REV_1_1)
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
			OMAP_PIN_INPUT);

	return 0;
}

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
	.vddhf_uV	= 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
	.voltage_raise_speed = 0x26,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
	.init		= twl6040_init,
};

static int sdp4430_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data sdp4430_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= sdp4430_batt_table,
	.tblsize			= ARRAY_SIZE(sdp4430_batt_table),
};
#endif

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,
	.clk32kg	= &sdp4430_clk32kg,
	.usb		= &omap4_usbphy_data,
//	.bci		= &sdp4430_bci_data,
	/* children */
//	.codec		= &twl6040_codec,
	.madc		= &twl6030_gpadc,

};

#if 0
static struct bq2415x_platform_data sdp4430_bqdata = {
	.max_charger_voltagemV = 4200,
	.max_charger_currentmA = 1550,
};
#endif

static struct i2c_board_info __initdata sdp4430_i2c_boardinfo[] = {
#ifdef CONFIG_BATTERY_BQ27541_Q
    {
        I2C_BOARD_INFO("bq27541", 0x55),
    },
#endif
};

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
        {       // chris 2011_0124
                I2C_BOARD_INFO("ilitek_i2c", 0x41),
                .irq = OMAP_GPIO_IRQ(OMAP4_TOUCH_IRQ_1),
        },
};

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {

};

static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {
#ifdef CONFIG_SUMMIT_SMB347_Q
    {
        I2C_BOARD_INFO("summit_smb347", 0x5F),
        .irq = OMAP_GPIO_IRQ(OMAP4_CHARGER_IRQ),
    },
    {
        I2C_BOARD_INFO("summit_smb347", 0x06),
        .irq = OMAP_GPIO_IRQ(OMAP4_CHARGER_IRQ),
    },
#endif  
#ifdef CONFIG_SENSORS_LM75
        { I2C_BOARD_INFO("tmp105", 0x48),    },
#endif

#ifdef CONFIG_BOSCH_BMA250	
	{ I2C_BOARD_INFO("bma250", 0x18),}
#endif

};

static void __init blaze_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{

	omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &sdp4430_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &sdp4430_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &sdp4430_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &sdp4430_i2c_4_bus_pdata);

	omap4_pmic_init("twl6030", &sdp4430_twldata);
	i2c_register_board_info(1, sdp4430_i2c_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	omap_register_i2c_bus(3, 400, sdp4430_i2c_3_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, sdp4430_i2c_4_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_4_boardinfo));

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);
	gpio_request(6, "msecure");
	gpio_direction_output(6, 1);

	return 0;
}

static struct omap_dss_device sdp4430_otter1_device = {
	.name			= "lcd2",
	.driver_name		= "otter1_panel_drv",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
	.panel = {
		.width_in_um    = 158000,
		.height_in_um   = 92000,
	},
};

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#if 0
static int dsi1_panel_set_backlight(struct omap_dss_device *dssdev, int level)
{
	int r;

	r = twl_i2c_write_u8(TWL_MODULE_PWM, 0x7F, LED_PWM2OFF);
	if (r)
		return r;

	if (level > 1) {
		if (level == 255)
			level = 0x7F;
		else
			level = (~(level/2)) & 0x7F;

		r = twl_i2c_write_u8(TWL_MODULE_PWM, level, LED_PWM2ON);
		if (r)
			return r;
		r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x30, TWL6030_TOGGLE3);
		if (r)
			return r;
	} else if (level <= 1) {
		r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x08, TWL6030_TOGGLE3);
		if (r)
			return r;
		r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x28, TWL6030_TOGGLE3);
		if (r)
			return r;
		r = twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x00, TWL6030_TOGGLE3);
		if (r)
			return r;
	}

	return 0;
}

static struct nokia_dsi_panel_data dsi1_panel;

static void sdp4430_lcd_init(void)
{
	u32 reg;
	int status;

	/* Enable 3 lanes in DSI1 module, disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x7 << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x7 << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);

	/* Panel Taal reset and backlight GPIO init */
	status = gpio_request_one(dsi1_panel.reset_gpio, GPIOF_DIR_OUT,
		"lcd_reset_gpio");
	if (status)
		pr_err("%s: Could not get lcd_reset_gpio\n", __func__);

	if (dsi1_panel.use_ext_te) {
		status = omap_mux_init_signal("gpmc_ncs4.gpio_101",
				OMAP_PIN_INPUT_PULLUP);
		if (status)
			pr_err("%s: Could not get ext_te gpio\n", __func__);
	}

	status = gpio_request_one(LCD_BL_GPIO, GPIOF_DIR_OUT, "lcd_bl_gpio");
	if (status)
		pr_err("%s: Could not get lcd_bl_gpio\n", __func__);

	gpio_set_value(LCD_BL_GPIO, 0);
}

static struct gpio sdp4430_hdmi_gpios[] = {
	{HDMI_GPIO_CT_CP_HPD,  GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_hpd"   },
	{HDMI_GPIO_LS_OE,      GPIOF_OUT_INIT_HIGH,    "hdmi_gpio_ls_oe" },
};


static void sdp4430_hdmi_mux_init(void)
{
	u32 r;
	int status;
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("gpmc_wait2.gpio_100",
			OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("hdmi_cec.hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(HDMI_GPIO_HPD, NULL);
	omap_mux_init_gpio(HDMI_GPIO_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(HDMI_GPIO_HPD);

	status = gpio_request_array(sdp4430_hdmi_gpios,
			ARRAY_SIZE(sdp4430_hdmi_gpios));
	if (status)
		pr_err("%s:Cannot request HDMI GPIOs %x \n", __func__, status);
}



static struct nokia_dsi_panel_data dsi1_panel = {
		.name		= "taal",
		.reset_gpio	= 102,
		.use_ext_te	= false,
		.ext_te_gpio	= 101,
		.esd_interval	= 0,
		.set_backlight	= dsi1_panel_set_backlight,
};

static struct omap_dss_device sdp4430_lcd_device = {
	.name			= "lcd",
	.driver_name		= "taal",
	.type			= OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi1_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
	},

	.clocks = {
		.dispc = {
			.channel = {
				.lck_div	= 1,	/* Logic Clock = 172.8 MHz */
				.pck_div	= 5,	/* Pixel Clock = 34.56 MHz */
				.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn		= 16,	/* Fint = 2.4 MHz */
			.regm		= 180,	/* DDR Clock = 216 MHz */
			.regm_dispc	= 5,	/* PLL1_CLK1 = 172.8 MHz */
			.regm_dsi	= 5,	/* PLL1_CLK2 = 172.8 MHz */

			.lp_clk_div	= 10,	/* LP Clock = 8.64 MHz */
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},
	.channel = OMAP_DSS_CHANNEL_LCD,
	.skip_init = false,
};

static struct omap_dss_device sdp4430_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = HDMI_GPIO_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};
#endif

static struct omap_dss_device *sdp4430_dss_devices[] = {
	//&sdp4430_lcd_device,
	&sdp4430_otter1_device,
	//&sdp4430_hdmi_device,
};

static struct omap_dss_board_info sdp4430_dss_data = {
	.num_devices	= ARRAY_SIZE(sdp4430_dss_devices),
	.devices	= sdp4430_dss_devices,
	.default_device	= &sdp4430_otter1_device,
};

#define BLAZE_FB_RAM_SIZE                (SZ_1M * 5) /* 1024×600 * 4 * 2 */
static struct omapfb_platform_data blaze_fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = BLAZE_FB_RAM_SIZE,
			},
		},
	},
};

#if 0
static void sdp4430_lcd_init(void)
{
	gpio_request(GPIO_3V_ENABLE, "3V_ENABLE");
	gpio_request(GPIO_OMAP_RGB_SHTDN, "OMAP_RGB_SHTDN");
	gpio_direction_output(GPIO_3V_ENABLE, 1);
	gpio_direction_output(GPIO_OMAP_RGB_SHTDN, 1);
}
#endif

static void omap_4430sdp_display_init(void)
{
	/* sdp4430_lcd_init();
	   sdp4430_hdmi_mux_init(); */
	omap_vram_set_sdram_vram(BLAZE_FB_RAM_SIZE, 0);
	omapfb_set_platform_data(&blaze_fb_pdata);
	omap_display_init(&sdp4430_dss_data);
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
    /* OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), */
    OMAP4_MUX(ABE_DMIC_CLK1,OMAP_MUX_MODE3 | OMAP_OFF_EN),//+5V ADO_SPEAK_ENABLE
    OMAP4_MUX(GPMC_AD0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT0
    OMAP4_MUX(GPMC_AD1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT1
    OMAP4_MUX(GPMC_AD2, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT2
    OMAP4_MUX(GPMC_AD3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT3
    OMAP4_MUX(GPMC_AD4, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT4
    OMAP4_MUX(GPMC_AD5, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT5
    OMAP4_MUX(GPMC_AD6, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT6
    OMAP4_MUX(GPMC_AD7, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // SDMMC2_DAT7
    OMAP4_MUX(GPMC_AD9, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(GPMC_AD10, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(GPMC_AD11, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PULL_ENA),
    OMAP4_MUX(GPMC_AD12, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 36
    OMAP4_MUX(GPMC_AD13,OMAP_MUX_MODE3 | OMAP_OFF_EN),//OMAP_RGB_SHTDN
    OMAP4_MUX(GPMC_AD14, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 38
    OMAP4_MUX(GPMC_A18, OMAP_MUX_MODE3), // gpio 42
    OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 43
    OMAP4_MUX(GPMC_A20, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 44
    OMAP4_MUX(GPMC_A21, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 45
    OMAP4_MUX(GPMC_A23,OMAP_MUX_MODE3 | OMAP_OFF_EN),//OMAP_3V_ENABLE ,LCDVCC
    OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 48
    OMAP4_MUX(GPMC_A25, OMAP_MUX_MODE3 | OMAP_PIN_INPUT), // gpio 49
    OMAP4_MUX(GPMC_CLK, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 55
    OMAP4_MUX(GPMC_NADV_ALE, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 56
    OMAP4_MUX(GPMC_NWE, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_OFF_EN | OMAP_OFFOUT_EN), // sdmmc2_cmd
    OMAP4_MUX(GPMC_NBE0_CLE, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 59
    OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 61
    OMAP4_MUX(GPMC_WAIT1, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 62
    OMAP4_MUX(C2C_DATA12, OMAP_MUX_MODE3 | OMAP_OFF_EN ),//Charger en pin
    OMAP4_MUX(C2C_DATA13, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 102
    OMAP4_MUX(UART4_RX, OMAP_MUX_MODE3 | OMAP_OFF_EN | OMAP_OFFOUT_VAL ),//Charger susp pin,
    OMAP4_MUX(USBB1_ULPITLL_DAT6, OMAP_MUX_MODE1 ),//Backlight GPTimer 10,
    OMAP4_MUX(C2C_DATA14, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN ),//gpio 103: MP/ENG Detect,
    OMAP4_MUX(UNIPRO_TX1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN ),//gpio 174: mbid1,
    OMAP4_MUX(UNIPRO_TY1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN ),//gpio 173: mbid0,
    OMAP4_MUX(UNIPRO_RX1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN ),//gpio 177: mbid3,
    OMAP4_MUX(UNIPRO_RY1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN ),//gpio 178: mbid2,
    OMAP4_MUX(UNIPRO_RX0, OMAP_MUX_MODE3 | OMAP_INPUT_EN ),//gpio 175: LCDid1,
    OMAP4_MUX(UNIPRO_RY0, OMAP_MUX_MODE3 | OMAP_INPUT_EN ),//gpio 176: LCDid0,
    OMAP4_MUX(GPMC_NCS0, OMAP_MUX_MODE3 | OMAP_INPUT_EN ),//gpio 50: TPid0,
    OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_INPUT_EN ),//gpio 51: TPid1,
    OMAP4_MUX(UNIPRO_TX0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP ),//gpio 171: Wifi IP select,
    OMAP4_MUX(UNIPRO_TY0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP ),//gpio 172: Wifi IP select,
    /* OMAP4_MUX(C2C_DATA11, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN |OMAP_PIN_OFF_INPUT_PULLUP),//Power button, same as GPMC_WAIT2*/
    /* OMAP4_MUX(C2C_DATA15, OMAP_MUX_MODE7 | OMAP_PULL_ENA),//Power button, same as GPMC_NCS7 */
    OMAP4_MUX(HDMI_HPD, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // HDMI_HDP
    OMAP4_MUX(HDMI_CEC, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // HDMI_CEC
    OMAP4_MUX(HDMI_DDC_SCL, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // HDMI_DDC_SCL
    OMAP4_MUX(HDMI_DDC_SDA, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // HDMI_DDC_SDA
    OMAP4_MUX(CSI21_DX0, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DX0
    OMAP4_MUX(CSI21_DY0, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DY0
    OMAP4_MUX(CSI21_DX1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DX1
    OMAP4_MUX(CSI21_DY1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DY1
    OMAP4_MUX(CSI21_DX2, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DX2
    OMAP4_MUX(CSI21_DY2, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI21_DY2
    OMAP4_MUX(CSI22_DX0, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI22_DX0
    OMAP4_MUX(CSI22_DY0, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI22_DY0
    OMAP4_MUX(CSI22_DX1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI22_DX1
    OMAP4_MUX(CSI22_DY1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CSI22_DY1
    OMAP4_MUX(CAM_SHUTTER, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CAM_SHUTTER
    OMAP4_MUX(CAM_STROBE, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // CAM_STROBE
    OMAP4_MUX(CAM_GLOBALRESET, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 83 
    OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 84 
    OMAP4_MUX(USBB1_ULPITLL_STP, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 85 
    OMAP4_MUX(USBB1_ULPITLL_DIR, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 86 
    OMAP4_MUX(USBB1_ULPITLL_NXT, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 87 
    OMAP4_MUX(USBB1_ULPITLL_DAT0, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 88 
    OMAP4_MUX(USBB1_ULPITLL_DAT1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 89 
    OMAP4_MUX(USBB1_ULPITLL_DAT2, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 90 
    OMAP4_MUX(USBB1_ULPITLL_DAT3, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 91 
    OMAP4_MUX(USBB1_ULPITLL_DAT4, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 92 
    OMAP4_MUX(USBB1_ULPITLL_DAT5, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 93 
    OMAP4_MUX(USBB1_ULPITLL_DAT7, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // USBB1_ULPITLL_DAT7
    OMAP4_MUX(USBB1_HSIC_DATA, OMAP_MUX_MODE7), // USBB1_HSIC_DATA
    OMAP4_MUX(USBB1_HSIC_STROBE, OMAP_MUX_MODE7), // USBB1_HSIC_STROBE
    OMAP4_MUX(USBC1_ICUSB_DP, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // USBC1_ICUSB_DP
    OMAP4_MUX(USBC1_ICUSB_DM, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // USBC1_ICUSB_DM
    OMAP4_MUX(SDMMC1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN), // SDMMC1_CLK

    OMAP4_MUX(ABE_MCBSP2_CLKX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP2_CLKX
    OMAP4_MUX(ABE_MCBSP2_DR, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP2_DR
    OMAP4_MUX(ABE_MCBSP2_DX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP2_DX
    OMAP4_MUX(ABE_MCBSP2_FSX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP2_FSX
    OMAP4_MUX(ABE_MCBSP1_CLKX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP1_CLKX
    OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP1_DR
    OMAP4_MUX(ABE_MCBSP1_DX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP1_DX
    OMAP4_MUX(ABE_MCBSP1_FSX, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_MCBSP1_FSX
    OMAP4_MUX(ABE_PDM_UL_DATA, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_OUTPUT_LOW), // ABE_PDM_UL_DATA
    OMAP4_MUX(ABE_DMIC_DIN1, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 120
    OMAP4_MUX(ABE_DMIC_DIN2, OMAP_MUX_MODE3 | OMAP_PULL_ENA), // gpio 121
    OMAP4_MUX(ABE_DMIC_DIN3, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // ABE_DMIC_DIN3
    OMAP4_MUX(UART2_CTS, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // UART2_CTS
    OMAP4_MUX(UART2_RTS, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // UART2_RTS
    OMAP4_MUX(UART2_RX, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // UART2_RX
    OMAP4_MUX(UART2_TX, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // UART2_TX
    OMAP4_MUX(HDQ_SIO, OMAP_MUX_MODE7), // gpio 127
    OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 139
    OMAP4_MUX(MCSPI1_CS3, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 140
    OMAP4_MUX(UART4_TX, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // gpio 156
    OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // gpio 157
    OMAP4_MUX(USBB2_HSIC_DATA, OMAP_MUX_MODE7), // gpio 169
    OMAP4_MUX(USBB2_HSIC_STROBE, OMAP_MUX_MODE7), // gpio 170
    OMAP4_MUX(UNIPRO_TX0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN), // gpio 171,for factory rndis
    OMAP4_MUX(UNIPRO_TY0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN), // gpio 172,for factory rndis
    OMAP4_MUX(FREF_CLK1_OUT, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // fref_clk1_out
    OMAP4_MUX(DPM_EMU2, OMAP_MUX_MODE7 | OMAP_PULL_ENA), // dpm_emu2
    OMAP4_MUX(I2C1_SCL, OMAP_MUX_MODE0 | OMAP_INPUT_EN), // I2C1_SCL
    OMAP4_MUX(I2C1_SDA, OMAP_MUX_MODE0 | OMAP_INPUT_EN), // I2C1_SDA
    OMAP4_MUX(I2C2_SCL, OMAP_MUX_MODE0 | OMAP_INPUT_EN), // I2C2_SCL
    OMAP4_MUX(I2C2_SDA, OMAP_MUX_MODE0 | OMAP_INPUT_EN), // I2C2_SDA
    OMAP4_MUX(MCSPI1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_INPUT_PULLDOWN), // MCSPI1_CLK
    OMAP4_MUX(MCSPI1_SOMI, OMAP_MUX_MODE0), // MCSPI1_SOMI
    OMAP4_MUX(MCSPI1_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_INPUT_PULLDOWN), // MCSPI1_SIMO
    OMAP4_MUX(MCSPI1_CS0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN | OMAP_PIN_OFF_INPUT_PULLDOWN), // MCSPI1_CS0
    OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE7 | OMAP_INPUT_EN), // MCSPI1_CS1
    OMAP4_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE7 | OMAP_INPUT_EN | OMAP_PULL_ENA | OMAP_PULL_UP), // UART3_CTS_RCTX
    OMAP4_MUX(UART3_RTS_SD, OMAP_MUX_MODE7 | OMAP_INPUT_EN | OMAP_PULL_ENA | OMAP_PULL_UP), // UART3_RTS_SD
    OMAP4_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_INPUT_EN | OMAP_PULL_ENA | OMAP_PULL_UP), // UART3_RX_IRRX
    OMAP4_MUX(MCSPI4_SOMI, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP), // MCSPI4_SOMI
    OMAP4_MUX(DPM_EMU0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), // DPM_EMU0
    OMAP4_MUX(DPM_EMU1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), // DPM_EMU1
    OMAP4_MUX(DPM_EMU7, OMAP_MUX_MODE5 | OMAP_PIN_INPUT_PULLDOWN), // DISPC2_HSYNC

    OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE7 | OMAP_PULL_ENA),				//gpmc_ncs7=GPIO_104-OMAP_TP_RESET
    OMAP4_MUX(GPMC_WAIT2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN |OMAP_PIN_OFF_INPUT_PULLUP),		//GPIO_100-ON_BUTTON
    { .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux board_wkup_mux[] __initdata = {
    OMAP4_MUX(SIM_IO, OMAP_MUX_MODE7 | OMAP_PULL_ENA),
    OMAP4_MUX(SIM_CLK, OMAP_MUX_MODE7 | OMAP_PULL_ENA),
    OMAP4_MUX(SIM_RESET, OMAP_MUX_MODE7 | OMAP_PULL_ENA),
    OMAP4_MUX(SIM_CD, OMAP_MUX_MODE7 | OMAP_PULL_ENA | OMAP_PULL_UP),
    OMAP4_MUX(SIM_PWRCTRL, OMAP_MUX_MODE7 | OMAP_PULL_ENA),
    OMAP4_MUX(FREF_CLK3_REQ, OMAP_MUX_MODE7 | OMAP_PULL_ENA),
    OMAP4_MUX(SR_SCL, OMAP_MUX_MODE0 | OMAP_INPUT_EN),		//No need to use PAD PU
    OMAP4_MUX(SR_SDA, OMAP_MUX_MODE0 | OMAP_INPUT_EN),		//No need to use PAD PU
    OMAP4_MUX(FREF_CLK3_OUT, OMAP_MUX_MODE7 | OMAP_PULL_ENA),		//FREF_CLK3_OUT
    OMAP4_MUX(SYS_PWR_REQ, OMAP_MUX_MODE0 | OMAP_PULL_ENA),		// SYS_PWR_REQ
    OMAP4_MUX(SYS_BOOT7, OMAP_MUX_MODE3 | OMAP_INPUT_EN),		// GPIO_WK10
    { .reg_offset = OMAP_MUX_TERMINATOR },
};

#else
#define board_mux	NULL
#define board_wkup_mux NULL
#endif

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
#if defined(CONFIG_OTTER2)
static __initdata struct emif_device_details emif_devices_1g_1cs = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
	.cs1_device = NULL
};

static __initdata struct emif_device_details emif_devices_1g_2cs = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev
};
#endif

#if defined(CONFIG_OTTER)
static __initdata struct emif_device_details emif_devices_512m = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = NULL
};
#endif

#if 0
static struct omap_device_pad blaze_uart1_pads[] __initdata = {
	{
		.name	= "uart1_cts.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rts.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad blaze_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};
#endif

static struct omap_device_pad blaze_uart3_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart3_rts_sd.uart3_rts_sd",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

#if 0
static struct omap_device_pad blaze_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};


static struct omap_uart_port_info blaze_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};
#endif

static struct omap_uart_port_info blaze_uart_info __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static inline void __init board_serial_init(void)
{
	//omap_serial_init_port_pads(0, blaze_uart1_pads,
	//	ARRAY_SIZE(blaze_uart1_pads), &blaze_uart_info_uncon);
	//omap_serial_init_port_pads(1, blaze_uart2_pads,
	//	ARRAY_SIZE(blaze_uart2_pads), &blaze_uart_info);
	omap_serial_init_port_pads(2, blaze_uart3_pads,
		ARRAY_SIZE(blaze_uart3_pads), &blaze_uart_info);
	//omap_serial_init_port_pads(3, blaze_uart4_pads,
	//	ARRAY_SIZE(blaze_uart4_pads), &blaze_uart_info_uncon);
}

static void omap4_sdp4430_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
}

static struct wl12xx_platform_data omap4_sdp4430_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL,
//	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};

static void omap4_sdp4430_wifi_init(void)
{
	omap4_sdp4430_wifi_mux_init();
	if (wl12xx_set_platform_data(&omap4_sdp4430_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#if 0
static void __init omap4_ehci_ohci_init(void)
{

	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);

	/* Power on the ULPI PHY */
	if (gpio_is_valid(BLAZE_MDM_PWR_EN_GPIO)) {
		gpio_request(BLAZE_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(BLAZE_MDM_PWR_EN_GPIO, 1);
	}

	usbhs_init(&usbhs_bdata);

	return;

}
#endif
#else
//static void __init omap4_ehci_ohci_init(void){}
#endif

static void blaze_set_osc_timings(void)
{
	/* Device Oscilator
	 * tstart = 2ms + 2ms = 4ms.
	 * tshut = Not defined in oscillator data sheet so setting to 1us
	 */
	omap_pm_set_osc_lp_time(4000, 1);
}


/*
 * As OMAP4430 mux HSI and USB signals, when HSI is used (for instance HSI
 * modem is plugged) we should configure HSI pad conf and disable some USB
 * configurations.
 * HSI usage is declared using bootargs variable:
 * board-4430sdp.modem_ipc=hsi
 * Any other or missing value will not setup HSI pad conf, and port_mode[0]
 * will be used by USB.
 * Variable modem_ipc is used to catch bootargs parameter value.
 */
static char *modem_ipc = "n/a";
module_param(modem_ipc, charp, 0);
MODULE_PARM_DESC(modem_ipc, "Modem IPC setting");

#ifdef CONFIG_TOUCHSCREEN_ILITEK
// chris 2011_0124
static void omap_ilitek_init(void)
{
	omap_mux_init_signal("dpm_emu7.dispc2_hsync", \
                OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE);
        
        if (gpio_request(OMAP4_TOUCH_RESET_GPIO , "ilitek_reset_gpio_18") <0 ){
		pr_err("Touch IRQ reset request failed\n");
                return;
	}
        gpio_direction_output(OMAP4_TOUCH_RESET_GPIO, 0);
        gpio_set_value(OMAP4_TOUCH_RESET_GPIO, 1);

	if (gpio_request(OMAP4_TOUCH_IRQ_1, "Touch IRQ") < 0) {
		pr_err("Touch IRQ GPIO request failed\n");
		return;
	}
	gpio_direction_input(OMAP4_TOUCH_IRQ_1);
}
#endif //CONFIG_TOUCHSCREEN_ILITEK

static void __init omap_bowser_fixup(struct machine_desc *desc,
    struct tag *tags,
    char **cmdline, struct meminfo *mi)
{
    pr_err("omap_bowser_fixup\n");
    bowser_init_idme();
}

static void __init omap_4430sdp_init(void)
{
#if 0
	int status;
#endif
	int sd_density;
	int package = OMAP_PACKAGE_CBS;
	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, board_wkup_mux, package);
	#if defined(CONFIG_OTTER)
	omap_emif_setup_device_details(&emif_devices_512m, &emif_devices_512m);
	#else
	sd_density = omap_sdram_density();
	if (sd_density == 0x14){
		omap_emif_setup_device_details(&emif_devices_1g_2cs, &emif_devices_1g_2cs);
	}else {
		omap_emif_setup_device_details(&emif_devices_1g_1cs, &emif_devices_1g_1cs);
	}
	#endif
	omap_board_config = sdp4430_config;
	omap_board_config_size = ARRAY_SIZE(sdp4430_config);

	omap_init_board_version(0);

	quanta_boardids();
//	omap4_audio_conf();
	omap4_create_board_props();
	blaze_pmic_mux_init();
	blaze_set_osc_timings();
	omap4_i2c_init();
	//blaze_sensor_init();
//	blaze_touch_init();
	omap4_register_ion();
	platform_add_devices(sdp4430_devices, ARRAY_SIZE(sdp4430_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	board_serial_init();
	omap4_sdp4430_wifi_init();
	omap4_twl6030_hsmmc_init(mmc);
	/*For smb347*/
	gpio_request(101, "CHARGE-en");
	gpio_direction_output(101, 0);
	gpio_request(155, "CHARGE-SUSP");
	gpio_direction_output(155, 1);
	omap_mux_init_signal("fref_clk4_req.gpio_wk7", \
		OMAP_PIN_INPUT_PULLUP |OMAP_PIN_OFF_WAKEUPENABLE|OMAP_PIN_OFF_INPUT_PULLUP);
#if 0
	/* blaze_modem_init shall be called before omap4_ehci_ohci_init */
	if (!strcmp(modem_ipc, "hsi"))
		blaze_modem_init(true);
	else
		blaze_modem_init(false);

	omap4_ehci_ohci_init();
#endif
	gpio_request(42,"OMAP_GPIO_ADC");
	gpio_direction_output(42,0);
	usb_musb_init(&musb_board_data);
#if 0
	status = omap_ethernet_init();
	if (status) {
		pr_err("Ethernet initialization failed: %d\n", status);
	} else {
		sdp4430_spi_board_info[0].irq = gpio_to_irq(ETH_KS8851_IRQ);
		spi_register_board_info(sdp4430_spi_board_info,
				ARRAY_SIZE(sdp4430_spi_board_info));
	}

	status = omap4_keyboard_init(&sdp4430_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
#endif
	spi_register_board_info(sdp4430_spi_board_info,	ARRAY_SIZE(sdp4430_spi_board_info));
	omap_dmm_init();
	omap_4430sdp_display_init();
	//blaze_panel_init();
	//blaze_keypad_init();
#ifdef CONFIG_OMAP4_DUTY_CYCLE
	init_duty_governor();
#endif
#ifdef CONFIG_TOUCHSCREEN_ILITEK
	omap_ilitek_init();	// chris 2011_0124
#endif //CONFIG_TOUCHSCREEN_ILITEK
	gpio_request(119, "ADO_SPK_ENABLE");
	gpio_direction_output(119, 1);
	gpio_set_value(119, 1);
#if 0
//kc1 backlight
//gpio_94
	omap_writew(0x001B,0x4A1000D6);
	gpio_request(94, "QBL");
	gpio_direction_output(94, 1);
	gpio_set_value(94, 1);

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}
#endif
	omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();
}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}
static void __init omap_4430sdp_reserve(void)
{
	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	printk(KERN_INFO "%s: SMC size=%dMB, addr=0x%x\n",
		__func__, (PHYS_ADDR_SMC_SIZE >> 20), PHYS_ADDR_SMC_MEM);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	printk(KERN_INFO "%s: DUCATI Memory size=%dMB, addr=0x%x\n",
		__func__, (PHYS_ADDR_DUCATI_SIZE >> 20), PHYS_ADDR_DUCATI_MEM);
	/* ipu needs to recognize secure input buffer area as well */
	/* Workaround:using DUCATI_SIZE+90MB to match with Ducati side
	 * address range.
	 * Actually it should be equal to DUCATI_SIZE+ION_HEAP_SECURE_INPUT_SIZE
	 */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE);
#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif

	omap_reserve();
}

MACHINE_START(OMAP_4430SDP, "OMAP4 blaze board")
	/* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_4430sdp_reserve,
	.map_io		= omap_4430sdp_map_io,
    .fixup = omap_bowser_fixup,
	.init_early	= omap_4430sdp_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
