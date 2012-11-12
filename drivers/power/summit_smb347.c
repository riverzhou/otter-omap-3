/*
 * summit_smb347.c
 *
 * Summit SMB347 Charger detection Driver
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 * Donald Chan (hoiho@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/sysdev.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/smb347.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/thermal_framework.h>
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif
#ifdef CONFIG_LAB126
#include <linux/metricslog.h>
#define THERMO_METRICS_STR_LEN 128
#endif

#define SUMMIT_SMB347_I2C_ADDRESS	0x5F
#define SUMMIT_SMB347_I2C_ADDRESS_SECONDARY	0x06

#define SUMMIT_SMB347_ID		0xE
#define DRIVER_NAME			"smb347"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Donald Chan"

#define SMB347_CHARGE_CURRENT			0x0
#define SUMMIT_SMB347_INPUT_CURR_LIMIT	0x1
#define SUMMIT_SMB347_FUNCTIONS		0x2
#define SUMMIT_SMB347_FLOAT_VOLTAGE	0x3
#define SUMMIT_SMB347_CHARGE_CONTROL	0x4
#define SUMMIT_SMB347_STAT_TIMERS	0x5
#define SMB347_ENABLE_CONTROL	0x6
#define SUMMIT_SMB347_THERMAL_CONTROL	0x7
#define SUMMIT_SMB347_SYSOK_USB30	0x8
#define SMB347_OTHER_CONTROL_A	0x9
#define SUMMIT_SMB347_OTG_THERM_CONTROL	0xA
#define SUMMIT_SMB347_CELL_TEMP		0xB
#define SUMMIT_SMB347_FAULT_INTERRUPT	0xC
#define SUMMIT_SMB347_INTERRUPT_STAT	0xD
#define SUMMIT_SMB347_SLAVE_ADDR	0xE

#define SMB347_COMMAND_REG_A	0x30
#define SMB347_COMMAND_REG_B	0x31
#define SMB347_COMMAND_REG_C	0x33
#define SMB347_INTSTAT_REG_A	0x35
#define SMB347_INTSTAT_REG_B	0x36
#define SMB347_INTSTAT_REG_C	0x37
#define SMB347_INTSTAT_REG_D	0x38
#define SMB347_INTSTAT_REG_E	0x39
#define SMB347_INTSTAT_REG_F	0x3A
#define SMB347_STATUS_REG_A	0x3B
#define SMB347_STATUS_REG_B	0x3C
#define SMB347_STATUS_REG_C	0x3D
#define SMB347_STATUS_REG_D	0x3E
#define SMB347_STATUS_REG_E	0x3F

#define SUMMIT_SMB347_TLOW_THRESHOLD	37
#define SUMMIT_SMB347_THIGH_THRESHOLD	113
#define SUMMIT_SMB347_VHI_THRESHOLD	4350
#define SUMMIT_SMB347_VLO_THRESHOLD	2500

#define SMB347_IS_APSD_DONE(value)	((value) & (1 << 3))

#define SMB347_APSD_RESULT(value)	((value) & 0x7)
#define SMB347_APSD_RESULT_NONE		(0)
#define SMB347_APSD_RESULT_CDP		(1)
#define SMB347_APSD_RESULT_DCP		(2)
#define SMB347_APSD_RESULT_OTHER	(3)
#define SMB347_APSD_RESULT_SDP		(4)
#define SMB347_APSD_RESULT_ACA		(5)
#define SMB347_APSD_RESULT_TBD_1	(6)
#define SMB347_APSD_RESULT_TBD_2	(7)

#define SMB347_CHARGING_STATUS(value)	(((value) >> 1) & 0x3)

#define SMB347_CHARGING_STATUS_NOT_CHARGING	(0)
#define SMB347_CHARGING_STATUS_PRE_CHARGING	(1)
#define SMB347_CHARGING_STATUS_FAST_CHARGING	(2)
#define SMB347_CHARGING_STATUS_TAPER_CHARGING	(3)
#define SMB347_CHARGING_STATUS_UNKNOWN		(4)

#define SMB347_USB1_5_HC_MODE(value)	(((value) >> 5) & 0x3)

#define SMB347_HC_MODE			(0)
#define SMB347_USB1_MODE		(1)
#define SMB347_USB5_MODE		(2)

#define SMB347_IS_AICL_DONE(value)	((value) & (1 << 4))

#define SMB347_AICL_RESULT(value)	((value) & 0xf)

enum {
	SMB347_USB_MODE_1,
	SMB347_USB_MODE_5,
	SMB347_USB_MODE_HC,
};

struct smb347_priv {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct power_supply ac;
	struct power_supply usb;
	atomic_t usb_online;
	atomic_t ac_online;
	struct otg_transceiver *otg_xceiv;
	struct mutex lock;
	struct notifier_block usb_notifier;
	struct work_struct irq_work;
#if defined(CONFIG_HAS_WAKELOCK)
	struct wake_lock wake_lock;
#endif
	struct regulator *regulator;
	struct thermal_dev *tdev;
	int max_thermal_charge_current;
	int charge_current;
	int thermal_affect;
};

/* DEBUG */
//#define SUMMIT_SMB347_DEBUG

static const struct i2c_device_id summit_smb347_id[] =  {
	{ "smb347", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, summit_smb347_id);

static const int smb347_aicl_results[] = {
				 300,  500,  700,  900,
				1200, 1500, 1800, 2000,
				2200, 2500, 2500, 2500,
				2500, 2500, 2500, 2500
			};

static const int smb347_fast_charge_currents[] = {
				 700,  900, 1200, 1500,
				1800, 2000, 2200, 2500
			};

static const int smb347_precharge_currents[] = { 100, 150, 200, 250 };

static int smb347_i2c_read(struct i2c_client *client,
			u8 reg_num, u8 *value);

static int smb347_i2c_write(struct i2c_client *client,
			u8 reg_num, u8 value);

static inline int smb347_apsd_result(int reg)
{
	if (SMB347_IS_APSD_DONE(reg)) {
		return SMB347_APSD_RESULT(reg);
	} else {
		return -1;
	}
}

static const char *smb347_apsd_result_string(u8 value)
{
	switch (smb347_apsd_result(value)) {
	case SMB347_APSD_RESULT_CDP:
		return "CDP";
		break;
	case SMB347_APSD_RESULT_DCP:
		return "DCP";
		break;
	case SMB347_APSD_RESULT_OTHER:
		return "Other Downstream Port";
		break;
	case SMB347_APSD_RESULT_SDP:
		return "SDP";
		break;
	case SMB347_APSD_RESULT_ACA:
		return "ADA charger";
		break;
	case SMB347_APSD_RESULT_TBD_1:
	case SMB347_APSD_RESULT_TBD_2:
		return "TBD";
		break;
	case -1:
		return "not run";
		break;
	case SMB347_APSD_RESULT_NONE:
	default:
		return "unknown";
		break;
	}
}

static inline int smb347_aicl_current(int reg)
{
	if (SMB347_IS_AICL_DONE(reg)) {
		int value = SMB347_AICL_RESULT(reg);

		if (value >= 0 && value <= 15) {
			return smb347_aicl_results[value];
		}

		return -1;
	} else {
		return -1;
	}
}

static void smb347_aicl_complete(struct smb347_priv *priv)
{
	u8 value = 0xff;
	int ret = -1;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SMB347_STATUS_REG_E, &value);

	if (ret) {
		dev_err(priv->dev,
			"Failed to read SMB347_STATUS_REG_E: %d\n", ret);
		goto done;
	}

	dev_info(priv->dev, "AICL result: %d mA\n",
			smb347_aicl_results[value & 0xf]);

done:
	mutex_unlock(&priv->lock);
	return;
}

static int smb347_config(struct smb347_priv *priv, int enable)
{
	int ret = 0;
	unsigned char value = 0xff;

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_A, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		if (enable) {
			value |= 0x80;
		} else {
			value &= ~0x80;
		}

		if ((ret = smb347_i2c_write(priv->i2c_client,
				SMB347_COMMAND_REG_A, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB347_COMMAND_REG_A: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

struct smb347_priv *g_priv;

void smb347_redo_apsd(int enable)
{
	u8 value;
	smb347_config(g_priv, 1);
	if(enable == 0) {
		smb347_i2c_read(g_priv->i2c_client, SUMMIT_SMB347_CHARGE_CONTROL, &value);
		value &= ~0x04;
		smb347_i2c_write(g_priv->i2c_client, SUMMIT_SMB347_CHARGE_CONTROL, value);
	}
	else {
		smb347_i2c_read(g_priv->i2c_client, SUMMIT_SMB347_CHARGE_CONTROL, &value);
		value |= 0x04;
		smb347_i2c_write(g_priv->i2c_client, SUMMIT_SMB347_CHARGE_CONTROL, value);
	}
	smb347_config(g_priv, 0);
}
EXPORT_SYMBOL(smb347_redo_apsd);

static int smb347_fast_charge_current_limit(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	if (smb347_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_CHARGE_CURRENT, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	ret = -1;

	/* Clear bits 5, 6, 7 */
	value &= ~(0x7 << 5);

	/* Limit Fast Charge Current to 1800 mA */
	value |= (0x4 << 5);

	if ((ret = smb347_i2c_write(priv->i2c_client,
			SMB347_CHARGE_CURRENT, value))) {
		dev_err(priv->dev,
			"%s: Unable to write SMB347_CHARGE_CURRENT: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	if (smb347_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}
done:
	return ret;
}

static int smb347_set_termination_current(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	if (smb347_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_CHARGE_CURRENT, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	ret = -1;

	/* Clear bit 2, set bits 0 and 1 (Termination current = 150 mA) */
	value &= ~(1 << 2);
	value |= 0x03;

	if ((ret = smb347_i2c_write(priv->i2c_client,
			SMB347_CHARGE_CURRENT, value))) {
		dev_err(priv->dev,
			"%s: Unable to write SMB347_CHARGE_CURRENT: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	if (smb347_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to disable writes to CONFIG regs\n");
		ret = -1;
		goto done;
	}

done:
	return ret;
}

static int smb347_set_charge_control(struct smb347_priv *priv)
{
	int ret = -1;
	u8 value = 0xff;

	if (smb347_config(priv, 1)) {
		dev_err(priv->dev,
			"Unable to enable writes to CONFIG regs\n");
		goto done;
	}

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_ENABLE_CONTROL, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_ENABLE_CONTROL: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	ret = -1;

	/* Clear bits 5 and 6 */
	value &= ~(0x3 << 5);

	/* Set bit 5 */
	value |= (0x1 << 5);

	/* Clear bits 4, Set HC mode to Register Control */
	value &= ~(0x1 << 4);

	if ((ret = smb347_i2c_write(priv->i2c_client,
			SMB347_ENABLE_CONTROL, value))) {
		dev_err(priv->dev,
			"%s: Unable to write SMB347_ENABLE_CONTROL: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	if (smb347_config(priv, 0)) {
		dev_err(priv->dev,
			"Unable to disable writes to CONFIG regs\n");
		ret = -1;
		goto done;
	}

done:
	return ret;
}

static int smb347_switch_mode(struct smb347_priv *priv, int mode)
{
	int ret = 0;
	unsigned char value = 0xff;

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_B, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_COMMAND_REG_B: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		switch (mode) {
		case SMB347_USB_MODE_1:
			dev_info(priv->dev, "Switching to USB1 mode\n");
			value &= ~0x01;
			value &= ~0x02;
			break;
		case SMB347_USB_MODE_5:
			dev_info(priv->dev, "Switching to USB5 mode\n");
			value &= ~0x01;
			value |= 0x02;
			break;
		case SMB347_USB_MODE_HC:
			dev_info(priv->dev, "Switching to HC mode\n");
			value |= 0x01;
			break;
		default:
			dev_err(priv->dev, "Unknown USB mode: %d\n", mode);
			return -1;
		}

		if ((ret = smb347_i2c_write(priv->i2c_client,
				SMB347_COMMAND_REG_B, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB347_COMMAND_REG_B: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static void smb347_apsd_complete(struct smb347_priv *priv)
{
	u8 value = 0xff;
	int ret = -1;
	int type = POWER_SUPPLY_TYPE_USB;

	mutex_lock(&priv->lock);

	ret = smb347_i2c_read(priv->i2c_client, SMB347_STATUS_REG_D, &value);

	if (ret) {
		dev_err(priv->dev,
			"Failed to read SMB347_STATUS_REG_D: %d\n", ret);
		mutex_unlock(&priv->lock);
		return;
	}

	dev_info(priv->dev, "Detected charger: %s\n",
			smb347_apsd_result_string(value));

	switch (smb347_apsd_result(value)) {
	case SMB347_APSD_RESULT_SDP:
	case SMB347_APSD_RESULT_CDP:
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock(&priv->wake_lock);
#endif
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 1);

		if (smb347_apsd_result(value) == SMB347_APSD_RESULT_CDP)
			type = POWER_SUPPLY_TYPE_USB_CDP;

		atomic_notifier_call_chain(&priv->otg_xceiv->notifier,
				USB_EVENT_VBUS, &type);

		break;
	case SMB347_APSD_RESULT_DCP:
	case SMB347_APSD_RESULT_OTHER:
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock(&priv->wake_lock);
#endif
		atomic_set(&priv->ac_online, 1);
		atomic_set(&priv->usb_online, 0);

		type = POWER_SUPPLY_TYPE_USB_DCP;

		atomic_notifier_call_chain(&priv->otg_xceiv->notifier,
				USB_EVENT_VBUS, &type);
		break;
	default:
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock_timeout(&priv->wake_lock, msecs_to_jiffies(2000));
#endif
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 0);
		break;
	}

	power_supply_changed(&priv->usb);
	power_supply_changed(&priv->ac);

	if (smb347_apsd_result(value) == SMB347_APSD_RESULT_OTHER) {
		/* Switch to HC mode */
		smb347_switch_mode(priv, SMB347_USB_MODE_HC);
	} else if (smb347_apsd_result(value) == SMB347_APSD_RESULT_SDP) {
		/* Switch to USB5 mode */
		smb347_switch_mode(priv, SMB347_USB_MODE_5);
	}

	mutex_unlock(&priv->lock);

	return;
}

extern int vusb_on;
/* IRQ worker */
static void summit_smb347_irq_worker(struct work_struct *work)
{
	u8 value = 0xff;
	int ret = -1;
	struct smb347_priv *priv = container_of(work,
				struct smb347_priv, irq_work);

	BUG_ON(!priv);

	/* Check interrupt status A register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_A, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_A: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_A is %02x\n", value);
#endif
	}

	/* Check interrupt status B register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_B, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_B: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_B is %02x\n", value);
#endif
	}

	/* Check interrupt status C register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_C, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_C: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_C is %02x\n", value);
#endif
	}

	/* Check interrupt status D register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_D, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_D: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_D is %02x\n", value);
#endif

		/* Check for APSD status */
		if (value & (1 << 7)) {
			if (!(value & (1 << 6))) {
				dev_warn(priv->dev, "Spurious APSD IRQ!\n");
			} else if(vusb_on == 1){
				/*
				 * VUSB already enable,
				 * so complete the APSD.
				 */
				smb347_apsd_complete(priv);
			} else {
				/*
				 *	The summit charger issues a IRQ before VUSB enables,
				 *	ignore this until the VUSB enables.
				 */
				dev_warn(priv->dev, "Ignore the summit APSD IRQ and wait until vusb_on!\n");
			}
		}

		/* Check for AICL status */
		if ((value & (1 << 5)) && (value & (1 << 4))) {
			smb347_aicl_complete(priv);
		}
	}

	/* Check interrupt status E register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_E, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_E: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_E is %02x\n", value);
#endif
	}

	/* Check interrupt status F register */
	ret = smb347_i2c_read(priv->i2c_client, SMB347_INTSTAT_REG_F, &value);

	if (ret) {
		dev_warn(priv->dev,
			"Failed to read SMB347_INTSTAT_REG_F: %d\n", ret);
	} else {
#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev,
			"INTSTAT_REG_F is %02x\n", value);
#endif
	}

	return;
}

static irqreturn_t summit_smb347_irq(int irq, void *data)
{
	struct smb347_priv *priv = (struct smb347_priv *)data;

	schedule_work(&priv->irq_work);

	return IRQ_HANDLED;
}

static int smb347_i2c_read(struct i2c_client *i2c_client,
				u8 reg_num, u8 *value)
{
	struct smb347_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_read_byte_data(i2c_client, reg_num);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
		return error;
	}

	*value = (unsigned char) (error & 0xff);
	return 0;
}

static int smb347_i2c_write(struct i2c_client *i2c_client,
				u8 reg_num, u8 value)
{
	struct smb347_priv *priv = i2c_get_clientdata(i2c_client);
	s32 error;

	error = i2c_smbus_write_byte_data(i2c_client, reg_num, value);

	if (error < 0) {
		dev_err(priv->dev,
			"i2c error at %s: %d\n", __FUNCTION__, error);
	}

	return error;
}

static int summit_smb347_read_id(struct smb347_priv *priv, int *id)
{
	int error = 0;
	unsigned char value = 0xff;

	error = smb347_i2c_read(priv->i2c_client,
				SUMMIT_SMB347_ID, &value);

	if (!error) {
		*id = value;
	}

	return error;
}

static void summit_smb347_init_registers(void)
{
	;
}

/* Enable/disable charging */
static int smb347_enable_charging(struct smb347_priv *priv, int enable)
{
	int ret = -1;
	unsigned char value = 0xff;

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_A, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

	if (!enable) {
		value |= 0x02;
	} else {
		value &= ~(0x02);
	}

	if ((ret = smb347_i2c_write(priv->i2c_client,
			SMB347_COMMAND_REG_A, value))) {
		dev_err(priv->dev,
			"%s: Unable to write SMB347_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	}

done:
	return ret;
}

#if 0
/* Check to see if charger is connected or not */
static int summit_smb347_charger_connected(void)
{
	unsigned char value = 0xff;

	smb347_i2c_read(SUMMIT_SMB347_INTERRUPT_STAT, &value);

	if (value & 0x4)
		return 1;
	else
		return 0;
}

static int summit_smb347_usb_valid(void)
{
	unsigned char value = 0xff;

	smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_E, &value);

	if (summit_smb347_charger_connected() && (value & 0x80))
		return 1;
	else
		return 0;
}

static int smb347_is_charging(void)
{
	u8 val = 0xff;
	int ret = -1;

	ret = smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_C, &val);

	if (ret) {
		printk(KERN_WARNING "%s failed: %d\n", __FUNCTION__, ret);
	} else {
		return (val & 0x01);
	}
}
#endif

static enum power_supply_property smb347_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property smb347_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


/* USB property */
static int smb347_get_usb_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb347_priv *priv = container_of(ps, struct smb347_priv, usb);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->usb_online);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* AC property */
static int smb347_get_ac_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct smb347_priv *priv = container_of(ps, struct smb347_priv, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = atomic_read(&priv->ac_online);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb347_usb_notifier_cb(struct notifier_block *nb,
			unsigned long val, void *data)
{
	struct smb347_priv *priv = container_of(nb, struct smb347_priv,
					usb_notifier);

	switch (val) {
	case USB_EVENT_NONE:
		/* USB disconnected */
		atomic_set(&priv->ac_online, 0);
		atomic_set(&priv->usb_online, 0);

		power_supply_changed(&priv->ac);
		power_supply_changed(&priv->usb);

		dev_info(priv->dev, "USB disconnected\n");

#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock_timeout(&priv->wake_lock, msecs_to_jiffies(2000));
#endif

		break;
	}

	return NOTIFY_OK;
}

static ssize_t smb347_status_a_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, voltage = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_A, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_A: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_A = 0x%02x\n\n", value);
	len += sprintf(buf + len, "Thermal Regulation Status: %s\n",
			(value & (1 << 7)) ? "Active" : "Inactive");
        len += sprintf(buf + len, "THERM Soft Limit Regulation Status: %s\n",
			(value & (1 << 6)) ? "Active" : "Inactive");

	voltage = 3500 + (value & 0x3f) * 20;

	/* Max out at 4500 mV */
	if (voltage > 4500) voltage = 4500;

	len += sprintf(buf + len,
		"Actual Float Voltage after compensation: %d mV\n", voltage);
done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_a, S_IRUGO, smb347_status_a_show, NULL);

static inline int smb347_charging_current(int reg)
{
	int curr = -1;

	if (reg & (1 << 5)) {
		curr = smb347_fast_charge_currents[reg & 0x07];
	} else {
		curr = smb347_precharge_currents[(reg >> 3) & 0x03];
	}

	return curr;
}

static ssize_t smb347_status_b_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_B, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_B: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_B = 0x%02x\n\n", value);
	len += sprintf(buf + len, "USB Suspend Mode: %s\n",
			(value & (1 << 7)) ? "Active" : "Inactive");

	curr = smb347_charging_current(value);

	if (curr != -1) {
		len += sprintf(buf + len, "Actual Charge Current after "
				"compensation: %d mA\n", curr);
	} else {
		len += sprintf(buf + len, "Actual Charge Current after "
				"compensation: Unknown\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_b, S_IRUGO, smb347_status_b_show, NULL);

static ssize_t smb347_status_c_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_C, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_C: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_C = 0x%02x\n\n", value);

	len += sprintf(buf + len, "Charging Enable/Disable: %s\n",
			(value & 0x1) ? "Enabled" : "Disabled");

	switch (SMB347_CHARGING_STATUS(value)) {
	case SMB347_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "Charging Status: Not charging\n");
		break;
	case SMB347_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "Charging Status: Pre-charging\n");
		break;
	case SMB347_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "Charging Status: Fast-charging\n");
		break;
	case SMB347_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "Charging Status: Taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "Charging Status: Unknown\n");
		break;
	}

	len += sprintf(buf + len, "Charger %s hold-off status\n",
			(value & (1 << 3)) ? "in" : "not in");

	len += sprintf(buf + len, "Vbatt %c 2.1 V\n",
			(value & (1 << 4)) ? '<' : '>');

	if (value & (1 << 5)) {
		len += sprintf(buf + len,
			"At least one charging cycle has terminated\n");
	} else {
		len += sprintf(buf + len,
			"No full charge cycle has occurred\n");
	}

	if (value & (1 << 6))
		len += sprintf(buf + len,
			"Charger has encountered an error\n");

	len += sprintf(buf + len, "Charger error %s an IRQ signal\n",
			(value & (1 << 7)) ? "asserts" : "does not assert");

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_c, S_IRUGO, smb347_status_c_show, NULL);

static ssize_t smb347_status_d_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_D, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_D: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_D = 0x%02x\n\n", value);

	if (smb347_apsd_result(value) != -1) {
		len += sprintf(buf + len, "APSD completed, result: %s\n",
				smb347_apsd_result_string(value));
	} else {
		len += sprintf(buf + len, "APSD not completed\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_d, S_IRUGO, smb347_status_d_show, NULL);

static ssize_t smb347_status_e_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_E, &value))) {
		len += sprintf(buf + len,
			"Error reading SMB347_STATUS_REG_E: %d\n", error);

		goto done;
	}

	len += sprintf(buf + len, "SMB347_STATUS_REG_E = 0x%02x\n\n", value);
	len += sprintf(buf + len, "USBIN Input: %s\n",
		(value & (1 << 7)) ? "In Use" : "Not In Use");

	switch (SMB347_USB1_5_HC_MODE(value)) {
	case SMB347_HC_MODE:
		len += sprintf(buf + len, "In HC mode\n");
		break;
	case SMB347_USB1_MODE:
		len += sprintf(buf + len, "In USB1 mode\n");
		break;
	case SMB347_USB5_MODE:
		len += sprintf(buf + len, "In USB5 mode\n");
		break;
	}

	curr = smb347_aicl_current(value);

	if (curr != -1) {
		len += sprintf(buf + len,
			"AICL Completed, Result = %d mA\n", curr);
	} else {
		len += sprintf(buf + len, "AICL not completed\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(status_e, S_IRUGO, smb347_status_e_show, NULL);


static ssize_t smb347_charge_current_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0, curr = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_B, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	curr = smb347_charging_current(value);

	if (curr != -1) {
		len += sprintf(buf + len, "%d\n", curr);
	} else {
		len += sprintf(buf + len, "-1\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static int smb347_force_precharge(struct smb347_priv *priv, int flag)
{
	int ret = -1;
	unsigned char value = 0xff;

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_A, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_COMMAND_REG_A: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		if (flag) {
			value &= ~0x40;
		} else {
			value |= 0x40;
		}

		if ((ret = smb347_i2c_write(priv->i2c_client,
				SMB347_COMMAND_REG_A, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB347_COMMAND_REG_A: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static int smb347_modify_charge_current(struct smb347_priv *priv,
			int fast_charge_current, int precharge_current)
{
	int ret = -1;
	unsigned char value = 0xff;

	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_CHARGE_CURRENT, &value))) {
		dev_err(priv->dev,
			"%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",
			__FUNCTION__, ret);
		goto done;
	} else {
		if (fast_charge_current != -1) {
			value &= ~0xe0;
			value |= (fast_charge_current << 5);
		}

		if (precharge_current != -1) {
			value &= ~0x18;
			value |= (precharge_current << 3);
		}

		if ((ret = smb347_i2c_write(priv->i2c_client,
				SMB347_CHARGE_CURRENT, value))) {
			dev_err(priv->dev,
				"%s: Unable to write SMB347_CHARGE_CURRENT: %d\n",
				__FUNCTION__, ret);
			goto done;
		}

		ret = 0;
	}

done:
	return ret;
}

static ssize_t smb347_charge_current_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int i = 0, value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (value < 0) {
		dev_err(priv->dev, "Invalid charge current %d mA\n", value);
		goto done;
	}

	if (value >= 1800 && priv->max_thermal_charge_current >= 1800) {
		/* Adjust to 1800 mA */
		value = 1800;
		priv->thermal_affect = 0;
	} else if (value > priv->max_thermal_charge_current) {
		/* Adjust to the new limit current */
		value = priv->max_thermal_charge_current;
		priv->thermal_affect = 1;
	} else priv->thermal_affect = 0;

	/* Locate the first smaller current in fast charge current */
	for (i = ARRAY_SIZE(smb347_fast_charge_currents) - 1; i >= 0; i--)
		if (smb347_fast_charge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Disable force precharge, set fast charge current */
		if (smb347_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev, "Enable writes to CONFIG regs\n");
#endif
		if (smb347_force_precharge(priv, 0)) {
			dev_warn(priv->dev,
				"Unable to disable force pre-charge\n");
		} else if (smb347_modify_charge_current(priv, i, -1)) {
			dev_warn(priv->dev,
				"Unable to modify fast charge current\n");
		}

		if (smb347_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev, "Disabled writes to CONFIG regs\n");
#endif

		priv->charge_current = smb347_fast_charge_currents[i];
		goto done;
	}

	/* Locate the first smaller current in precharge current */
	for (i = ARRAY_SIZE(smb347_precharge_currents) - 1; i >= 0; i--)
		if (smb347_precharge_currents[i] <= value)
			break;

	if (i >= 0) {
		/* Force precharge, set pre-charge current */
		if (smb347_config(priv, 1)) {
			dev_err(priv->dev,
				"Unable to enable writes to CONFIG regs\n");
			goto done;
		}

#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev, "Enable writes to CONFIG regs\n");
#endif

		if (smb347_force_precharge(priv, 1)) {
			dev_warn(priv->dev,
				"Unable to force pre-charge\n");
		} else if (smb347_modify_charge_current(priv, -1, i)) {
			dev_warn(priv->dev,
				"Unable to modify pre-charge current\n");
		}

		if (smb347_config(priv, 0)) {
			dev_err(priv->dev,
				"Unable to disable writes to CONFIG regs\n");
			goto done;
		}

#ifdef SUMMIT_SMB347_DEBUG
		dev_info(priv->dev, "Disabled writes to CONFIG regs\n");
#endif

		priv->charge_current = smb347_precharge_currents[i];
		goto done;
	}

	dev_warn(priv->dev,
		"Unable to find a valid charge current setting for %d mA\n",
		value);

done:
	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_current, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_charge_current_show,
			smb347_charge_current_store);

static ssize_t smb347_charge_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_STATUS_REG_C, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x01) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb347_charge_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int value = simple_strtoul(buf, NULL, 10);

	mutex_lock(&priv->lock);

	if (value) {
		smb347_enable_charging(priv, 1);
	} else {
		smb347_enable_charging(priv, 0);
	}

	mutex_unlock(&priv->lock);
	return len;
}
static DEVICE_ATTR(charge_enable, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_charge_enable_show,
			smb347_charge_enable_store);

static ssize_t smb347_suspend_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0xff;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_COMMAND_REG_A, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x04) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static ssize_t smb347_suspend_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t len)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int enable = simple_strtoul(buf, NULL, 10);
	int ret = -1;
	unsigned char value = 0xff;

	mutex_lock(&priv->lock);
	if ((ret = smb347_i2c_read(priv->i2c_client,
			SMB347_COMMAND_REG_A, &value))) {
		printk("%s: Unable to read SMB347_COMMAND_REG_A: %d\n",__FUNCTION__, ret);
		goto done;
	}

	if (enable) {
		value |= 0x04;
	} else {
		value &= ~(0x04);
	}

	if ((ret = smb347_i2c_write(priv->i2c_client,
			SMB347_COMMAND_REG_A, value))) {
		printk("%s: Unable to write SMB347_COMMAND_REG_A: %d\n", __FUNCTION__, ret);
		goto done;
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(suspend_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			smb347_suspend_mode_show,
			smb347_suspend_mode_store);

int summit_suspend = 0;
int smb347_suspend_mode(int enable)
{
	int ret = -1;
	unsigned char value = 0xff;

	mutex_lock(&g_priv->lock);
	if ((ret = smb347_i2c_read(g_priv->i2c_client,
			SMB347_COMMAND_REG_A, &value))) {
		printk("%s: Unable to read SMB347_COMMAND_REG_A: %d\n",__FUNCTION__, ret);
		goto done;
	}

	if (enable) {
		value |= 0x04;
		summit_suspend = 1;
	} else {
		value &= ~(0x04);
		summit_suspend = 0;
	}

	if ((ret = smb347_i2c_write(g_priv->i2c_client,
			SMB347_COMMAND_REG_A, value))) {
		printk("%s: Unable to write SMB347_COMMAND_REG_A: %d\n", __FUNCTION__, ret);
		goto done;
	}

done:
	mutex_unlock(&g_priv->lock);
	return 0;
}

static ssize_t smb347_precharge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x01) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(precharge_timeout, S_IRUGO,
			smb347_precharge_timeout_show, NULL);

static ssize_t smb347_complete_charge_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
				SMB347_INTSTAT_REG_D, &value))) {
		len += sprintf(buf + len, "-1\n");
		goto done;
	}

	if (value & 0x04) {
		len += sprintf(buf + len, "1\n");
	} else {
		len += sprintf(buf + len, "0\n");
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(complete_charge_timeout, S_IRUGO,
			smb347_complete_charge_timeout_show, NULL);

static ssize_t smb347_charge_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
	int error = 0;
	ssize_t len = 0;
	u8 value = 0x00;

	mutex_lock(&priv->lock);

	if ((error = smb347_i2c_read(priv->i2c_client,
			SMB347_STATUS_REG_C, &value))) {
		len += sprintf(buf + len, "error\n");
		goto done;
	}

	switch (SMB347_CHARGING_STATUS(value)) {
	case SMB347_CHARGING_STATUS_NOT_CHARGING:
		len += sprintf(buf + len, "not-charging\n");
		break;
	case SMB347_CHARGING_STATUS_PRE_CHARGING:
		len += sprintf(buf + len, "pre-charging\n");
		break;
	case SMB347_CHARGING_STATUS_FAST_CHARGING:
		len += sprintf(buf + len, "fast-charging\n");
		break;
	case SMB347_CHARGING_STATUS_TAPER_CHARGING:
		len += sprintf(buf + len, "taper-charging\n");
		break;
	default:
		len += sprintf(buf + len, "unknown\n");
		break;
	}

done:
	mutex_unlock(&priv->lock);
	return len;
}

static DEVICE_ATTR(charge_status, S_IRUGO,
			smb347_charge_status_show, NULL);

static struct attribute *smb347_attrs[] = {
	&dev_attr_status_a.attr,
	&dev_attr_status_b.attr,
	&dev_attr_status_c.attr,
	&dev_attr_status_d.attr,
	&dev_attr_status_e.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_charge_enable.attr,
	&dev_attr_suspend_mode.attr,
	&dev_attr_complete_charge_timeout.attr,
	&dev_attr_precharge_timeout.attr,
	&dev_attr_charge_status.attr,
	NULL,
};

static struct attribute_group smb347_attrs_group = {
	.attrs = smb347_attrs,
};

#define THERMAL_PREFIX "Thermal Policy: Charger cooling agent: "
#define THERMAL_INFO(fmt, args...) do { printk(KERN_INFO THERMAL_PREFIX fmt, ## args); } while(0)

#ifdef THERMAL_DEBUG
#define THERMAL_DBG(fmt, args...) do { printk(KERN_DEBUG THERMAL_PREFIX fmt, ## args); } while(0)
#else
#define THERMAL_DBG(fmt, args...) do {} while(0)
#endif
static int smb347_apply_cooling(struct thermal_dev *dev,
				int level)
{
	struct smb347_priv *priv = i2c_get_clientdata(to_i2c_client(dev->dev));
	int current_limit, previous_max_charge_current;	
	unsigned char temp = 0xff;
	int ret = -1;
	static int previous_cooling_level = 0, new_cooling_level = 0;
#ifdef CONFIG_LAB126
	char *thermal_metric_prefix = "charger_cooling:def:monitor=1";
	char buf[THERMO_METRICS_STR_LEN];
#endif
	/* transform into current limitation */
	current_limit = thermal_cooling_device_reduction_get(dev, level);

	if (current_limit < 0 || current_limit > 1800)
		return -EINVAL;

	THERMAL_DBG("%s : %d : %d\n",__func__,__LINE__,current_limit);

	/* for logging ...*/
	previous_max_charge_current = priv->max_thermal_charge_current;

	new_cooling_level = level;

	if (current_limit != previous_max_charge_current) {	
		smb347_config(priv, 1);

		switch (current_limit){
			case 1800:
				/* If summit charger is in suspend mode, wake up it. */
				if (summit_suspend == 1) smb347_suspend_mode(0);

				/* Set the limitation of input current to 1800 mA. */
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x66);

				/* Redo the AICL to change the input current limit. */
				ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
				if (ret) {
					pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
					return ret;
				}
				/* Disable AICL */
				temp &= ~0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Enable AICL */
				temp |= 0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Set the charging current to 1800 mA. */
				ret = smb347_i2c_read(priv->i2c_client,SMB347_CHARGE_CURRENT, &temp);
				if (ret) {
					pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",__FUNCTION__, ret);
					return ret;
				}
				temp &= ~0xE0;
				temp |= 4 << 5;
				smb347_i2c_write(priv->i2c_client,SMB347_CHARGE_CURRENT, temp);
				priv->max_thermal_charge_current = 1800;
				break;

			case 1500:
				/* If summit charger is in suspend mode, wake up it. */
				if (summit_suspend == 1) smb347_suspend_mode(0);

				/* Set the limitation of input current to 1500 mA. */
				smb347_i2c_write(priv->i2c_client,SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x65);

				/* Redo the AICL to change the input current limit. */
				ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
				if (ret) {
					pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
					return ret;
				}
				/* Disable AICL */
				temp &= ~0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Enable AICL */
				temp |= 0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Set the charging current to 1500 mA. */
				ret = smb347_i2c_read(priv->i2c_client,SMB347_CHARGE_CURRENT, &temp);
				if (ret) {
					pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",__FUNCTION__, ret);
					return ret;
				}

				temp &= ~0xE0;
				temp |= 3 << 5;
				smb347_i2c_write(priv->i2c_client,SMB347_CHARGE_CURRENT, temp);

				priv->max_thermal_charge_current = 1500;
				break;

			case 1200:
				/* If summit charger is in suspend mode, wake up it. */
				if (summit_suspend == 1) smb347_suspend_mode(0);

				/* Set the limitation of input current to 1200 mA. */
				smb347_i2c_write(priv->i2c_client,SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x64);

				/* Redo the AICL to change the input current limit. */
				ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
				if (ret) {
					pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
					return ret;
				}
				/* Disable AICL */
				temp &= ~0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Enable AICL */
				temp |= 0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Set the charging current to 1200 mA. */
				ret = smb347_i2c_read(priv->i2c_client,SMB347_CHARGE_CURRENT, &temp);
				if (ret) {
					pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",__FUNCTION__, ret);
					return ret;
				}

				temp &= ~0xE0;
				temp |= 2 << 5;
				smb347_i2c_write(priv->i2c_client,SMB347_CHARGE_CURRENT, temp);

				priv->max_thermal_charge_current = 1200;
				break;

			case 900:
				/* If summit charger is in suspend mode, wake up it. */
				if (summit_suspend == 1) smb347_suspend_mode(0);

				/* Set the limitation of input current to 900 mA. */
				smb347_i2c_write(priv->i2c_client,SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x63);

				/* Redo the AICL to change the input current limit. */
				ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
				if (ret) {
					pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
					return ret;
				}
				/* Disable AICL */
				temp &= ~0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Enable AICL */
				temp |= 0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Set the charging current to 900 mA. */
				ret = smb347_i2c_read(priv->i2c_client,SMB347_CHARGE_CURRENT, &temp);
				if (ret) {
					pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",__FUNCTION__, ret);
					return ret;
				}

				temp &= ~0xE0;
				temp |= 1 << 5;
				smb347_i2c_write(priv->i2c_client,SMB347_CHARGE_CURRENT, temp);

				priv->max_thermal_charge_current = 900;
				break;

			case 0:
				/* Receiving no charging command, summit charger enters to suspend mode. */
				if (summit_suspend == 0) smb347_suspend_mode(1);

				/* Set the limitation of input current to 1800 mA. */
				smb347_i2c_write(priv->i2c_client,SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x66);

				/* Redo the AICL to change the input current limit. */
				ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
				if (ret) {
					pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
					return ret;
				}
				/* Disable AICL */
				temp &= ~0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				/* Enable AICL */
				temp |= 0x10;
				smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

				msleep(10);

				priv->max_thermal_charge_current = 0;
				break;
		}

		smb347_config(priv,0);

		THERMAL_INFO("max charge current transision from %d to %d",previous_max_charge_current,priv->max_thermal_charge_current); 
	}

#ifdef CONFIG_LAB126
		if ( previous_cooling_level == 0 ) {
			snprintf(buf,THERMO_METRICS_STR_LEN,"%s,throttling=%s:", thermal_metric_prefix, "start");
			log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
		} else if ( new_cooling_level == 0 ) {
			snprintf(buf,THERMO_METRICS_STR_LEN,"%s,throttling=%s:", thermal_metric_prefix, "stop");
			log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
		}
#endif
	previous_cooling_level = new_cooling_level;

	return 0;
}

static struct thermal_dev_ops smb347_cooling_ops = {
	.cool_device = smb347_apply_cooling,
};

static struct thermal_dev case_thermal_dev = {
	.name		= "charger_cooling",
	.domain_name	= "case",
	.dev_ops	= &smb347_cooling_ops,
};



int summit_ready = 0;
EXPORT_SYMBOL(summit_ready);
static int summit_smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct smb347_priv *priv = NULL;
	struct smb347_platform_data *pdata = NULL;
	struct regulator *regulator = NULL;
	int ret = 0, chip_id = 0;
	struct thermal_dev *tdev = NULL;
#ifdef SUMMIT_SMB347_DEBUG
	int i = 0;
	u8 value = 0;
#endif

	/*
	 * FIXME: Enable regulators for GPIO0 here,
	 *        which we really shouldn't
	 */
	pdata = dev_get_platdata(&client->dev);

	tdev = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (tdev == NULL){
		ret = -ENOMEM;
		goto err5;
	}

	if (pdata && pdata->regulator_name) {
		regulator = regulator_get(&client->dev, pdata->regulator_name);

		if (IS_ERR(regulator)) {
			pr_err("Unable to get regulator for gpio0\n");
			ret = -EINVAL;
			goto err5;
		}

		if (regulator_set_voltage(regulator, 1800000, 1800000)) {
			pr_err("Unable to set regulator to 1.8V\n");
			ret = -EINVAL;
			goto err5;
		}

		regulator_enable(regulator);
	}

	if (!(priv = kzalloc(sizeof(*priv), GFP_KERNEL))) {
		pr_err("Out of memory\n");
		ret = -ENOMEM;
		goto err5;
	}

	g_priv = priv;
	priv->regulator = regulator;

	/* Set up I2C structs */
	priv->i2c_client = client;
	i2c_set_clientdata(client, priv);

	/* Set up mutex */
	mutex_init(&priv->lock);

	/* Set up dev pointer */
	priv->dev = &client->dev;

	/* Check for device ID */
	if (summit_smb347_read_id(priv, &chip_id) < 0) {
		client->addr = SUMMIT_SMB347_I2C_ADDRESS_SECONDARY;
		if (summit_smb347_read_id(priv, &chip_id) < 0) {
			pr_err("Unable to detect device ID\n");
			ret = -ENODEV;
			goto err4;
		}
	}

	dev_info(priv->dev, "SMB347 detected, addr=0x%02x chip_id=0x%0x\n",
		 client->addr, chip_id);

	/* Set up USB OTG notifier */
	priv->otg_xceiv = otg_get_transceiver();
	priv->usb_notifier.notifier_call = smb347_usb_notifier_cb;

	if ((ret = otg_register_notifier(priv->otg_xceiv,
				&priv->usb_notifier))) {
		dev_err(priv->dev, "otg_register_notifier failed: %d\n", ret);
		goto err4;
	}

	/* Set up and register the power_supply structs */
	priv->ac.name = "smb347_ac";
	priv->ac.type = POWER_SUPPLY_TYPE_MAINS;
	priv->ac.get_property = smb347_get_ac_property;
	priv->ac.properties = smb347_charger_props;
	priv->ac.num_properties = ARRAY_SIZE(smb347_charger_props);

	priv->usb.name = "smb347_usb";
	priv->usb.type = POWER_SUPPLY_TYPE_USB;
	priv->usb.get_property = smb347_get_usb_property;
	priv->usb.properties = smb347_usb_props;
	priv->usb.num_properties = ARRAY_SIZE(smb347_usb_props);
	priv->charge_current = 0;
	priv->thermal_affect = 0;
	if ((ret = power_supply_register(&client->dev, &priv->ac))) {
		dev_err(priv->dev,
			"failed to register ac power supply: %d\n", ret);
		goto err3;
	}

	if ((ret = power_supply_register(&client->dev, &priv->usb))) {
		dev_err(priv->dev,
			"failed to register usb power supply: %d\n", ret);
		goto err2;
	}

	/* Register the sysfs nodes */
	if ((ret = sysfs_create_group(&priv->dev->kobj,
					&smb347_attrs_group))) {
		dev_err(priv->dev, "Unable to create sysfs group\n");
		goto err1;
	}

	memcpy(tdev, &case_thermal_dev, sizeof(struct thermal_dev));
	tdev->dev = priv->dev;
	ret = thermal_cooling_dev_register(tdev);

	if (ret < 0) {
		goto err1;
	}
	priv->tdev = tdev;

#ifdef SUMMIT_SMB347_DEBUG
	/* Dump config, command and status registers */
	for (i = 0; i <= 0xe; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "cfg_reg=0x%x, value=0x%x\n", i, value);
	}

	for (i = 0x30; i <= 0x33; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "cmd_reg=0x%x, value=0x%x\n", i, value);
	}

	for (i = 0x3b; i <= 0x3f; i++) {
		ret = smb347_i2c_read(priv->i2c_client, i, &value);
		dev_info(priv->dev, "stats_reg=%d, value=0x%x\n", i, value);
	}
#endif

#if defined(CONFIG_HAS_WAKELOCK)
	/* Initialize charger wake lock */
	wake_lock_init(&priv->wake_lock, WAKE_LOCK_SUSPEND, "smb347");
#endif

	/* Limit the fast charge current to 1800 mA */
	smb347_fast_charge_current_limit(priv);

	/* for thermal logging;
	 * Make sure max_thermal_charge_current has the same value
	 * set by above smb347_fast_charge_current_limit
	 */
	priv->max_thermal_charge_current = 1800;

	/* Enable charge enable control via I2C (active low) */
	smb347_set_charge_control(priv);

	/* Limit termination current to 150 mA */
	smb347_set_termination_current(priv);

	/* Init work */
	INIT_WORK(&priv->irq_work, summit_smb347_irq_worker);

	/* Scrub through the registers to ack any interrupts */
	summit_smb347_irq_worker(&priv->irq_work);

	/* Configure IRQ as GPIO input */
	if (priv->i2c_client->irq != -1) {
		gpio_request(priv->i2c_client->irq, "smb347-irq");
		gpio_direction_input(priv->i2c_client->irq);
	}

	/* Enable interrupt */
	if (priv->i2c_client->irq != -1 &&
		request_threaded_irq(priv->i2c_client->irq,
			NULL, summit_smb347_irq, IRQF_TRIGGER_RISING,
			"smb347_irq", priv)) {
		dev_err(priv->dev, "Unable to set up threaded IRQ\n");
		goto err1;
	}

	/* Notify to TWL6032 USB PHY, while the summit charger is ready. */
	summit_ready = 1;
	return 0;

err1:
	power_supply_unregister(&priv->usb);
err2:
	power_supply_unregister(&priv->ac);
err3:
	otg_unregister_notifier(priv->otg_xceiv, &priv->usb_notifier);
err4:
	i2c_set_clientdata(client, NULL);
err5:
	if (priv && priv->regulator)
		regulator_put(priv->regulator);

	kfree(priv);

	return ret;
}

static int summit_smb347_remove(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	/* Free IRQ and stop any pending IRQ work */
	free_irq(priv->i2c_client->irq, priv);
	cancel_work_sync(&priv->irq_work);

	wake_lock_destroy(&priv->wake_lock);

	sysfs_remove_group(&priv->dev->kobj, &smb347_attrs_group);

	power_supply_unregister(&priv->usb);
	power_supply_unregister(&priv->ac);

	otg_unregister_notifier(priv->otg_xceiv, &priv->usb_notifier);

	if (priv && priv->regulator)
		regulator_put(priv->regulator);

	i2c_set_clientdata(client, NULL);
	
	thermal_cooling_dev_unregister(priv->tdev);
	kfree(priv);

	return 0;
}

static void summit_smb347_shutdown(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);
	unsigned char temp = 0xff;
	int ret = -1;

	smb347_config(priv, 1);

	if(summit_suspend == 1){

		smb347_suspend_mode(0);

		/* Recovery the limitation of input current to 1800 mA. */
		smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x66);

		/* Redo the AICL to change the input current limit. */
		ret = smb347_i2c_read(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, &temp);
		if (ret) {
			pr_err("%s: Unable to read SUMMIT_SMB347_FUNCTIONS: %d\n",__FUNCTION__, ret);
			return;
		}
		/* Disable AICL */
		temp &= ~0x10;
		smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

		msleep(10);

		/* Enable AICL */
		temp |= 0x10;
		smb347_i2c_write(priv->i2c_client, SUMMIT_SMB347_FUNCTIONS, temp);

		msleep(10);

		/* Set the charging current to 1800 mA. */
		ret = smb347_i2c_read(priv->i2c_client,SMB347_CHARGE_CURRENT, &temp);
		if (ret) {
			pr_err("%s: Unable to read SMB347_CHARGE_CURRENT: %d\n",__FUNCTION__, ret);
			return;
		}
		temp &= ~0xE0;
		temp |= 4 << 5;
		smb347_i2c_write(priv->i2c_client,SMB347_CHARGE_CURRENT, temp);
	}

	smb347_config(priv, 0);

	dev_info(priv->dev, "Shutting down\n");

	summit_smb347_remove(client);

	return;
}

static int summit_smb347_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	dev_info(priv->dev, "Entering suspend, event = 0x%04x\n", mesg.event);

	cancel_work_sync(&priv->irq_work);

	dev_info(priv->dev, "Finishing suspend\n");

	return 0;
}

static int summit_smb347_resume(struct i2c_client *client)
{
	struct smb347_priv *priv = i2c_get_clientdata(client);

	dev_info(priv->dev, "Finishing resume\n");

	return 0;
}

static unsigned short normal_i2c[] = { SUMMIT_SMB347_I2C_ADDRESS, I2C_CLIENT_END };

static struct i2c_driver summit_smb347_i2c_driver = {
	.driver = {
			.name = "smb347",
		},
	.probe = summit_smb347_probe,
	.remove = summit_smb347_remove,
	.id_table = summit_smb347_id,
	.address_list = normal_i2c,
	.suspend = summit_smb347_suspend,
	.resume = summit_smb347_resume,
	.shutdown = summit_smb347_shutdown,
};

static int __init summit_smb347_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Summit SMB347 Driver\n");

	if ((ret = i2c_add_driver(&summit_smb347_i2c_driver))) {
		printk(KERN_ERR "summit_smb347: Could not add driver\n");
		return ret;
	}

	printk(KERN_INFO "SMB347 Driver added\n");

	return 0;
}

static void __exit summit_smb347_exit(void)
{
	i2c_del_driver(&summit_smb347_i2c_driver);
}

module_init(summit_smb347_init);
module_exit(summit_smb347_exit);

MODULE_DESCRIPTION("Summit SMB347 Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
