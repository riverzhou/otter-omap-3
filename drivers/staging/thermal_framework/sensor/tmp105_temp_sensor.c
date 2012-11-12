/*
 * TMP105 Temperature sensor driver file
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * Written By Vince Kim <vince.kim@ti.com>
 *
 * Derived Work from: tmp102_temp_sensor.c from
 * Steven King <sfking@fdwdc.com>
 * Sabatier, Sebastien" <s-sabatier1@ti.com>
 * Mandrenko, Ievgen" <ievgen.mandrenko@ti.com>
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
 * Derived Work from: tmp102_temp_sensor.c from
 * Steven King <sfking@fdwdc.com>
 * Sabatier, Sebastien" <s-sabatier1@ti.com>
 * Mandrenko, Ievgen" <ievgen.mandrenko@ti.com>
 *
 */

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <plat/common.h>
#include <plat/tmp105_temp_sensor.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>

#include <linux/thermal_framework.h>

/* Spec is available at http://www.ti.com/product/tmp105  */
#define	TMP105_TEMP_REG			0x00
#define	TMP105_CONF_REG			0x01
/* TMP105 Configuration Register Format */ 
#define		TMP105_CONF_SD		0x01
#define		TMP105_CONF_TM		0x02
#define		TMP105_CONF_POL		0x04
#define		TMP105_CONF_F0		0x08
#define		TMP105_CONF_F1		0x10
#define		TMP105_CONF_CR0		0x20
#define		TMP105_CONF_CR1		0x40
#define		TMP105_CONF_OS		0x80

#define		TMP105_TLOW_REG		0x02
#define		TMP105_THIGH_REG	0x03



/* Addresses scanned */
static const unsigned short normal_i2c[] = { 0x48, 0x49, I2C_CLIENT_END };
/*
 * omap_temp_sensor structure
 * @iclient - I2c client pointer
 * @dev - device pointer
 * @sensor_mutex - Mutex for sysfs, irq and PM
 * @therm_fw - thermal device
 */
struct tmp105_temp_sensor {
	struct i2c_client *iclient;
	struct device *dev;
	struct mutex sensor_mutex;
	struct thermal_dev *therm_fw;
	u16 config_orig;
	unsigned long last_update;
	int temp[3];
	int debug_temp;
};

static inline int tmp105_read_reg_byte(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static inline int tmp105_read_reg_word(struct i2c_client *client, u8 reg)
{
	return swab16(i2c_smbus_read_word_data(client, reg));
}

static inline int tmp105_write_reg(struct i2c_client *client,
				u8 reg, u16 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static inline int tmp105_reg_to_mC(int val)
{
	/*Negative numbers*/
	if (val & 0x8000) {
		val = ~val + 1;
		return -( (val / 128) * 500  )  ;
	}
	return ((val / 128) * 500 );
}

#define LM75_TEMP_MIN (-55000)
#define LM75_TEMP_MAX 125000

/* convert milliCelsius to 8-bit TMP105 register value */
/* TEMP: 0.001C/bit (-55C to +125C)
   REG: (0.5C/bit, two's complement) << 7 */
static inline u8 tmp105_mC_to_reg(int val)
{
	int temp;
	
	if (val < LM75_TEMP_MIN)
		temp = LM75_TEMP_MIN;
	else if (val > LM75_TEMP_MAX)
		temp = LM75_TEMP_MAX;
	else
		temp = val;
	
	temp += (temp<0 ? -250 : 250);
	return ((temp / 500) << 7);
}


static int tmp105_read_current_temp(struct device *dev)
{
	int index = 0 ,status = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp105_temp_sensor *tmp105 = i2c_get_clientdata(client);

	tmp105 = i2c_get_clientdata(client);

	mutex_lock(&tmp105->sensor_mutex);
	if (time_after(jiffies, tmp105->last_update + HZ / 3)) {
		status = tmp105_read_reg_word(client, TMP105_TEMP_REG);
		if (status > -1)
			tmp105->temp[index] = tmp105_reg_to_mC(status);
		tmp105->last_update = jiffies;
	}
	mutex_unlock(&tmp105->sensor_mutex);

	return tmp105->temp[index];
}

static int tmp105_get_temp(struct thermal_dev *tdev)
{
	struct platform_device *pdev = to_platform_device(tdev->dev);
	struct tmp105_temp_sensor *tmp105 = platform_get_drvdata(pdev);

	tmp105->therm_fw->current_temp =
			tmp105_read_current_temp(tdev->dev);

	return tmp105->therm_fw->current_temp;
}

/*
 * sysfs hook functions
 */
static ssize_t tmp105_show_temp_user_space(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp105_temp_sensor *tmp105 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tmp105->debug_temp);
}

static ssize_t tmp105_set_temp_user_space(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tmp105_temp_sensor *tmp105 = i2c_get_clientdata(client);
	long val;

	if (strict_strtol(buf, 10, &val)) {
		count = -EINVAL;
		goto out;
	}

	/* Set new temperature */
	tmp105->debug_temp = val;

	tmp105->therm_fw->current_temp = val;
	thermal_sensor_set_temp(tmp105->therm_fw);
	/* Send a kobj_change */
	kobject_uevent(&tmp105->dev->kobj, KOBJ_CHANGE);

out:
	return count;
}

static int tmp105_temp_sensor_read_temp(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	int temp = tmp105_read_current_temp(dev);

	return sprintf(buf, "%d\n", temp);
}

static DEVICE_ATTR(debug_user, S_IWUSR | S_IRUGO, tmp105_show_temp_user_space,
			  tmp105_set_temp_user_space);
static DEVICE_ATTR(temp1_input, S_IRUGO, tmp105_temp_sensor_read_temp,
			  NULL);

static struct attribute *tmp105_temp_sensor_attributes[] = {
	&dev_attr_temp1_input.attr,
	&dev_attr_debug_user.attr,
	NULL
};

static const struct attribute_group tmp105_temp_sensor_attr_group = {
	.attrs = tmp105_temp_sensor_attributes,
};

static struct thermal_dev_ops tmp105_temp_sensor_ops = {
	.report_temp = tmp105_get_temp,
};

/*  TMP105_CONF_CR0 and TMP105_CONF_CR1 are set to 0 for 9bit (0.5 degreeC )resolution */
#define TMP105_CONFIG  (0) 
static int __devinit tmp105_temp_sensor_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmp105_temp_sensor *tmp105;
	int ret = 0;
	
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "adapter doesn't support SMBus word "
			"transactions\n");

		return -ENODEV;
	}

	tmp105 = kzalloc(sizeof(struct tmp105_temp_sensor), GFP_KERNEL);
	if (!tmp105)
		return -ENOMEM;

	mutex_init(&tmp105->sensor_mutex);
	
	tmp105->iclient = client;
	tmp105->dev = &client->dev;

	kobject_uevent(&client->dev.kobj, KOBJ_ADD);
	i2c_set_clientdata(client, tmp105);

	ret = tmp105_read_reg_byte(client, TMP105_CONF_REG);
	if (ret < 0) {
		dev_err(&client->dev, "error reading config register\n");
		goto free_err;
	}
	tmp105->config_orig = ret;
	

	if (ret != TMP105_CONFIG){
		ret = tmp105_write_reg(client, TMP105_CONF_REG, TMP105_CONFIG);
		if (ret < 0) {
			dev_err(&client->dev, "error writing config register\n");
			goto restore_config_err;
		}
	}

	tmp105->last_update = jiffies - HZ;
	mutex_init(&tmp105->sensor_mutex);

	ret = sysfs_create_group(&client->dev.kobj,
		&tmp105_temp_sensor_attr_group);
	if (ret)
		goto sysfs_create_err;

	tmp105->therm_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (tmp105->therm_fw) {
		tmp105->therm_fw->name = TMP105_SENSOR_NAME;
		tmp105->therm_fw->domain_name = "pcb";
		tmp105->therm_fw->dev = tmp105->dev;
		tmp105->therm_fw->dev_ops = &tmp105_temp_sensor_ops;
		thermal_sensor_dev_register(tmp105->therm_fw);
	} else {
		ret = -ENOMEM;
		goto therm_fw_alloc_err;
	}

	dev_info(&client->dev, "initialized\n");

	return 0;

sysfs_create_err:
	thermal_sensor_dev_unregister(tmp105->therm_fw);
	kfree(tmp105->therm_fw);
restore_config_err:
	tmp105_write_reg(client, TMP105_CONF_REG, tmp105->config_orig);
therm_fw_alloc_err:
free_err:
	mutex_destroy(&tmp105->sensor_mutex);
	kfree(tmp105);

	return ret;
}

static int __devexit tmp105_temp_sensor_remove(struct i2c_client *client)
{
	struct tmp105_temp_sensor *tmp105 = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &tmp105_temp_sensor_attr_group);
	/* Stop monitoring if device was stopped originally */
	if (tmp105->config_orig & TMP105_CONF_SD) {
		int config;

		config = tmp105_read_reg_byte(client, TMP105_CONF_REG);
		if (config >= 0)
			tmp105_write_reg(client, TMP105_CONF_REG,
					 config | TMP105_CONF_SD);
	}

	tmp105_write_reg(client, TMP105_CONF_REG, tmp105->config_orig);
	kfree(tmp105);

	return 0;
}

static void  tmp105_shutdown(struct i2c_client *client)
{
	struct tmp105_temp_sensor *tmp105 = i2c_get_clientdata(client);
	thermal_sensor_dev_unregister(tmp105->therm_fw);
}
#if 0 // Same as tate 
//#ifdef CONFIG_PM
static int tmp105_temp_sensor_suspend(struct i2c_client *client,
			pm_message_t mesg)
{
	int conf = tmp105_read_reg_byte(client, TMP105_CONF_REG);

	if (conf < 0)
		return conf;
	conf |= TMP105_CONF_SD;

	return tmp105_write_reg(client, TMP105_CONF_REG, conf);
}

static int tmp105_temp_sensor_resume(struct i2c_client *client)
{
	int conf = tmp105_read_reg_byte(client, TMP105_CONF_REG);

	if (conf < 0)
		return conf;
	conf &= ~TMP105_CONF_SD;

	return tmp105_write_reg(client, TMP105_CONF_REG, conf);
}

#else

#define tmp105_temp_sensor_suspend NULL
#define tmp105_temp_sensor_resume NULL

#endif /* CONFIG_PM */

static int tmp105_detect(struct i2c_client *new_client,
		       struct i2c_board_info *info)
{
	strlcpy(info->type, "tmp105_temp_sensor", I2C_NAME_SIZE);

	return 0;
}

static const struct i2c_device_id tmp105_id[] = {
	{ "tmp105_temp_sensor", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp105_id);

static struct i2c_driver tmp105_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe = tmp105_temp_sensor_probe,
	.remove = tmp105_temp_sensor_remove,
	.suspend = tmp105_temp_sensor_suspend,
	.resume = tmp105_temp_sensor_resume,
	.driver = {
		.name = "tmp105_temp_sensor",
	},
	.id_table	= tmp105_id,
	.detect		= tmp105_detect,
	.address_list	= normal_i2c,
	.shutdown       = tmp105_shutdown,
};

static int __init tmp105_init(void)
{
	return i2c_add_driver(&tmp105_driver);
}
module_init(tmp105_init);

static void __exit tmp105_exit(void)
{
	i2c_del_driver(&tmp105_driver);
}
module_exit(tmp105_exit);

MODULE_DESCRIPTION("OMAP44XX TMP105 Temperature Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
