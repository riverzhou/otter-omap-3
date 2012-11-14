/*
 *  stk_i2c_als22x7.c - Linux kernel modules for ambient light sensor
 *
 *  Copyright (C) 2011 Patrick Chang / SenseTek <patrick_chang@sitronix.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include "stk_i2c_als22x7.h"
#include "stk_defines.h"
#include "stk_lk_defs.h"

#define STKALS_DRV_NAME	"stk_als"
#define DRIVER_VERSION		"1.5.0"
#define DEVICE_NAME "stk-oss"

#define ODR_DELAY (1000/CONFIG_STK_ALS_SAMPLING_RATE)


#define stk_writeb i2c_smbus_write_byte
#define stk_readb i2c_smbus_read_byte


#define ABS(x) ((x)>=0? (x):(-x))

#define STK_LOCK0 mutex_unlock(&stkals_io_lock)
#define STK_LOCK1 mutex_lock(&stkals_io_lock)

static uint32_t init_all_setting(void);
static int32_t get_lux(void);
static int32_t set_it(uint32_t it);
static int32_t set_gain(uint32_t gain);
static int32_t enable_als(int enable);
static uint32_t get_enable(void);


static int polling_function(void* arg);
static struct task_struct *polling_tsk;


static struct mutex stkals_io_lock;
static struct completion thread_completion;
static struct stkals22x7_data* pStkAlsData = NULL;

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
static int32_t als_transmittance = CONFIG_STK_ALS_TRANSMITTANCE;
#endif //CONFIG_STK_ALS_TRANSMITTANCE_TUNING

//////////////////////
static uint32_t init_all_setting(void)
{
	set_it(2); //x2
	set_gain(1); //x2
	stk_writeb(pStkAlsData->client2,64);
	return true;
}

static int32_t set_it(uint32_t it)
{
	pStkAlsData->it = it;
	pStkAlsData->reg1 &= (~STK_ALS_CMD1_IT_MASK);
	pStkAlsData->reg1 |= STK_ALS_CMD1_IT(it);
	return stk_writeb(pStkAlsData->client1,pStkAlsData->reg1);
}
static int32_t set_gain(uint32_t gain)
{
	pStkAlsData->gain = gain;
	pStkAlsData->reg1 &= (~STK_ALS_CMD1_GAIN_MASK);
	pStkAlsData->reg1 |= STK_ALS_CMD1_GAIN(gain);
	return stk_writeb(pStkAlsData->client1,pStkAlsData->reg1);
}

static int32_t stkals_set_power_state(unsigned int shutdown)
{
	pStkAlsData->reg1 &= (~STK_ALS_CMD1_SD_MASK);
	pStkAlsData->reg1 |= STK_ALS_CMD1_SD(shutdown);
	pStkAlsData->power_down_state = shutdown;
	return stk_writeb(pStkAlsData->client1,pStkAlsData->reg1);
}

inline void report_event(struct input_dev* dev,int32_t report_value)
{
	input_report_abs(dev, ABS_MISC, report_value);
	input_sync(dev);
#if 0
	INFO("STK ALS : als input event %d lux\n",report_value);
#endif
}

inline void update_and_check_report_als(int32_t lux)
{
    int32_t lux_last;
    lux_last = pStkAlsData->als_lux_last;

    if (unlikely(abs(lux - lux_last)>=CONFIG_STK_ALS_CHANGE_THRESHOLD))
    {
        pStkAlsData->als_lux_last = lux;
        report_event(pStkAlsData->input_dev,lux);
    }
}

static int polling_function(void* arg)
{
	uint32_t lux;
	uint32_t delay;
	init_completion(&thread_completion);
	while (1)
	{

		STK_LOCK(1);
		delay = pStkAlsData->als_delay;
		lux = get_lux();
        update_and_check_report_als(lux);
		if (pStkAlsData->bThreadRunning == 0)
			break;
		STK_LOCK(0);
		msleep(delay);
	};

    STK_LOCK(0);
    complete(&thread_completion);

	return 0;
}

static int32_t enable_als(int enable)
{
	int32_t ret;
	ret = stkals_set_power_state(enable?0:1);
	if (enable)
	{

		if (pStkAlsData->bThreadRunning == 0)
		{
			pStkAlsData->als_lux_last = 0;
			pStkAlsData->bThreadRunning = 1;
			polling_tsk = kthread_run(polling_function,NULL,"als_polling");
		}
		else
		{
		    WARNING("STK_ALS : thread has running\n");
        }
	}
	else
	{
		if (pStkAlsData->bThreadRunning)
		{
			pStkAlsData->bThreadRunning = 0;
			STK_LOCK(0);
			wait_for_completion(&thread_completion);
			STK_LOCK(1);
			polling_tsk = NULL;
		}
	}
	return ret;
}

static uint32_t get_enable(void)
{
	return (pStkAlsData->power_down_state)?0:1;
}
inline int32_t alscode2lux(uint32_t alscode)
{
    alscode = (alscode<<9) + (alscode<<6);
    // x 576
    // gain + it --> x 10.64 ==> total ~ x 6000
    // Org : 1 code = 0.6 lux
    // ==> 6000 code = 0.6 lux  (--> Transmittance Factor = 10,000 if directly incident)
#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    alscode/=als_transmittance;
#else
    alscode/=CONFIG_STK_ALS_TRANSMITTANCE;
#endif //CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    return alscode;
}
inline uint32_t lux2alscode(uint32_t lux)
{

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    lux*=als_transmittance;
#else
    lux*=CONFIG_STK_ALS_TRANSMITTANCE;
#endif //CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    lux/=576;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;
}

static int32_t get_lux()
{
	int32_t raw;
	stk_writeb(pStkAlsData->client2,(uint8_t)(pStkAlsData->als_lux_last&0x0f)|pStkAlsData->reg2);
	raw = (stk_readb(pStkAlsData->client1) << 8) | stk_readb(pStkAlsData->client2);
	//1 code = 0.6 lux (for 100ms GAIN = 00)
	// x10.64
	return alscode2lux(raw);

}

#ifdef CONFIG_STK_SYSFS_DBG
// For Debug
static ssize_t help_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
     return sprintf(buf, "Usage : cat xxxx\nor echo val > xxxx\
     \nWhere xxxx = lux : RO (0~by your setting)\
     \nals_enable : RW (0~1)\nals_transmittance : RW (1~10000)\n");

}

static ssize_t lux_range_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    return sprintf(buf, "%d\n", alscode2lux((1<<16) -1));//full code

}


static ssize_t als_enable_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t enable;
    STK_LOCK(1);
    enable = pStkAlsData->bThreadRunning;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", enable);
}


static ssize_t als_enable_store(struct kobject *kobj,
                                struct kobj_attribute *attr,
                                const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    enable_als(value);
    STK_LOCK(0);
    return len;
}


static ssize_t lux_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t lux;
    STK_LOCK(1);
    lux = pStkAlsData->als_lux_last;
    STK_LOCK(0);
    return sprintf(buf, "%d lux\n", lux);
}


#endif //CONFIG_STK_SYSFS_DBG

#define __DEBUG_SYSFS_BIN 1

ssize_t als_lux_range_read(struct kobject *kobj, struct bin_attribute *bin_attr,	char * buffer, loff_t offset, size_t count)
{
    uint32_t* pDst = (uint32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK ALS Driver : Error --> Read Lux Range(bin) size !=4\n");
        return 0;
    }
#endif

    *pDst = alscode2lux((1<<16)-1);
    return sizeof(uint32_t);
}

ssize_t als_lux_resolution_read(struct kobject *kobj, struct bin_attribute *bin_attr,	char * buffer, loff_t offset, size_t count)
{
    uint32_t* pDst = (uint32_t*)buffer;
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK ALS Driver : Error --> Read Distance Range(bin) size !=4\n");
        return 0;
    }
#endif
   // means 1 lux for Android
  *pDst = 1;
  return sizeof(uint32_t);
}

ssize_t lux_bin_read(struct kobject *kobj, struct bin_attribute *bin_attr,	char * buffer, loff_t offset, size_t count)
{

    int32_t *pDst = (int32_t*)buffer;

#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK ALS Driver : Error --> Read Lux(bin) size !=4\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    *pDst = pStkAlsData->als_lux_last;
    STK_LOCK(0);
    return sizeof(uint32_t);
}


ssize_t als_enable_read(struct kobject *kobj, struct bin_attribute *bin_attr,	char * buffer, loff_t offset, size_t count)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint8_t))
    {
        WARNING("STK ALS Driver : Error --> Read als_enable_bin size !=1\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    buffer[0] = pStkAlsData->bThreadRunning?1:0;
    STK_LOCK(0);
    return sizeof(uint8_t);
}

ssize_t  als_enable_write(struct kobject *kobj, struct bin_attribute *bin_attr,char * buffer, loff_t offset, size_t count)
{
#if __DEBUG_SYSFS_BIN
    INFO("STK ALS Driver : Enable ALS : %d\n",(int32_t)(buffer[0]));
#endif
    STK_LOCK(1);
    enable_als(buffer[0]);
    STK_LOCK(0);
    return count;
}

ssize_t als_delay_read(struct kobject *kobj, struct bin_attribute *bin_attr,	char * buffer, loff_t offset, size_t count)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK ALS Driver : Error --> Read als_delay size !=4\n");
        return 0;
    }
#endif
    STK_LOCK(1);
    *((uint32_t*)buffer) = pStkAlsData->als_delay;
    STK_LOCK(0);
    return sizeof(uint32_t);
}

ssize_t als_delay_write(struct kobject *kobj, struct bin_attribute *bin_attr,char * buffer, loff_t offset, size_t count)
{
    uint32_t delay;
#if __DEBUG_SYSFS_BIN
    INFO("STK ALS Driver : Set ALS Delay: %d\n",*((int32_t*)buffer));
#endif
    delay = *((uint32_t*)buffer);
    if (delay<ALS_MIN_DELAY)
        delay = ALS_MIN_DELAY;
    STK_LOCK(1);
    pStkAlsData->als_delay = delay;
    STK_LOCK(0);
    return count;
}

ssize_t als_min_delay_read(struct kobject *kobj, struct bin_attribute *bin_attr, char * buffer, loff_t offset, size_t count)
{
#if __DEBUG_SYSFS_BIN
    if (count != sizeof(uint32_t))
    {
        WARNING("STK ALS Driver : Error --> Read als_min_delay size !=4\n");
        return 0;
    }
#endif
    *((uint32_t*)buffer) = ALS_MIN_DELAY;
    return sizeof(uint32_t);
}

#ifdef CONFIG_STK_SYSFS_DBG
/* Only for debug */
static struct kobj_attribute help_attribute = (struct kobj_attribute)__ATTR_RO(help);
static struct kobj_attribute lux_range_attribute = (struct kobj_attribute)__ATTR_RO(lux_range);
static struct kobj_attribute lux_attribute = (struct kobj_attribute)__ATTR_RO(lux);
static struct kobj_attribute als_enable_attribute = (struct kobj_attribute)__ATTR(als_enable,0666,als_enable_show,als_enable_store);
#endif //CONFIG_STK_SYSFS_DBG

static struct bin_attribute als_lux_range_bin_attribute = __ATTR_BIN_RO(lux_range,als_lux_range_read,sizeof(uint32_t));
static struct bin_attribute als_lux_bin_attribute = __ATTR_BIN_RO(lux,lux_bin_read,sizeof(uint32_t));
static struct bin_attribute als_lux_res_bin_attribute = __ATTR_BIN_RO(lux_resolution,als_lux_resolution_read,sizeof(uint32_t));
static struct bin_attribute als_enable_bin_attribute = __ATTR_BIN_RW(als_enable,als_enable_read,als_enable_write,sizeof(uint8_t));
static struct bin_attribute als_delay_bin_attribute = __ATTR_BIN_RW(als_delay,als_delay_read,als_delay_write,sizeof(uint32_t));
static struct bin_attribute als_min_delay_bin_attribute = __ATTR_BIN_RO(als_min_delay,als_min_delay_read,sizeof(uint32_t));

#ifdef CONFIG_STK_SYSFS_DBG

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
#pragma message("Enable STK ALS Transmittance Tuning w/ SYSFS")
static ssize_t als_transmittance_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int32_t transmittance;
    STK_LOCK(1);
    transmittance = als_transmittance;
    STK_LOCK(0);
    return sprintf(buf, "%d\n", transmittance);
}

static ssize_t als_transmittance_store(struct kobject *kobj,
                                       struct kobj_attribute *attr,
                                       const char *buf, size_t len)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);
    STK_LOCK(1);
    als_transmittance = value;
    STK_LOCK(0);
    return len;
}

static struct kobj_attribute als_transmittance_attribute = (struct kobj_attribute)__ATTR(als_transmittance,0666,als_transmittance_show,als_transmittance_store);
#endif // CONFIG_STK_ALS_TRANSMITTANCE_TUNING


#endif //CONFIG_STK_SYSFS_DBG

#ifdef CONFIG_STK_SYSFS_DBG
static struct attribute* sensetek_optical_sensors_attrs [] =
{
    &help_attribute.attr,
    &lux_range_attribute.attr,
    &lux_attribute.attr,
    &als_enable_attribute.attr,

#ifdef CONFIG_STK_ALS_TRANSMITTANCE_TUNING
    &als_transmittance_attribute.attr,
#endif
    NULL,
};
// those attributes are only for engineer test/debug
static struct attribute_group sensetek_optics_sensors_attrs_group =
{
    .name = "DBG",
    .attrs = sensetek_optical_sensors_attrs,
};
#endif //CONFIG_STK_SYSFS_DBG


static struct bin_attribute* sensetek_optical_sensors_bin_attrs[] =
{
    &als_lux_range_bin_attribute,
    &als_lux_bin_attribute,
    &als_lux_res_bin_attribute,
    &als_enable_bin_attribute,
    &als_delay_bin_attribute,
    &als_min_delay_bin_attribute,
    NULL,
};


static struct platform_device *stk_oss_dev = NULL; /* Device structure */

static int stk_sysfs_create_bin_files(struct kobject *kobj,struct bin_attribute** bin_attrs)
{
    int err;
    while(*bin_attrs!=NULL)
    {
        err = sysfs_create_bin_file(kobj,*bin_attrs);
        if (err)
            return err;
        bin_attrs++;
    }
    return 0;
}

static void stk_sysfs_remove_bin_files(struct kobject *kobj,struct bin_attribute** bin_attrs)
{
    while(*bin_attrs!=NULL)
    {
        sysfs_remove_bin_file(kobj,*bin_attrs);
        bin_attrs++;
    }
}

/*
 * Called when a stk als device is matched with this driver
 */
static int stk_als_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	int err;
	int32_t nRead;
	struct stkals22x7_data*  als_data;
	INFO("STKALS -- %s: I2C is probing (%s)%d\n", __func__,id->name,(int)id->driver_data);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		ERR("STKALS -- No Support for I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}
	if (id->driver_data == 1)
	{
		nRead = stk_readb(client);
		if (nRead<0)
		{
			ERR("STKALS : no device found\n");
			return -ENODEV;
		}
		als_data = kzalloc(sizeof(struct stkals22x7_data),GFP_KERNEL);
		als_data->client1 = client;
		i2c_set_clientdata(client,als_data);
		mutex_init(&stkals_io_lock);

		pStkAlsData = als_data;
		als_data->it = 3;
		als_data->gain = 2;
		als_data->reg1 = 75;
		als_data->reg2 = 0x40;
		return 0;
	}

	if (pStkAlsData)
	{
		pStkAlsData->client2 = client;
		i2c_set_clientdata(client,pStkAlsData);
		pStkAlsData->als_delay = ODR_DELAY;
		init_all_setting();

		pStkAlsData->input_dev = input_allocate_device();
		if (pStkAlsData->input_dev==NULL)
		{
            mutex_destroy(&stkals_io_lock);
            kfree(pStkAlsData);
            return -ENOMEM;
		}
		pStkAlsData->input_dev->name = ALS_NAME;
		set_bit(EV_ABS, pStkAlsData->input_dev->evbit);
        input_set_abs_params(pStkAlsData->input_dev, ABS_MISC, 0, alscode2lux((1<<16)-1), 0, 0);
        err = input_register_device(pStkAlsData->input_dev);
        if (err<0)
        {
            ERR("STK ALS : can not register als input device\n");
            mutex_destroy(&stkals_io_lock);
            input_free_device(pStkAlsData->input_dev);
            kfree(pStkAlsData);
            return err;
        }
        INFO("STK ALS : register als input device OK\n");
	}
	return 0;


}

static int stk_als_remove(struct i2c_client *client)
{
	mutex_destroy(&stkals_io_lock);
	if (pStkAlsData)
	{
        input_unregister_device(pStkAlsData->input_dev);
        input_free_device(pStkAlsData->input_dev);
		kfree(pStkAlsData);
		pStkAlsData = 0;
	}
	return 0;
}

static const struct i2c_device_id stk_als_id[] = {
	{ "stk_als22x7_addr1", 1},
	{ "stk_als22x7_addr2", 2},
	{}
};
MODULE_DEVICE_TABLE(i2c, stk_als_id);

static struct i2c_driver stk_als_driver = {
	.driver = {
		.name = STKALS_DRV_NAME,
	},
	.probe = stk_als_probe,
	.remove = stk_als_remove,
	.id_table = stk_als_id,
};

static int __init stk_i2c_als22x7_init(void)
{
    int ret;
    ret = i2c_add_driver(&stk_als_driver);
    if (ret)
        return ret;
    if (pStkAlsData == NULL)
        return -EINVAL;
    stk_oss_dev = platform_device_alloc(DEVICE_NAME,-1);
    if (!stk_oss_dev)
    {
       i2c_del_driver(&stk_als_driver);
       return -ENOMEM;
    }
    if (platform_device_add(stk_oss_dev))
    {
       platform_device_put(stk_oss_dev);
       i2c_del_driver(&stk_als_driver);
       return -ENOMEM;
    }
    ret = stk_sysfs_create_bin_files(&(stk_oss_dev->dev.kobj),sensetek_optical_sensors_bin_attrs);
    if (ret)
    {
        platform_device_put(stk_oss_dev);
        i2c_del_driver(&stk_als_driver);
        return -ENOMEM;
    }
#ifdef CONFIG_STK_SYSFS_DBG
    ret = sysfs_create_group(&(stk_oss_dev->dev.kobj), &sensetek_optics_sensors_attrs_group);
    if (ret)
    {
        platform_device_put(stk_oss_dev);
        i2c_del_driver(&stk_als_driver);
        return -ENOMEM;
    }
#endif
	INFO("STK ALS Module initialized.\n");
    return 0;
}

static void __exit stk_i2c_als22x7_exit(void)
{
	i2c_del_driver(&stk_als_driver);
	pStkAlsData = NULL;

}

MODULE_AUTHOR("Patrick Chang <patrick_chang@sitronix.com>");
MODULE_DESCRIPTION("SenseTek Ambient Light Sensor driver");
MODULE_LICENSE("GPL");

module_init(stk_i2c_als22x7_init);
module_exit(stk_i2c_als22x7_exit);
