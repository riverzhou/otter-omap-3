/*
 * LED Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include "leds.h"
#include <linux/thermal_framework.h>
#ifdef CONFIG_LAB126
#include <linux/metricslog.h>
#define THERMO_METRICS_STR_LEN 128
#endif

#define THERMAL_PREFIX "Thermal Policy: brightness agent: "
#define THERMAL_INFO(fmt, args...) do { printk(KERN_INFO THERMAL_PREFIX fmt, ## args); } while(0)

#ifdef THERMAL_DEBUG
#define THERMAL_DBG(fmt, args...) do { printk(KERN_DEBUG THERMAL_PREFIX fmt, ## args); } while(0)
#else
#define THERMAL_DBG(fmt, args...) do {} while(0)
#endif

static struct class *leds_class;

static void led_update_brightness(struct led_classdev *led_cdev)
{
	if (led_cdev->brightness_get)
		led_cdev->brightness = led_cdev->brightness_get(led_cdev);
}

static ssize_t led_brightness_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	/* no lock needed for this */
	led_update_brightness(led_cdev);

	return sprintf(buf, "%u\n", led_cdev->brightness);
}

static ssize_t led_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (state == LED_OFF)
			led_trigger_remove(led_cdev);
		led_set_brightness(led_cdev, state);
		/* save requested brightness even though it is greater than max_thermal_brightness */
		/* so, when system cools down, brightness will be set to saved_brightness. */
		led_cdev->saved_brightness = state;

	}
	return ret;
}

static ssize_t led_max_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", led_cdev->max_brightness);
}

static ssize_t led_show_max_thermal_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", led_cdev->max_thermal_brightness);
}

static struct device_attribute led_class_attrs[] = {
	__ATTR(brightness, 0644, led_brightness_show, led_brightness_store),
	__ATTR(max_brightness, 0444, led_max_brightness_show, NULL),
#ifdef CONFIG_LEDS_TRIGGERS
	__ATTR(trigger, 0644, led_trigger_show, led_trigger_store),
#endif
	__ATTR(max_thermal_brightness, 0444, led_show_max_thermal_brightness, NULL),
	__ATTR_NULL,
};

static void led_timer_function(unsigned long data)
{
	struct led_classdev *led_cdev = (void *)data;
	unsigned long brightness;
	unsigned long delay;

	if (!led_cdev->blink_delay_on || !led_cdev->blink_delay_off) {
		led_set_brightness(led_cdev, LED_OFF);
		return;
	}

	brightness = led_get_brightness(led_cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = led_cdev->blink_brightness;
		delay = led_cdev->blink_delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		led_cdev->blink_brightness = brightness;
		brightness = LED_OFF;
		delay = led_cdev->blink_delay_off;
	}

	led_set_brightness(led_cdev, brightness);

	mod_timer(&led_cdev->blink_timer, jiffies + msecs_to_jiffies(delay));
}

static void led_stop_software_blink(struct led_classdev *led_cdev)
{
	/* deactivate previous settings */
	del_timer_sync(&led_cdev->blink_timer);
	led_cdev->blink_delay_on = 0;
	led_cdev->blink_delay_off = 0;
}

static void led_set_software_blink(struct led_classdev *led_cdev,
				   unsigned long delay_on,
				   unsigned long delay_off)
{
	int current_brightness;

	current_brightness = led_get_brightness(led_cdev);
	if (current_brightness)
		led_cdev->blink_brightness = current_brightness;
	if (!led_cdev->blink_brightness)
		led_cdev->blink_brightness = led_cdev->max_brightness;

	if (led_get_trigger_data(led_cdev) &&
	    delay_on == led_cdev->blink_delay_on &&
	    delay_off == led_cdev->blink_delay_off)
		return;

	led_stop_software_blink(led_cdev);

	led_cdev->blink_delay_on = delay_on;
	led_cdev->blink_delay_off = delay_off;

	/* never on - don't blink */
	if (!delay_on)
		return;

	/* never off - just set to brightness */
	if (!delay_off) {
		led_set_brightness(led_cdev, led_cdev->blink_brightness);
		return;
	}

	mod_timer(&led_cdev->blink_timer, jiffies + 1);
}


/**
 * led_classdev_suspend - suspend an led_classdev.
 * @led_cdev: the led_classdev to suspend.
 */
void led_classdev_suspend(struct led_classdev *led_cdev)
{
	led_cdev->flags |= LED_SUSPENDED;
	led_cdev->brightness_set(led_cdev, 0);
}
EXPORT_SYMBOL_GPL(led_classdev_suspend);

/**
 * led_classdev_resume - resume an led_classdev.
 * @led_cdev: the led_classdev to resume.
 */
void led_classdev_resume(struct led_classdev *led_cdev)
{
	led_cdev->brightness_set(led_cdev, led_cdev->brightness);
	led_cdev->flags &= ~LED_SUSPENDED;
}
EXPORT_SYMBOL_GPL(led_classdev_resume);

static int led_suspend(struct device *dev, pm_message_t state)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_suspend(led_cdev);

	return 0;
}

static int led_resume(struct device *dev)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_resume(led_cdev);

	return 0;
}

static int backlight_apply_cooling(struct thermal_dev *dev,
				int level)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev->dev);
	int brightness;
	int percent;
	static int previous_cooling_level = 0, new_cooling_level = 0;
#ifdef CONFIG_LAB126
	char *thermal_metric_prefix = "backlight_cooling:def:monitor=1";
	char buf[THERMO_METRICS_STR_LEN];
#endif

	/* transform into percentage */
	percent = thermal_cooling_device_reduction_get(dev, level);
	if (percent < 0 || percent > 100)
		return -EINVAL;
	brightness = (led_cdev->max_brightness * percent) / 100;

	down_write(&leds_list_lock);
	new_cooling_level = level;
	THERMAL_DBG("%s: previous_cooling_level %d, new_cooling_level %d , percent %d, currentbl %d, saved bl %d, thermalmax %lu " ,
	       __FUNCTION__, previous_cooling_level, new_cooling_level, led_cdev->brightness, led_cdev->saved_brightness, brightness );

	led_cdev->max_thermal_brightness = brightness;
	if ((new_cooling_level == 0) && (previous_cooling_level == 0) )  {
		/* reached level 0 without hitting throttling down the brightness */
		/* so no reason to change brightness */
		goto exit;

	} else if ( (new_cooling_level > previous_cooling_level) && (led_cdev->brightness < brightness) ) {
		/* device is heating up, but new max thermal brightness
		  * is greater than current brightness.
		  * so, we don't need to update the brightness.
		  */
		THERMAL_DBG("%s: device is heating up, but no change is brightness. ",__FUNCTION__);
		goto exit;
	}
	else if ( (new_cooling_level < previous_cooling_level) && (led_cdev->saved_brightness < brightness)) {
		/* devices is cooling down, and new max thermal brightness
		  * is greater than previously saved brightness.
		  * So, we need to increase brightness to the saved value not thermal max.
		  */
		led_cdev->brightness = led_cdev->saved_brightness;
		THERMAL_DBG("%s device is cooling down, restore saved brightness",__FUNCTION__);
	}
	else {
		THERMAL_INFO("brightness transition from %d to %d", led_cdev->brightness, brightness);
		led_cdev->brightness = brightness;
	}
#ifdef CONFIG_LAB126
	if ( previous_cooling_level == 0 ) {
		snprintf(buf, THERMO_METRICS_STR_LEN,"%s,throttling=%s:", thermal_metric_prefix, "start");
		log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
	} else if ( new_cooling_level == 0 ) {
		snprintf(buf, THERMO_METRICS_STR_LEN,"%s,throttling=%s:", thermal_metric_prefix, "stop");
		log_to_metrics(ANDROID_LOG_INFO, "ThermalEvent", buf);
	}
#endif
	led_set_brightness(led_cdev, led_cdev->brightness);
	//backlight_update_status(bd);
	//backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);

exit:
	up_write(&leds_list_lock);
	previous_cooling_level = new_cooling_level;
	return 0;
}

static struct thermal_dev_ops backlight_cooling_ops = {
	.cool_device = backlight_apply_cooling,
};

static struct thermal_dev case_thermal_dev = {
	.name		= "backlight_cooling",
	.domain_name	= "case",
	.dev_ops	= &backlight_cooling_ops,
};



/**
 * led_classdev_register - register a new object of led_classdev class.
 * @parent: The device to register.
 * @led_cdev: the led_classdev structure for this device.
 */
int led_classdev_register(struct device *parent, struct led_classdev *led_cdev)
{
	struct thermal_dev *tdev = NULL;
	int ret = 0;
	led_cdev->dev = device_create(leds_class, parent, 0, led_cdev,
				      "%s", led_cdev->name);
	if (IS_ERR(led_cdev->dev))
		return PTR_ERR(led_cdev->dev);
	if (!strcmp(led_cdev->name, "lcd-backlight")) {
		tdev = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
		if (!tdev) {
			device_unregister(led_cdev->dev);
			return -ENOMEM;
		}
	}
	//mutex_init(&led_cdev->new_bd->update_lock);
#ifdef CONFIG_LEDS_TRIGGERS
	init_rwsem(&led_cdev->trigger_lock);
#endif
	/* add to the list of leds */
	down_write(&leds_list_lock);
	list_add_tail(&led_cdev->node, &leds_list);
	up_write(&leds_list_lock);

	if (!led_cdev->max_brightness)
		led_cdev->max_brightness = LED_FULL;

	led_update_brightness(led_cdev);

	init_timer(&led_cdev->blink_timer);
	led_cdev->blink_timer.function = led_timer_function;
	led_cdev->blink_timer.data = (unsigned long)led_cdev;

#ifdef CONFIG_LEDS_TRIGGERS
	led_trigger_set_default(led_cdev);
#endif
	if (!strcmp(led_cdev->name, "lcd-backlight")) {
		memcpy(tdev, &case_thermal_dev, sizeof(struct thermal_dev));
		tdev->dev = led_cdev->dev;
		ret = thermal_cooling_dev_register(tdev);
		if (ret < 0) {
			device_unregister(led_cdev->dev);
			kfree(tdev);
			return ret;
		}
		led_cdev->tdev = tdev;
		led_cdev->max_thermal_brightness = led_cdev->max_brightness;
	}
	led_cdev->saved_brightness = 0;
	printk(KERN_DEBUG "Registered led device: %s\n",
			led_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(led_classdev_register);

/**
 * led_classdev_unregister - unregisters a object of led_properties class.
 * @led_cdev: the led device to unregister
 *
 * Unregisters a previously registered via led_classdev_register object.
 */
void led_classdev_unregister(struct led_classdev *led_cdev)
{
#ifdef CONFIG_LEDS_TRIGGERS
	down_write(&led_cdev->trigger_lock);
	if (led_cdev->trigger)
		led_trigger_set(led_cdev, NULL);
	up_write(&led_cdev->trigger_lock);
#endif

	/* Stop blinking */
	led_brightness_set(led_cdev, LED_OFF);
	thermal_cooling_dev_unregister(led_cdev->tdev);
	device_unregister(led_cdev->dev);

	down_write(&leds_list_lock);
	list_del(&led_cdev->node);
	up_write(&leds_list_lock);
}
EXPORT_SYMBOL_GPL(led_classdev_unregister);

void led_blink_set(struct led_classdev *led_cdev,
		   unsigned long *delay_on,
		   unsigned long *delay_off)
{
	if (led_cdev->blink_set &&
	    !led_cdev->blink_set(led_cdev, delay_on, delay_off))
		return;

	/* blink with 1 Hz as default if nothing specified */
	if (!*delay_on && !*delay_off)
		*delay_on = *delay_off = 500;

	led_set_software_blink(led_cdev, *delay_on, *delay_off);
}
EXPORT_SYMBOL(led_blink_set);

void led_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	led_stop_software_blink(led_cdev);
	led_cdev->brightness_set(led_cdev, brightness);
}
EXPORT_SYMBOL(led_brightness_set);

static int __init leds_init(void)
{
	leds_class = class_create(THIS_MODULE, "leds");
	if (IS_ERR(leds_class))
		return PTR_ERR(leds_class);
	leds_class->suspend = led_suspend;
	leds_class->resume = led_resume;
	leds_class->dev_attrs = led_class_attrs;
	return 0;
}

static void __exit leds_exit(void)
{
	class_destroy(leds_class);
}

subsys_initcall(leds_init);
module_exit(leds_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LED Class Interface");
