/**
 * twl4030-pwrbutton.c - TWL4030 Power Button Input Driver
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 * Several fixes by Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/metricslog.h>
#include <linux/jiffies.h>
#include <linux/leds.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <plat/led.h>
#define PWR_PWRON_IRQ (1 << 0)
#define PWR_PWRON_FORCE (1 << 7)


struct input_dev *g_pwr ;

static struct delayed_work pwrbutton_work;
static struct wake_lock pwrbutton_wakelock;

typedef struct{
	struct work_struct pwr_btn_bh;
	struct input_dev *pwr;
	u8 state;
	u8 g_in_suspend;
	u8 irq_g_in_suspend;
}pbut;

static pbut otter_button;

static irqreturn_t powerbutton_irq(int irq, void *_pwr){
	otter_button.irq_g_in_suspend = otter_button.g_in_suspend;
	struct input_dev *pwr = _pwr;
	otter_button.pwr = _pwr;
	u8 val;
	int err;

#ifdef CONFIG_LOCKDEP
	printk("------------->%s Entered\n", __func__);
	/* WORKAROUND for lockdep forcing IRQF_DISABLED on us, which
	* we don't want and can't tolerate.  Although it might be
	* friendlier not to borrow this thread context...*/
	local_irq_enable();
#endif

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &val ,REG_STS_HW_CONDITIONS);	//read state
	if (!err)  {
		otter_button.state = !(val & PWR_PWRON_IRQ);
		if (otter_button.irq_g_in_suspend && !otter_button.state){
			input_report_key(pwr, KEY_POWER, 1);
			otter_button.g_in_suspend = 0;
			otter_button.state |= PWR_PWRON_FORCE;				//set force marker
		}
		input_report_key(pwr, KEY_POWER, otter_button.state & PWR_PWRON_IRQ);	//report state
                input_sync(pwr);
		wake_lock(&pwrbutton_wakelock);						//lok us awake! for the BH
		schedule_work(&otter_button.pwr_btn_bh);					//schedule the BH
	}
     	else {
                dev_err(otter_button.pwr->dev.parent, "twl4030: i2c error %d while reading"
                        " TWL4030 PM_MASTER STS_HW_CONDITIONS register\n", err);
        }
        printk("------------->%s Left\n", __func__);

        return IRQ_HANDLED;
}

static void pwr_btn_bh_work_func(struct work_struct *work) {

	u8 value;
	printk("------------->%s Entered\n", __func__);
	char buf[128];
	char *action = otter_button.state ? "press" : "release";
	printk("press=%d;\n", otter_button.state);
	sprintf(buf, "%s:powi%c:action=%s:", __func__, action[0], action);
	log_to_metrics(ANDROID_LOG_INFO, "PowerKeyEvent", buf);
	strcat(buf, "\n");
	printk(buf);

	if (otter_button.state){
		/* Check if we are connected to USB */
		if (!twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, 0x03) && !(value & (1 << 2))) {
				/*
			 * No USB, hold a partial wakelock,
			 * scheduled a work 2 seconds later
			 * to switch off the LED
			 */
			cancel_delayed_work_sync(&pwrbutton_work);
			omap4430_orange_led_set(NULL, 0);
			omap4430_green_led_set(NULL, 255);
			schedule_delayed_work(&pwrbutton_work,
			msecs_to_jiffies(2000));
		}
	}
	else{
		if (!delayed_work_pending(&pwrbutton_work)){
			wake_unlock(&pwrbutton_wakelock);
		}
	}
	printk("------------->%s Left\n", __func__);
}
void twl_system_poweroff(void)
{
    printk("          system_poweroff\n ");
    //twl_i2c_write_u8(0x0d, 0x40, 0x25);//reboot
    twl_i2c_write_u8(0x0d, 0x07, 0x25);//shutdown
}

static void pwrbutton_work_func(struct work_struct *work)
{
	u8 value = 0;

	/* Switch off LED if it's not connected to USB */
	if (!twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &value, 0x03)
			&& !(value & (1 << 2))) {
		omap4430_orange_led_set(NULL, 0);
		omap4430_green_led_set(NULL, 0);
	}

	wake_unlock(&pwrbutton_wakelock);
}
static int twl4030_pwrbutton_suspend(struct platform_device *pdev)
{
	otter_button.g_in_suspend = 1;
    	return 0;
}
static int twl4030_pwrbutton_resume(struct platform_device *pdev)
{
	otter_button.g_in_suspend = 0;
    	return 0;
}
static int __devinit twl4030_pwrbutton_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int irq = platform_get_irq(pdev, 0);
	int err;
	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}
	otter_button.g_in_suspend = 0;
	pwr->evbit[0] = BIT_MASK(EV_KEY);
	pwr->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	pwr->name = "twl4030_pwrbutton";
	pwr->phys = "twl4030_pwrbutton/input0";
	pwr->dev.parent = &pdev->dev;
	twl6030_interrupt_unmask(0x03,REG_INT_MSK_LINE_A);
	twl6030_interrupt_unmask(0x03,REG_INT_MSK_STS_A);
	err = request_threaded_irq(irq, NULL, powerbutton_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"twl4030_pwrbutton", pwr);
	if (err < 0) {
		dev_dbg(&pdev->dev, "Can't get IRQ for pwrbutton: %d\n", err);
		goto free_input_dev;
	}
        g_pwr=pwr;
	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power button: %d\n", err);
		goto free_irq;
	}
	pm_power_off = twl_system_poweroff;
	platform_set_drvdata(pdev, pwr);

	/* Set up wakelock */
	wake_lock_init(&pwrbutton_wakelock, WAKE_LOCK_SUSPEND, "twl4030-pwrbutton");

	INIT_WORK(&otter_button.pwr_btn_bh, pwr_btn_bh_work_func);
	/* Set up delayed work */
	INIT_DELAYED_WORK(&pwrbutton_work, pwrbutton_work_func);

	return 0;

free_irq:
	free_irq(irq, NULL);
free_input_dev:
	input_free_device(pwr);
	return err;
}

static int __devexit twl4030_pwrbutton_remove(struct platform_device *pdev)
{
	struct input_dev *pwr = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, pwr);
	input_unregister_device(pwr);

	return 0;
}

struct platform_driver twl4030_pwrbutton_driver = {
	.probe		= twl4030_pwrbutton_probe,
	.remove		= __devexit_p(twl4030_pwrbutton_remove),
	.suspend	=	twl4030_pwrbutton_suspend,
	.resume		=	twl4030_pwrbutton_resume,
	.driver		= {
		.name	= "twl4030_pwrbutton",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_pwrbutton_init(void)
{
	return platform_driver_register(&twl4030_pwrbutton_driver);
}
module_init(twl4030_pwrbutton_init);

static void __exit twl4030_pwrbutton_exit(void)
{
	platform_driver_unregister(&twl4030_pwrbutton_driver);
}
module_exit(twl4030_pwrbutton_exit);

MODULE_ALIAS("platform:twl4030_pwrbutton");
MODULE_DESCRIPTION("Triton2 Power Button");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver <peter.de-schrijver@nokia.com>");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

