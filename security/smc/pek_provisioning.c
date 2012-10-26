#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <media/provisioning.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <mach/omap4-common.h>

#define DEVICE_NAME "provisioning"


/* PPA encryption API ids */
#define PPA_SERV_HAL_HDCP_KEY_ENCRYPT 0x0000002C
#define PPA_SERV_HAL_PEK_KEY_ENCRYPT 0x0000002D
#define FLAG_START_HAL_CRITICAL     0x4

#define OMAP4_SRAM_VA           0xfe40d000
#define OMAP4_SRAM_PA		0x4030d000

struct ppa_pek_encrypt_control {
	u8 *in_key;
	u8 *out_key;
	uint32_t magic_key;
};

static struct mutex prov_lock;
static struct miscdevice mdev;

#define DEBUG

/*-----------------------------------------------------------------------------
 * Function: pek_encrypt_key_ctl
 *-----------------------------------------------------------------------------
 */
static long pek_encrypt_key_ctl(void __user *argp)
{
	struct pek_encrypt_control *ctrl;
	struct ppa_pek_encrypt_control *pa_ctrl;
	u8 *in_key_ppa;
	u8 *out_key_ppa;
	int ret;
#ifdef CONFIG_SMP
	long ret_affinity;
	cpumask_t saved_cpu_mask;
	cpumask_t local_cpu_mask = CPU_MASK_NONE;
#endif
	int i;
	u32 *dbg_ptr;

#ifdef DEBUG
	printk("***** provisioning_ioctl() - PEK ENCRYPT KEY *****\n");
#endif

	mutex_lock(&prov_lock);

#ifdef CONFIG_SMP
	/* Ensure secure call is made from CPU0 */
	cpu_set(0, local_cpu_mask);
	sched_getaffinity(0, &saved_cpu_mask);
	ret_affinity = sched_setaffinity(0, &local_cpu_mask);
	if(ret_affinity != 0)
		printk(KERN_ERR "sched_setaffinity #1 -> 0x%lX", ret_affinity);
#endif

	/* Encryption happens in ioctl / user context */
	ctrl = kmalloc(sizeof(struct pek_encrypt_control),
		       GFP_KERNEL);

	if(ctrl == 0) {
		printk(KERN_WARNING "Provisioning: Cannot allocate memory "
				    "for PEK encryption control struct\n");
#ifdef CONFIG_SMP
		ret_affinity = sched_setaffinity(0, &saved_cpu_mask);
		if(ret_affinity != 0)
			printk(KERN_ERR "sched_setaffinity #2 -> 0x%lX", ret_affinity);
#endif
		mutex_unlock(&prov_lock);
		return -EFAULT;
	}

	if(copy_from_user(ctrl, argp,
				sizeof(struct pek_encrypt_control))) {
		printk(KERN_WARNING "Provisioning: Error copying from "
				    " user space - encrypt ioctl\n");
		kfree(ctrl);
		mutex_unlock(&prov_lock);
		return -EFAULT;
	}
 
	/* Need an uncached region to pass data to PPA: SRAM will do */
	pa_ctrl = (struct ppa_pek_encrypt_control *)OMAP4_SRAM_VA;

	in_key_ppa = (u8 *)(OMAP4_SRAM_VA + sizeof(struct ppa_pek_encrypt_control));	
	out_key_ppa = (u8 *)(in_key_ppa + (sizeof(uint32_t)*PEK_KEY_SIZE));

	memcpy(in_key_ppa, ctrl->in_key, sizeof(uint32_t)*PEK_KEY_SIZE);

#if 0
	dbg_ptr = (u32*)in_key_ppa;
	for (i=0;i<8;i++) {
		printk("PEK in_key[%d]: 0x%08x\n", i, dbg_ptr[i]);
	}
#endif

	/* Call encrypt function */
        pa_ctrl->in_key = (u8*)(OMAP4_SRAM_PA + sizeof(struct ppa_pek_encrypt_control));
        pa_ctrl->out_key = (u8*)(pa_ctrl->in_key + (sizeof(uint32_t)*PEK_KEY_SIZE));
        pa_ctrl->magic_key = ctrl->magic_key;

#if 0
	printk("VA: pa_ctrl 0x%08x, in_key 0x%08x, out_key: 0x%08x, magic_key: %d\n",
			(u32)pa_ctrl, (u32)in_key_ppa, (u32)out_key_ppa, ctrl->magic_key);

	printk("PA: in_key 0x%08x, out_key: 0x%08x\n",
		(u32)pa_ctrl->in_key, (u32)pa_ctrl->out_key);
#endif

	/*
	 * Call the PEK KEK encryption PPA
	 */
	ret = omap4_secure_dispatcher(PPA_SERV_HAL_PEK_KEY_ENCRYPT,
			FLAG_START_HAL_CRITICAL, 1,
			OMAP4_SRAM_PA, 0, 0, 0);


#ifdef DEBUG
	printk("%s done with PEK encryption (result=%d)\n", __func__ , ret);
#endif

	mutex_unlock(&prov_lock);

#if 0
	if(!ret) {
		dbg_ptr = (u32 *)out_key_ppa;
		for (i=0;i<8;i++) {
			printk("PEK out_key[%d]: 0x%08x\n", i, dbg_ptr[i]);
		}
	}
#endif

//for(i=0; i < sizeof(uint32_t)*PEK_KEY_SIZE; i++) out_key_ppa[i] = i;

	/* Store output data to output pointer */
	if(copy_to_user(ctrl->out_key, out_key_ppa,
				sizeof(uint32_t)*PEK_KEY_SIZE)) {
		printk(KERN_WARNING "Provisioning: Error copying to user space"
				    " - encrypt ioctl\n");
		kfree(ctrl);
		return -EFAULT;
	}

	/* Clean up the area for further usage */
	memset(out_key_ppa, 0, sizeof(uint32_t)*PEK_KEY_SIZE);

#ifdef CONFIG_SMP
	ret_affinity = sched_setaffinity(0, &saved_cpu_mask);
	if(ret_affinity != 0)
		printk(KERN_ERR "sched_setaffinity #2 -> 0x%lX", ret_affinity);
#endif

	kfree(ctrl);
	return 0;
}


static long provisioning_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch(cmd)
	{
		case PROVISIONING_PEK_ENCRYPT_KEY:
			return pek_encrypt_key_ctl(argp);
		break;
		default:
		break;
	}

	return -ENOTTY;
}


static const struct file_operations provisioning_fops = {
	.unlocked_ioctl = provisioning_ioctl,
};

static int __init provisioning_init(void)
{
#ifdef DEBUG
	printk("***** provisioning_init() *****\n");
#endif

	mdev.minor = MISC_DYNAMIC_MINOR;
	mdev.name = DEVICE_NAME;
	mdev.mode = 0644;
	mdev.fops = &provisioning_fops;

	mutex_init(&prov_lock);

	if(misc_register(&mdev)) {
		printk(KERN_ERR "Provisioning: Could not add character driver\n");
		goto err_register;
	}

	return 0;

err_register:
	mutex_destroy(&prov_lock);
	return -EFAULT;
}

static void __exit provisioning_exit(void)
{
#ifdef DEBUG
	printk("***** provisioning_exit() *****\n");
#endif

	misc_deregister(&mdev);
	mutex_destroy(&prov_lock);
}

module_init(provisioning_init);
module_exit(provisioning_exit);
MODULE_LICENSE("GPL");

