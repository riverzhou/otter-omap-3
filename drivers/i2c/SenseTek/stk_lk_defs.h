/*
 * drivers/i2c/chips/SenseTek/stk_lk_defs.h
 * Basic Defines for Linux Kernel Driver
 *
 */

#ifndef __STK__LK_DEFS_H
#define __STK__LK_DEFS_H

#define ERR(format, args...) \
	printk(KERN_ERR "%s: " format, DEVICE_NAME, ## args)
#define WARNING(format, args...) \
	printk(KERN_WARNING "%s: " format, DEVICE_NAME, ## args)
#ifdef CONFIG_STK_SHOW_INFO
#define INFO(format, args...) \
	printk(KERN_INFO "%s: " format, DEVICE_NAME, ## args)
#else
#define INFO(format,args...)
#endif

#define __ATTR_BIN(_name,_mode,_read,_write,_size) { \
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.read	= _read,					\
	.write	= _write,					\
	.size = _size,                      \
}
#define __ATTR_BIN_RO(_name,_read,_size) __ATTR_BIN(_name,0444,_read,NULL,_size)
#define __ATTR_BIN_RW(_name,_read,_write,_size) __ATTR_BIN(_name,0666,_read,_write,_size)

#define STK_LOCK(x) STK_LOCK##x

#endif // __STK__LK_DEFS_H
